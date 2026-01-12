#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/dns.h"
#include "lwip/tcp.h"
#include <string.h>
#include <stdio.h>
#include "json/cJSON.h"
#include "hardware/pwm.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/i2c.h"
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include "wifi_config.h"
#include "flask_api.h"
#include "max30100.h"

struct tcp_pcb *client_pcb = NULL;
const uint led_pin_red = 13;
const uint led_pin_green = 11;

#define BUZZER_PIN 21        // Configuração do pino do buzzer
#define BUZZER_FREQUENCY 100 // Configuração da frequência do buzzer (em Hz)

// Variável global opcional para o buffer da resposta
#define RESPONSE_BUFFER_SIZE 2048
static char response_buffer[RESPONSE_BUFFER_SIZE];

// ----------------- Funções auxiliares I2C -----------------

/*
 * Escreve 1 byte (val) no registrador (reg) do MAX30102.
 * Retorna true se a escrita foi bem-sucedida.
 */
static bool wr8(uint8_t reg, uint8_t val)
{
    uint8_t b[2] = {reg, val};
    return i2c_write_blocking(I2C_PORT, MAX_ADDR, b, 2, false) == 2;
}

/*
 * Lê 1 byte do registrador (reg) do MAX30102 para *val.
 * Retorna true se leu com sucesso.
 */
static bool rd8(uint8_t reg, uint8_t *val)
{
    if (i2c_write_blocking(I2C_PORT, MAX_ADDR, &reg, 1, true) != 1)
        return false;
    return i2c_read_blocking(I2C_PORT, MAX_ADDR, val, 1, false) == 1;
}

/*
 * Lê N bytes começando em (reg) para o buffer buf.
 * Retorna true se a leitura foi bem-sucedida.
 */
static bool rdbuf(uint8_t reg, uint8_t *buf, size_t n)
{
    if (i2c_write_blocking(I2C_PORT, MAX_ADDR, &reg, 1, true) != 1)
        return false;
    return i2c_read_blocking(I2C_PORT, MAX_ADDR, buf, n, false) == (int)n;
}

// ----------------- Inicialização / reset do MAX30102 -----------------

/*
 * Reseta o MAX30102:
 *  - Escreve 0x40 em MODE_CFG para reset.
 *  - Espera 10 ms.
 *  - Escreve 0x00 para acordar.
 */
static void max_reset(void)
{
    wr8(REG_MODE_CFG, 0x40);
    sleep_ms(10);
    wr8(REG_MODE_CFG, 0x00);
    sleep_ms(10);
}

/*
 * Configuração inicial:
 *  - desabilita interrupções,
 *  - zera ponteiros da FIFO (evita lixo),
 *  - define média de 4 amostras na FIFO + rollover,
 *  - configura SPO2 (range + 100Hz + 18 bits),
 *  - acerta correntes dos LEDs,
 *  - coloca em modo SpO2 (RED+IR).
 */
static void max_init(uint8_t led_pa)
{
    max_reset();

    // Interrupções desabilitadas (usamos polling simples)
    wr8(REG_INT_EN1, 0x00);
    wr8(REG_INT_EN2, 0x00);

    // Limpa FIFO (ponteiros e contador de overflow)
    wr8(REG_FIFO_WR_PTR, 0x00);
    wr8(REG_OVF_CNT, 0x00);
    wr8(REG_FIFO_RD_PTR, 0x00);

    // FIFO_CFG:
    //  bits 7:5 = 0b010 => média de 4 amostras
    //  bit 4   = 1      => rollover habilitado (circular)
    wr8(REG_FIFO_CFG, (2 << 5) | (1 << 4));

    // SPO2_CFG: range ADC + SR=100 Hz + PW=18 bits
    wr8(REG_SPO2_CFG, SPO2_CFG_RANGE_SR_PW);

    // Correntes de LED (mesmo valor para RED e IR; ajuste se quiser)
    wr8(REG_LED1_PA, led_pa); // RED
    wr8(REG_LED2_PA, led_pa); // IR

    // MODE_CFG: 0x03 => modo SpO2 (LEDs RED+IR alternados)
    wr8(REG_MODE_CFG, 0x03);

    // Aguardar estabilização
    sleep_ms(100);
}

// ----------------- FIFO: disponibilidade e leitura de uma amostra -----------------

/*
 * Quantas amostras estão disponíveis na FIFO
 *  - Lê os ponteiros WR_PTR e RD_PTR e calcula diferença com wrap (5 bits).
 *  - A FIFO tem profundidade de 32 amostras (0..31).
 */
static inline uint8_t fifo_available(void)
{
    uint8_t wr, rd;
    rd8(REG_FIFO_WR_PTR, &wr);
    rd8(REG_FIFO_RD_PTR, &rd);
    return (uint8_t)((wr - rd) & 0x1F);
}

/*
 * Lê uma amostra (um par RED/IR) da FIFO:
 *  - O MAX30102 entrega 6 bytes por amostra (3 por canal, 18 bits).
 *  - Montagem: (b0<<16 | b1<<8 | b2) & 0x3FFFF.
 */
static inline bool read_sample(uint32_t *red, uint32_t *ir)
{
    uint8_t b[6];
    if (!rdbuf(REG_FIFO_DATA, b, 6))
        return false;

    *red = (((uint32_t)b[0] << 16) | ((uint32_t)b[1] << 8) | b[2]) & 0x3FFFF;
    *ir = (((uint32_t)b[3] << 16) | ((uint32_t)b[4] << 8) | b[5]) & 0x3FFFF;
    return true;
}

// ----------------- Estado e processamento (BPM / SpO2) -----------------

/*
 * Estrutura de estado:
 *  - dc_* e var_*: acumuladores para EMA (Exponential Moving Average - média móvel exponencial) da base (DC) e variância (≈RMS^2) do AC.
 *  - ma_*: média móvel de 5 pontos em AC do IR (suavização para detectar vales).
 *  - last_peak / rr_hist: memória para estimar BPM com média dos últimos RR.
 *  - finger_on: flag indicando dedo/pulso presente (DC/SNR dentro da faixa).
 *  - last_*: métricas de debug para imprimir (DC_IR, RMS_IR, SNR).
 *  - acs_prev*: histórico do AC suavizado para testar “mínimo local”.
 */
typedef struct
{
    // Filtros DC/AC (EMA) e variâncias
    float dc_red, dc_ir;
    float var_red, var_ir;

    // Média móvel (5 amostras) do AC do IR
    float ma_sum;
    float ma_buf[5];
    int ma_idx;

    // Ritmo cardíaco
    int last_peak; // índice da última batida (em amostras)
    float rr_hist[5];
    int rr_len;
    float bpm;

    // Estado: dedo presente?
    bool finger_on;

    // Debug/telemetria
    float last_rms_ir, last_snr, last_dc_ir;

    // Histórico para mínimo local (AC suavizado)
    float acs_prev2, acs_prev1;
} PulseState;

/* Inicializa a estrutura de estado com valores neutros. */
static void ps_init(PulseState *s)
{
    s->dc_red = s->dc_ir = 0.0f;
    s->var_red = s->var_ir = 1.0f; // evita zero ao tirar sqrt
    s->ma_sum = 0.0f;
    for (int i = 0; i < 5; i++)
        s->ma_buf[i] = 0.0f;
    s->ma_idx = 0;
    s->last_peak = -100000; // bem no passado
    s->rr_len = 0;
    s->bpm = 0.0f;
    s->finger_on = false;
    s->last_rms_ir = 0.0f;
    s->last_snr = 0.0f;
    s->last_dc_ir = 0.0f;
    s->acs_prev1 = s->acs_prev2 = 0.0f;
}

/*
 * process(...)
 *  - Entrada: amostras brutas RED/IR e o índice da amostra (idx).
 *  - Saída: valor de SpO2 estimado (didático) ou NAN se sem dedo.
 *
 * Passos:
 *  1) EMA rápida para DC e variância do AC (resposta ~0,3–0,5 s).
 *  2) RMS da AC e SNR = RMS/DC.
 *  3) Testa dedo/pulso (DC em faixa + SNR mínimo).
 *  4) Suaviza AC do IR (média móvel de 5 pts).
 *  5) Detecta batimento por *vale* (mínimo local) + amplitude mínima + refratário.
 *  6) Calcula RR, rejeita outliers relativos à média, estima BPM (média de 5 RR).
 *  7) Estima SpO2 por ratio-of-ratios (didático).
 */
static float process(PulseState *s, uint32_t red_raw, uint32_t ir_raw, int idx)
{
    // EMA mais rápida: a maior => resposta mais curta (menos atraso)
    const float a_dc = 0.03f;  // ~0,33 s com FS=100
    const float a_var = 0.03f; // janela semelhante para variância

    // Converte para float
    float red = (float)red_raw;
    float ir = (float)ir_raw;

    // 1) Estima DC (EMA) e extrai AC = x - DC
    s->dc_red += a_dc * (red - s->dc_red);
    s->dc_ir += a_dc * (ir - s->dc_ir);
    float ac_red = red - s->dc_red;
    float ac_ir = ir - s->dc_ir;

    // 2) Estima variância do AC (EMA) e RMS = sqrt(variância)
    s->var_red += a_var * ((ac_red * ac_red) - s->var_red);
    s->var_ir += a_var * ((ac_ir * ac_ir) - s->var_ir);

    float dc_ir = fmaxf(s->dc_ir, 1.0f); // evita dividir por 0
    float dc_redv = fmaxf(s->dc_red, 1.0f);
    float rms_ir = sqrtf(fmaxf(s->var_ir, 1.0f));
    float rms_red = sqrtf(fmaxf(s->var_red, 1.0f));
    float snr = rms_ir / dc_ir; // fração (ex.: 0,01 = 1%)

    // Guarda métricas para impressão
    s->last_dc_ir = dc_ir;
    s->last_rms_ir = rms_ir;
    s->last_snr = snr;

    // 3) Dedo/pulso presente? (DC dentro da faixa + SNR mínimo)
    bool dc_ok = (dc_ir > FINGER_DC_MIN && dc_ir < FINGER_DC_MAX &&
                  dc_redv > FINGER_DC_MIN && dc_redv < FINGER_DC_MAX);
    bool snr_ok = (snr >= FINGER_SNR_MIN);
    s->finger_on = dc_ok && snr_ok;

    // 4) Suavização do AC do IR via média móvel (5 amostras)
    s->ma_sum -= s->ma_buf[s->ma_idx];
    s->ma_buf[s->ma_idx] = ac_ir;
    s->ma_sum += s->ma_buf[s->ma_idx];
    s->ma_idx = (s->ma_idx + 1) % 5;
    float ac_s = s->ma_sum / 5.0f; // AC do IR suavizado

    // 5) Detecção de batimento por *mínimos* (vales), com refratário/outliers
    if (s->finger_on)
    {
        // Amplitude mínima do vale (fração do RMS da AC)
        float thr_vale = 0.45f * rms_ir;
        // Refratário em amostras
        int refractory = (int)(REFRACTORY_S * FS_HZ);

        // “Mínimo local” no ponto anterior (acs_prev1)?
        bool is_min_local = (s->acs_prev1 < s->acs_prev2) && (s->acs_prev1 <= ac_s);
        bool amp_ok = (-s->acs_prev1) > thr_vale; // vale “fundo” suficiente

        // Se for um vale válido e respeita refratário, computa RR
        if (is_min_local && amp_ok && (idx - s->last_peak) > refractory)
        {
            int diff = idx - s->last_peak;
            float rr = diff / FS_HZ; // intervalo em segundos
            s->last_peak = idx;

            // RR dentro de limites físicos plausíveis?
            if (rr > RR_MIN_S && rr < RR_MAX_S)
            {
                // Calcula média dos RR anteriores (aprox. de mediana)
                float mean = rr;
                if (s->rr_len)
                {
                    float soma = 0.0f;
                    for (int i = 0; i < s->rr_len; i++)
                        soma += s->rr_hist[i];
                    mean = soma / s->rr_len;
                }
                // Rejeita outliers relativos à média (evita “dobrar” batida)
                if (rr > RR_LOWER_FRAC * mean && rr < RR_UPPER_FRAC * mean)
                {
                    // Enfileira RR (buffer de até 5 amostras)
                    if (s->rr_len < 5)
                        s->rr_hist[s->rr_len++] = rr;
                    else
                    {
                        for (int i = 1; i < 5; i++)
                            s->rr_hist[i - 1] = s->rr_hist[i];
                        s->rr_hist[4] = rr;
                    }
                    // BPM pela média dos RR
                    float m = 0.0f;
                    for (int i = 0; i < s->rr_len; i++)
                        m += s->rr_hist[i];
                    m /= s->rr_len;
                    s->bpm = 60.0f / m;
                }
            }
        }
    }
    else
    {
        // Sem dedo/pulso: zera estimativa de ritmo
        s->bpm = 0.0f;
        s->rr_len = 0;
    }

    // 6) Estimativa didática de SpO2 (ratio-of-ratios) somente se finger_on
    float spo2 = NAN;
    if (s->finger_on)
    {
        // R = (AC_red/DC_red) / (AC_ir/DC_ir)
        float R = (rms_red / dc_redv) / (rms_ir / dc_ir);
        // Conversão linear de exemplo (didática, não clínica)
        spo2 = 110.0f - 25.0f * R;
        if (spo2 < 70.0f)
            spo2 = 70.0f;
        if (spo2 > 100.0f)
            spo2 = 100.0f;
    }

    // 7) Atualiza histórico para checar “mínimo local” na próxima iteração
    s->acs_prev2 = s->acs_prev1;
    s->acs_prev1 = ac_s;

    return spo2;
}

// Definição de uma função para inicializar o PWM no pino do buzzer
void pwm_init_buzzer(uint pin)
{
    // Configurar o pino como saída de PWM
    gpio_set_function(pin, GPIO_FUNC_PWM);

    // Obter o slice do PWM associado ao pino
    uint slice_num = pwm_gpio_to_slice_num(pin);

    // Configurar o PWM com frequência desejada
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, clock_get_hz(clk_sys) / (BUZZER_FREQUENCY * 4096)); // Divisor de clock
    pwm_init(slice_num, &config, true);

    // Iniciar o PWM no nível baixo
    pwm_set_gpio_level(pin, 0);
}

// Definição de uma função para emitir um beep com duração especificada
void beep(uint pin, uint duration_ms)
{
    // Obter o slice do PWM associado ao pino
    uint slice_num = pwm_gpio_to_slice_num(pin);

    // Configurar o duty cycle para 50% (ativo)
    pwm_set_gpio_level(pin, 2048);

    // Temporização
    sleep_ms(duration_ms);

    // Desativar o sinal PWM (duty cycle 0)
    pwm_set_gpio_level(pin, 0);

    // Pausa entre os beeps
    sleep_ms(100); // Pausa de 100ms
}

// Verifica status da conexão com a API Flask
static err_t client_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
    if (p == NULL)
    {
        // printf("Conexão encerrada pelo servidor\n");
        tcp_close(tpcb);
        return ERR_OK;
    }

    int len = p->len > RESPONSE_BUFFER_SIZE - 1 ? RESPONSE_BUFFER_SIZE - 1 : p->len;
    memcpy(response_buffer, p->payload, len);
    response_buffer[len] = '\0';

    printf("Conteúdo do JSON recebido:\n%s\n", response_buffer);

    cJSON *root = cJSON_Parse(response_buffer);
    cJSON *entry = NULL;
    cJSON *last_entry = NULL;

    cJSON_ArrayForEach(entry, root)
    {
        last_entry = entry; // sempre armazena o último objeto iterado
    }

    if (last_entry && cJSON_IsObject(last_entry))
    {
        cJSON *bpm = cJSON_GetObjectItemCaseSensitive(last_entry, "bpm");
        if (cJSON_IsNumber(bpm))
        {
            printf("Último BPM: %d\n", bpm->valueint);

            // Verifica se os batimentos cardíacos estão acima de 85 BPM
            if (bpm->valueint > 85)
            {
                gpio_put(led_pin_green, false);
                gpio_put(led_pin_red, true);
                beep(BUZZER_PIN, 2000);
            }
            else
            {
                gpio_put(led_pin_red, false);
                gpio_put(led_pin_green, true);
            }
        }
    }

    cJSON_Delete(root);
    pbuf_free(p);
    return ERR_OK;
}

// Variável global para armazenar o valor atual do sensor
static float current_bpm_value = 0;

static err_t client_connected(void *arg, struct tcp_pcb *tpcb, err_t err) {
    if (err != ERR_OK) return err;

    // 1. Cria o corpo do JSON com o BPM real
    char json_body[64];
    snprintf(json_body, sizeof(json_body), "{\"bpm\": %.1f}", current_bpm_value);

    // 2. Monta o cabeçalho HTTP POST
    char request[512];
    snprintf(request, sizeof(request),
             "POST %s HTTP/1.1\r\n"
             "Host: %s:%d\r\n"
             "Content-Type: application/json\r\n"
             "Content-Length: %d\r\n"
             "Connection: close\r\n\r\n"
             "%s",
             FLASK_ENDPOINT, FLASK_SERVER_IP, FLASK_SERVER_PORT, (int)strlen(json_body), json_body);

    // 3. Envia para a API Flask
    tcp_write(tpcb, request, strlen(request), TCP_WRITE_FLAG_COPY);
    tcp_output(tpcb);
    
    return ERR_OK;
}

// Chame esta função passando st.bpm do seu loop principal
void connect_to_flask(float bpm) {
    current_bpm_value = bpm;
    ip_addr_t server_ip;
    ipaddr_aton(FLASK_SERVER_IP, &server_ip);

    client_pcb = tcp_new();
    tcp_connect(client_pcb, &server_ip, FLASK_SERVER_PORT, client_connected);
}

int main()
{
    stdio_init_all();
    uint32_t last_api_send_time = 0;

    // Configura I2C a 100 kHz e põe os pinos na função I2C, com pull-ups internos
    i2c_init(I2C_PORT, 100 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
    sleep_ms(300); // pequeno atraso para estabilizar

    // Opcional: leitura do PART_ID (esperado 0x15)
    uint8_t part = 0;
    rd8(REG_PART_ID, &part);
    printf("PART_ID=0x%02X (esperado 0x15)\n", part);

    // Inicializa o MAX30102 com a corrente de LED escolhida
    max_init(LED_PA);

    // Estado do processamento
    PulseState st;
    ps_init(&st);

    // idx: contador de amostras (0,1,2,...) — usado para tempos e RR
    // ticker: imprime a cada ~1 s (conta regressiva de FS_HZ)
    int idx = 0, ticker = (int)FS_HZ;

    // Inicializando LEDs
    gpio_init(led_pin_red);
    gpio_set_dir(led_pin_red, GPIO_OUT);

    gpio_init(led_pin_green);
    gpio_set_dir(led_pin_green, GPIO_OUT);

    // Configuração do GPIO para o buzzer como saída
    gpio_init(BUZZER_PIN);
    gpio_set_dir(BUZZER_PIN, GPIO_OUT);

    // Inicializar o PWM no pino do buzzer
    pwm_init_buzzer(BUZZER_PIN);

    if (cyw43_arch_init())
    {
        printf("Erro ao iniciar Wi-Fi\n");
        return -1;
    }

    cyw43_arch_enable_sta_mode();

    printf("Conectando ao Wi-Fi...\n");

    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_MIXED_PSK, 30000))
    {
        printf("Falha ao conectar ao Wi-Fi\n");
        return -1;
    }

    printf("Conectado ao Wi-Fi com sucesso!\n");

    // Loop principal
   while (true)
{
    // 1. Manutenção do Wi-Fi (Sempre necessário para o stack TCP/IP)
    cyw43_arch_poll();

    // 2. Processamento do Sensor (Prioridade Máxima)
    uint8_t avail = fifo_available();
    if (avail > 0)
    {
        for (uint8_t i = 0; i < avail; i++)
        {
            uint32_t red, ir;
            if (!read_sample(&red, &ir)) break;

            float spo2 = process(&st, red, ir, idx++);

            // Ticker de 1 segundo para logs e Verificação de Alerta
            if (--ticker <= 0)
            {
                bool pulse_recent = (idx - st.last_peak) < (int)(FS_HZ * 2.0f);
                bool valid = st.finger_on && pulse_recent && (st.bpm >= 35.0f && st.bpm <= 200.0f);

                if (valid) {
                    printf("BPM=%5.1f  SpO2≈%5.1f%%\n", st.bpm, spo2);
                    
                    // Lógica de Alerta (LEDs e Buzzer)
                    if (st.bpm > 85.0f) {
                        gpio_put(led_pin_green, false);
                        gpio_put(led_pin_red, true);
                        beep(BUZZER_PIN, 500); // Beep curto para não travar muito o loop
                    } else {
                        gpio_put(led_pin_red, false);
                        gpio_put(led_pin_green, true);
                    }
                } else {
                    printf("[Aguardando dedo/pulso...]\n");
                    gpio_put(led_pin_red, false);
                    gpio_put(led_pin_green, false);
                }
                ticker = (int)FS_HZ;
            }
        }
    }

    // 3. Envio para API Flask/Firebase (A cada 15 segundos)
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    if (current_time - last_api_send_time > 15000)
    {
        // Só envia se houver uma leitura válida para não sujar o banco
        if (st.finger_on && st.bpm > 35.0f)
        {
            printf("\n--- Enviando BPM %.1f para o Firebase via Flask ---\n", st.bpm);
            connect_to_flask(st.bpm); 
        }
        last_api_send_time = current_time;
    }

    // Pequeno delay para aliviar a CPU sem perder amostras (opcional)
    sleep_ms(1);
}

    return 0;
}

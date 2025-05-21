#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/dns.h"
#include "lwip/tcp.h"
#include <string.h>
#include <stdio.h>
#include "cJSON.h"
#include "hardware/pwm.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/i2c.h"
#include "ssd1306_font.h"
#include "display_config.c"


#define WIFI_SSID "Jana Fonseca"
#define WIFI_PASSWORD "capricornio31"

// Altere para o IP da máquina onde roda sua API Flask
#define FLASK_SERVER_IP "192.168.3.143"
#define FLASK_SERVER_PORT 5000
#define FLASK_ENDPOINT "/"

struct tcp_pcb *client_pcb = NULL;

const uint led_pin_red = 13;
const uint led_pin_green = 11;

#define BUZZER_PIN 21 // Configuração do pino do buzzer
#define BUZZER_FREQUENCY 100 // Configuração da frequência do buzzer (em Hz)

// Variável global opcional para o buffer da resposta
#define RESPONSE_BUFFER_SIZE 2048
static char response_buffer[RESPONSE_BUFFER_SIZE];

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

//Verifica status da conexão com a API Flask
static err_t client_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    if (p == NULL) {
        //printf("Conexão encerrada pelo servidor\n");
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

    cJSON_ArrayForEach(entry, root) {
        last_entry = entry; // sempre armazena o último objeto iterado
    }

    if (last_entry && cJSON_IsObject(last_entry)) {
        cJSON *bpm = cJSON_GetObjectItemCaseSensitive(last_entry, "bpm");
        if (cJSON_IsNumber(bpm)) {
            printf("Último BPM: %d\n", bpm->valueint);

            //Verifica se os batimentos cardíacos estão acima de 85 BPM
            if(bpm->valueint > 85){
                gpio_put(led_pin_green, false);
                gpio_put(led_pin_red, true);
                beep(BUZZER_PIN, 2000);
            }else{
                gpio_put(led_pin_red, false);
                gpio_put(led_pin_green, true);
            }
            
        }
    }

    cJSON_Delete(root);
    pbuf_free(p);
    return ERR_OK;
}


static err_t client_connected(void *arg, struct tcp_pcb *tpcb, err_t err) {
    if (err != ERR_OK) {
        printf("Erro ao conectar: %d\n", err);
        return err;
    }

    printf("Conectado ao servidor Flask!\n");

    // Monta a requisição GET
    char request[256];
    snprintf(request, sizeof(request),
             "GET %s HTTP/1.1\r\n"
             "Host: %s:%d\r\n"
             "Connection: close\r\n\r\n",
             FLASK_ENDPOINT, FLASK_SERVER_IP, FLASK_SERVER_PORT);

    tcp_write(tpcb, request, strlen(request), TCP_WRITE_FLAG_COPY);
    tcp_output(tpcb);

    tcp_recv(tpcb, client_recv);
    return ERR_OK;
}

void connect_to_flask() {
    ip_addr_t server_ip;

    // Converte IP string para estrutura IP
    if (!ipaddr_aton(FLASK_SERVER_IP, &server_ip)) {
        printf("IP inválido\n");
        return;
    }

    client_pcb = tcp_new();
    tcp_connect(client_pcb, &server_ip, FLASK_SERVER_PORT, client_connected);
}

int main() {
    stdio_init_all();

    //Inicializando LEDs
    gpio_init(led_pin_red);
    gpio_set_dir(led_pin_red, GPIO_OUT);

    gpio_init(led_pin_green);
    gpio_set_dir(led_pin_green, GPIO_OUT);

    // Configuração do GPIO para o buzzer como saída
    gpio_init(BUZZER_PIN);
    gpio_set_dir(BUZZER_PIN, GPIO_OUT);

    // Inicializar o PWM no pino do buzzer
    pwm_init_buzzer(BUZZER_PIN);

     // Inicializa o I2C para o SSD1306
    i2c_init(i2c_default, SSD1306_I2C_CLK * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    SSD1306_init();

    char mensagem[50];              // Defina um tamanho adequado para armazenar a mensagem

    struct render_area area = {
        .start_col = 0,
        .end_col = SSD1306_WIDTH - 1,
        .start_page = 0,
        .end_page = SSD1306_NUM_PAGES - 1};

    calc_render_area_buflen(&area);
    uint8_t buf[area.buflen];
    memset(buf, 0, sizeof(buf));

    if (cyw43_arch_init()) {
        printf("Erro ao iniciar Wi-Fi\n");
        return -1;
    }

    cyw43_arch_enable_sta_mode();

    printf("Conectando ao Wi-Fi...\n");

    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_MIXED_PSK, 30000)) {
        printf("Falha ao conectar ao Wi-Fi\n");
        return -1;
    }

    printf("Conectado ao Wi-Fi com sucesso!\n");

    // Loop principal
    while (true) {
        // Exibe no display "Controle de Acesso"
        WriteString(buf, calcular_centro("Monitorando"), 8, "Monitorando");
        WriteString(buf, calcular_centro("BPM"), 32, "BPM");
        render(buf, &area);
        sleep_ms(3000);

         // Faz a requisição à API Flask a cada 15 segundos
        cyw43_arch_poll();
        connect_to_flask();
        sleep_ms(10000);
    }

    return 0;
}

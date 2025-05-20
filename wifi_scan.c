#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/dns.h"
#include "lwip/tcp.h"
#include <string.h>
#include <stdio.h>
#include "cJSON.h" // Biblioteca para parsear JSON (precisa adicionar ao seu projeto)


#define WIFI_SSID "Nokia G60 5G"
#define WIFI_PASSWORD "Nikolas2905"

// Altere para o IP da máquina onde roda sua API Flask
#define FLASK_SERVER_IP "192.168.68.197"
#define FLASK_SERVER_PORT 5000
#define FLASK_ENDPOINT "/"

struct tcp_pcb *client_pcb = NULL;

const uint led_pin_red = 13;
const uint led_pin_green = 11;


// Variável global opcional para o buffer da resposta
#define RESPONSE_BUFFER_SIZE 2048
static char response_buffer[RESPONSE_BUFFER_SIZE];

void simple_hex_dump(const void *data, int size) {
    const unsigned char *byte = (const unsigned char *)data;
    for (int i = 0; i < size; i++) {
        printf("%02x ", byte[i]);
        if ((i + 1) % 16 == 0) printf("\n");
    }
    printf("\n");
}

//Verifica status da conexão com a API Flask
static err_t client_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    if (p == NULL) {
        printf("Conexão encerrada pelo servidor\n");
        tcp_close(tpcb);
        return ERR_OK;
    }

    int len = p->len > RESPONSE_BUFFER_SIZE - 1 ? RESPONSE_BUFFER_SIZE - 1 : p->len;
    memcpy(response_buffer, p->payload, len);
    response_buffer[len] = '\0';

    printf("Conteúdo do JSON recebido:\n%s\n", response_buffer);

    cJSON *root = cJSON_Parse(response_buffer);
    if (root == NULL) {
        const char *error_ptr = cJSON_GetErrorPtr();
        if (error_ptr != NULL) {
            printf("Erro no JSON próximo de: %s\n", error_ptr);
        }
        pbuf_free(p);
        return ERR_OK;
    }

    cJSON *entry = NULL;
    cJSON *last_entry = NULL;

    cJSON_ArrayForEach(entry, root) {
        last_entry = entry; // sempre armazena o último objeto iterado
    }

    if (last_entry && cJSON_IsObject(last_entry)) {
        cJSON *bpm = cJSON_GetObjectItemCaseSensitive(last_entry, "bpm");
        if (cJSON_IsNumber(bpm)) {
            printf("Último BPM: %d\n", bpm->valueint);
            gpio_put(led_pin_red, bpm->valueint > 60);
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
    gpio_init(led_pin_red);
    gpio_set_dir(led_pin_red, GPIO_OUT);

    gpio_init(led_pin_green);
    gpio_set_dir(led_pin_green, GPIO_OUT);

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
        
        gpio_put(led_pin_green, true);
        sleep_ms(200);
        gpio_put(led_pin_green, false);
        sleep_ms(200);

         // Faz a requisição à API Flask a cada 15 segundos
        connect_to_flask();
        sleep_ms(15000);
        cyw43_arch_poll();

    }

    return 0;
}

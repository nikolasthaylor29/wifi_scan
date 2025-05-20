#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/dns.h"
#include "lwip/tcp.h"
#include <string.h>
#include <stdio.h>

#define WIFI_SSID "Nokia G60 5G"
#define WIFI_PASSWORD "Nikolas2905"

// Altere para o IP da máquina onde roda sua API Flask
#define FLASK_SERVER_IP "192.168.68.197"
#define FLASK_SERVER_PORT 5000
#define FLASK_ENDPOINT "/"

struct tcp_pcb *client_pcb = NULL;

const uint led_pin_red = 13;

static err_t client_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    if (p == NULL) {
        printf("Conexão encerrada pelo servidor\n");
        tcp_close(tpcb);
        return ERR_OK;
    }

    printf("Resposta da API Flask:\n%.*s\n", p->len, (char *)p->payload);
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
        
        gpio_put(led_pin_red, true);
        sleep_ms(200);
        gpio_put(led_pin_red, false);
        sleep_ms(200);

         // Faz a requisição à API Flask a cada 15 segundos
        connect_to_flask();
        sleep_ms(15000);
        cyw43_arch_poll();

    }

    return 0;
}

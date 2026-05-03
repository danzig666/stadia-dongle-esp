#include "captive_portal.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"

#include <string.h>

static TaskHandle_t s_dns_task;
static volatile bool s_running;

static void dns_task(void *arg)
{
    (void)arg;
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        s_dns_task = NULL;
        vTaskDelete(NULL);
    }

    struct timeval tv = { .tv_sec = 1, .tv_usec = 0 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port = htons(53),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    bind(sock, (struct sockaddr *)&addr, sizeof(addr));

    uint8_t req[256];
    uint8_t resp[300];
    while (s_running) {
        struct sockaddr_in from;
        socklen_t from_len = sizeof(from);
        int len = recvfrom(sock, req, sizeof(req), 0, (struct sockaddr *)&from, &from_len);
        if (len <= 12) continue;
        if (len > 240) len = 240;
        memcpy(resp, req, len);
        resp[2] = 0x81;
        resp[3] = 0x80;
        resp[6] = 0x00;
        resp[7] = 0x01;
        int p = len;
        resp[p++] = 0xc0;
        resp[p++] = 0x0c;
        resp[p++] = 0x00;
        resp[p++] = 0x01;
        resp[p++] = 0x00;
        resp[p++] = 0x01;
        resp[p++] = 0x00;
        resp[p++] = 0x00;
        resp[p++] = 0x00;
        resp[p++] = 0x3c;
        resp[p++] = 0x00;
        resp[p++] = 0x04;
        resp[p++] = 192;
        resp[p++] = 168;
        resp[p++] = 4;
        resp[p++] = 1;
        sendto(sock, resp, p, 0, (struct sockaddr *)&from, from_len);
    }
    closesocket(sock);
    s_dns_task = NULL;
    vTaskDelete(NULL);
}

void captive_portal_start(void)
{
    if (s_running) return;
    s_running = true;
    xTaskCreate(dns_task, "captive_dns", 3072, NULL, 2, &s_dns_task);
}

void captive_portal_stop(void)
{
    s_running = false;
}


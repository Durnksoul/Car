#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

typedef struct {
    uint32_t session;
    uint16_t length;
    uint8_t data[244];
} car_ble_packet_t;

esp_err_t car_ble_uart_start(void);
bool car_ble_uart_receive(car_ble_packet_t *out);
uint32_t car_ble_uart_session(void);
/* Notifications are best effort; errors never block motor control. */
void car_ble_uart_send(const char *text);

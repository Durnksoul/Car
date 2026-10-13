#include "tb6612_scope_test.h"

#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stddef.h>
#include <unistd.h>
#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "tb6612_command.h"
#include "tb6612fng.h"

static const char *TAG = "tb6612_scope";

typedef struct {
    tb6612fng_handle_t motor;
    bool enabled;
    tb6612_command_kind_t mode[2];
    int percent[2];
} control_t;

typedef struct {
    char line[TB6612_COMMAND_LINE_SIZE];
    size_t used;
    bool discard;
} input_t;

static void print_help(void)
{
    ESP_LOGI(TAG, "Commands (CR or LF ends each line; integer percentages):");
    ESP_LOGI(TAG, "help | status | enable | standby");
    ESP_LOGI(TAG, "a <percent> | b <percent> | ab <A-percent> <B-percent> (-100..100)");
    ESP_LOGI(TAG, "coast a|b|all | brake a|b|all");
    ESP_LOGI(TAG, "Example: enable, then ab 25 -50, then standby (separate lines)");
    ESP_LOGI(TAG, "PWM=20kHz, 0=coast, +/-100=HIGH; holds until next command");
}

static void print_status(const control_t *control)
{
    ESP_LOGI(TAG, "Commanded state: %s, STBY=%d (not a hardware measurement)",
             control->enabled ? "ENABLED" : "STANDBY", control->enabled);
    for (int i = 0; i < 2; ++i) {
        if (!control->enabled) {
            ESP_LOGI(TAG, "%c: IN=00, PWM=LOW", 'A' + i);
        } else if (control->mode[i] == TB_CMD_BRAKE) {
            ESP_LOGI(TAG, "%c: BRAKE, IN=11, PWM=LOW", 'A' + i);
        } else if (control->mode[i] == TB_CMD_COAST || control->percent[i] == 0) {
            ESP_LOGI(TAG, "%c: COAST, IN=00, PWM=HIGH", 'A' + i);
        } else {
            int p = control->percent[i];
            ESP_LOGI(TAG, "%c: %+d%%, IN=%s, PWM=%s", 'A' + i, p,
                     p > 0 ? "10" : "01", (p == 100 || p == -100) ? "HIGH" : "20kHz");
        }
    }
}

static void clear_commands(control_t *control)
{
    for (int i = 0; i < 2; ++i) {
        control->mode[i] = TB_CMD_COAST;
        control->percent[i] = 0;
    }
}

static esp_err_t apply_command(control_t *control, const tb6612_command_t *command)
{
    esp_err_t err = ESP_OK;
    if (command->kind == TB_CMD_HELP) { print_help(); return ESP_OK; }
    if (command->kind == TB_CMD_STATUS) { print_status(control); return ESP_OK; }
    if (command->kind == TB_CMD_ENCODER || command->kind == TB_CMD_ENCODER_ZERO)
        return ESP_ERR_NOT_SUPPORTED;
    if (command->kind == TB_CMD_ENABLE || command->kind == TB_CMD_STANDBY) {
        bool enable = command->kind == TB_CMD_ENABLE;
        err = tb6612fng_enable(control->motor, enable);
        if (err == ESP_OK) {
            if (!enable || !control->enabled) clear_commands(control);
            control->enabled = enable;
        }
    } else {
        if (!control->enabled) return ESP_ERR_INVALID_STATE;
        if (command->kind == TB_CMD_SPEED && command->motor_mask == 3) {
            err = tb6612fng_set_speeds(control->motor,
                                     command->a_percent * 10, command->b_percent * 10);
        } else {
            for (int i = 0; i < 2 && err == ESP_OK; ++i) {
                if (!(command->motor_mask & (1U << i))) continue;
                tb6612fng_motor_t motor = (tb6612fng_motor_t)i;
                if (command->kind == TB_CMD_SPEED) {
                    int p = i == 0 ? command->a_percent : command->b_percent;
                    err = tb6612fng_set_speed(control->motor, motor, p * 10);
                } else if (command->kind == TB_CMD_COAST) {
                    err = tb6612fng_coast(control->motor, motor);
                } else {
                    err = tb6612fng_brake(control->motor, motor);
                }
            }
        }
        if (err == ESP_OK) {
            for (int i = 0; i < 2; ++i) {
                if (!(command->motor_mask & (1U << i))) continue;
                control->mode[i] = command->kind;
                control->percent[i] = command->kind == TB_CMD_SPEED ?
                    (i == 0 ? command->a_percent : command->b_percent) : 0;
            }
        }
    }
    if (err != ESP_OK) {
        (void)tb6612fng_enable(control->motor, false);
        control->enabled = false;
        clear_commands(control);
    }
    return err;
}

static void receive_char(control_t *control, input_t *input, unsigned char c)
{
    if (c == '\r' || c == '\n') {
        if (input->discard) {
            ESP_LOGW(TAG, "ERR: line too long or invalid character; command discarded");
        } else if (input->used) {
            input->line[input->used] = '\0';
            tb6612_command_t command;
            esp_err_t err = tb6612_command_parse(input->line, &command);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "ERR: invalid command; send help (outputs unchanged)");
            } else {
                err = apply_command(control, &command);
                if (err == ESP_ERR_INVALID_STATE) {
                    ESP_LOGW(TAG, "ERR: in standby; send enable first");
                } else if (err == ESP_ERR_NOT_SUPPORTED) {
                    ESP_LOGW(TAG, "ERR: encoder commands unavailable in scope test mode");
                } else if (err != ESP_OK) {
                    ESP_LOGE(TAG, "ERR: %s; standby commanded", esp_err_to_name(err));
                } else {
                    ESP_LOGI(TAG, "OK: %s", input->line);
                }
            }
        }
        input->used = 0;
        input->discard = false;
    } else if (c == 8 || c == 127) {
        if (!input->discard && input->used) --input->used;
    } else if (!input->discard) {
        if ((c < 32 && c != '\t') || c > 126 || input->used >= sizeof(input->line) - 1) {
            input->discard = true;
        } else input->line[input->used++] = (char)c;
    }
}

static bool poll_input(int fd, control_t *control, input_t *input)
{
    unsigned char buffer[64];
    ssize_t count = read(fd, buffer, sizeof(buffer));
    if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) return false;
    for (ssize_t i = 0; i < count; ++i) receive_char(control, input, buffer[i]);
    return true;
}

static void scope_task(void *argument)
{
    control_t control = { .motor = argument };
    input_t primary = {0}, secondary = {0};
    clear_commands(&control);
    int usb_fd = -1;
#if CONFIG_ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG
    /* stdin forwards only to UART; open the secondary VFS for native USB RX. */
    usb_serial_jtag_vfs_set_rx_line_endings(ESP_LINE_ENDINGS_LF);
    usb_fd = open("/dev/secondary", O_RDONLY | O_NONBLOCK);
    if (usb_fd < 0) ESP_LOGW(TAG, "USB command input unavailable; use UART console");
#endif
    ESP_LOGI(TAG, "Serial scope control ready; STANDBY on boot; PWMA=GPIO6, PWMB=GPIO15");
    print_help();
    while (1) {
        if (!poll_input(STDIN_FILENO, &control, &primary) ||
            (usb_fd >= 0 && !poll_input(usb_fd, &control, &secondary))) {
            (void)tb6612fng_enable(control.motor, false);
            ESP_LOGE(TAG, "Serial read failed; standby commanded, command task stopped");
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (usb_fd >= 0) close(usb_fd);
    (void)tb6612fng_deinit(control.motor);
    vTaskDelete(NULL);
}

esp_err_t tb6612_scope_test_start(void)
{
    const tb6612fng_config_t config = TB6612FNG_CONFIG_DEFAULT();
    tb6612fng_handle_t motor = NULL;
    esp_err_t err = tb6612fng_init(&config, &motor);
    if (err != ESP_OK) return err;
#if CONFIG_ESP_CONSOLE_UART
    /* Buffered RX while the IMU prints to the same console. */
    bool installed = false;
    if (!uart_is_driver_installed(CONFIG_ESP_CONSOLE_UART_NUM)) {
        err = uart_driver_install(CONFIG_ESP_CONSOLE_UART_NUM, 1024, 0, 0, NULL, 0);
        if (err != ESP_OK) goto failed;
        installed = true;
    }
    uart_vfs_dev_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);
    uart_vfs_dev_port_set_rx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM, ESP_LINE_ENDINGS_LF);
#endif
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (flags < 0 || fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK) < 0) {
        err = ESP_FAIL;
        goto failed;
    }
    if (xTaskCreate(scope_task, "tb6612_scope", 4096, motor, 5, NULL) != pdPASS) {
        err = ESP_ERR_NO_MEM;
        goto failed;
    }
    return ESP_OK;
failed:
#if CONFIG_ESP_CONSOLE_UART
    if (installed) {
        uart_vfs_dev_use_nonblocking(CONFIG_ESP_CONSOLE_UART_NUM);
        (void)uart_driver_delete(CONFIG_ESP_CONSOLE_UART_NUM);
    }
#endif
    (void)tb6612fng_deinit(motor);
    return err;
}

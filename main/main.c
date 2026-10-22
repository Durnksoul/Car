#include <stdbool.h>
#include <stdio.h>
#include <inttypes.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "imu660rc.h"
#include "sdkconfig.h"
#include "tb6612_scope_test.h"
#include "car_motor_control.h"
#include "car_log_mode.h"

static const char *TAG = "imu_app";
static const int64_t YAW_SETTLE_US = 3000000;

void app_main(void)
{
    /* Runtime application logs only; ROM/bootloader output occurs earlier. */
    if (CAR_ENCODER_ONLY_LOG) esp_log_level_set("*", ESP_LOG_NONE);
#if CONFIG_CAR_PID_TUNER
    /* Keep ESP-IDF logs out of the tuner's serial stream. */
    esp_log_level_set("*", ESP_LOG_WARN);
#endif
#if CONFIG_CAR_MOTOR_CONTROL
    esp_err_t motor_err = car_motor_control_start();
    if (motor_err != ESP_OK) {
        ESP_LOGE(TAG, "Motor/encoder control init failed: %s", esp_err_to_name(motor_err));
    }
#elif CONFIG_CAR_TB6612_SCOPE_TEST
    /* Independent serial command task; starts in standby even without an IMU. */
    esp_err_t motor_err = tb6612_scope_test_start();
    if (motor_err != ESP_OK) {
        ESP_LOGE(TAG, "TB6612 serial control init failed: %s", esp_err_to_name(motor_err));
    }
#endif
    /* UART0 console: 115200 8N1. TX=GPIO43, RX=GPIO44 on ESP32-S3.
     * Keep the module stationary for a few seconds after power-on. */
    vTaskDelay(pdMS_TO_TICKS(300));
    const imu660rc_config_t config = IMU660RC_CONFIG_DEFAULT();
    imu660rc_handle_t imu = NULL;
    esp_err_t err;
    while ((err = imu660rc_init(&config, &imu)) != ESP_OK) {
        ESP_LOGE(TAG, "IMU init failed: %s; retry in 1 s", esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGI(TAG, "Keep IMU stationary for 3 s; recording startup yaw as zero.");
    int64_t settle_until_us = esp_timer_get_time() + YAW_SETTLE_US;
    float yaw_zero_deg = 0.0f;
    bool yaw_zeroed = false;
    #if !CONFIG_CAR_PID_TUNER
    int64_t last_print_us = 0;
#endif
    int64_t last_warning_us = 0;
#if CONFIG_CAR_MOTOR_CONTROL && !CONFIG_CAR_PID_TUNER
    int64_t last_telemetry_us = 0;
#endif
    while (1) {
        imu660rc_sample_t sample;
        /* ISR only wakes this task; all SPI reads happen in task context. */
        err = imu660rc_read(imu, &sample, 1000);
        const int64_t now_us = esp_timer_get_time();
        if (err != ESP_OK) {
#if CONFIG_CAR_MOTOR_CONTROL
            car_motor_control_imu(now_us, 0, 0, 0, false);
#endif
            /* Require a fresh settling period if startup data is interrupted. */
            if (!yaw_zeroed) {
                settle_until_us = now_us + YAW_SETTLE_US;
            }
            if (now_us - last_warning_us >= 1000000) {
                if (err == ESP_ERR_TIMEOUT) {
                    ESP_LOGW(TAG, "IMU interrupt timeout; check INT2 -> GPIO%d and sensor power",
                             (int)config.int2_gpio);
                } else if (err == ESP_ERR_INVALID_RESPONSE) {
                    ESP_LOGW(TAG, "Invalid IMU quaternion; see imu660rc FIFO packet/WHO_AM_I diagnostics; "
                             "not an INT2 timeout");
                } else if (err == ESP_ERR_INVALID_STATE) {
                    ESP_LOGW(TAG, "IMU FIFO overrun; discarded batch, waiting for fresh samples");
                } else {
                    ESP_LOGW(TAG, "IMU read failed: %s; check SPI and sensor power",
                             esp_err_to_name(err));
                }
                last_warning_us = now_us;
            }
            continue;
        }
        if (!yaw_zeroed) {
            if (now_us < settle_until_us) {
                continue;
            }
            yaw_zero_deg = sample.yaw_deg;
            yaw_zeroed = true;
            ESP_LOGI(TAG, "Yaw zero recorded: %.2f deg; output range [-180, 180).",
                     (double)yaw_zero_deg);
        }
        /* The driver retains its original 0..360 degree output. Only the
         * business-layer yaw is relative to the startup heading. */
        float yaw_relative_deg = sample.yaw_deg - yaw_zero_deg;
        if (yaw_relative_deg >= 180.0f) {
            yaw_relative_deg -= 360.0f;
        } else if (yaw_relative_deg < -180.0f) {
            yaw_relative_deg += 360.0f;
        }
        /* Read at the sensor's 120 Hz, print at most 20 Hz. */
#if CONFIG_CAR_MOTOR_CONTROL
        car_motor_control_imu(now_us, sample.tilt_deg, sample.tilt_rate_dps,
                              (float)sample.gyro_raw[2] / 14.2857f, sample.tilt_valid);
#endif
#if !CONFIG_CAR_PID_TUNER
        bool fast_telemetry = true;
#if CONFIG_CAR_MOTOR_CONTROL
        fast_telemetry = car_motor_control_telemetry_fast();
#endif
        if (fast_telemetry && !CAR_ENCODER_ONLY_LOG && now_us - last_print_us >= 50000) {
            printf("roll=%.2f,pitch=%.2f,yaw=%.2f\r\n",
                   (double)sample.roll_deg, (double)sample.pitch_deg,
                   (double)yaw_relative_deg);
            last_print_us = now_us;
        }
#endif
        /* Separate line preserves the existing viewer's strict roll/pitch/yaw
         * parser. PID uses the independent projected tilt, not these Euler angles. */
#if CONFIG_CAR_MOTOR_CONTROL && !CONFIG_CAR_PID_TUNER
        if (fast_telemetry && !CAR_ENCODER_ONLY_LOG && now_us - last_telemetry_us >= 200000) {
            car_motor_snapshot_t motor;
            const bool encoder_valid = car_motor_control_snapshot(&motor);
            printf("telemetry,t_us=%" PRId64 ",roll=%.2f,pitch=%.2f,yaw=%.2f,"
                   "gx_dps=%.3f,gy_dps=%.3f,gz_dps=%.3f,"
                   "tilt_deg=%.3f,tilt_rate_dps=%.3f,tilt_valid=%d,"
                   "enc_us=%" PRId64 ",enc_valid=%d,enc_left=%" PRId64 ",enc_right=%" PRId64 ","
                   "cps_left=%.2f,cps_right=%.2f,enabled=%d,"
                   "cmd_left=%d,cmd_right=%d,brake_left=%d,brake_right=%d,"
                   "balance_active=%d,balance_state=%d,trim_deg=%.3f,target_tilt_deg=%.3f,"
                   "pid_left=%.3f,pid_right=%.3f\r\n",
                   now_us, (double)sample.roll_deg, (double)sample.pitch_deg,
                   (double)yaw_relative_deg,
                   (double)sample.gyro_raw[0] / 14.2857,
                   (double)sample.gyro_raw[1] / 14.2857,
                   (double)sample.gyro_raw[2] / 14.2857,
                   (double)sample.tilt_deg, (double)sample.tilt_rate_dps, sample.tilt_valid,
                   motor.encoder_us, encoder_valid,
                   motor.encoder.count[0], motor.encoder.count[1],
                   (double)motor.encoder.cps[0], (double)motor.encoder.cps[1],
                   motor.enabled, motor.command_percent[0], motor.command_percent[1],
                   motor.braking[0], motor.braking[1],
                   motor.balance.state == BALANCE_ARMED, motor.balance.state,
                   (double)motor.balance_trim_deg, (double)motor.balance.target_tilt_deg,
                   (double)motor.balance.left_percent, (double)motor.balance.right_percent);
            last_telemetry_us = now_us;
        }
#endif
    }
}

set(car_sources "tb6612_scope_test.c" "tb6612_command.c" "main.c"
                "pid_controller.c" "balance_controller.c")
set(car_dependencies imu660rc esp_timer tb6612fng esp_driver_uart esp_driver_usb_serial_jtag vfs
                     esp_driver_pcnt bt nvs_flash)
if(CONFIG_CAR_MOTOR_CONTROL)
    list(APPEND car_sources "car_motor_control.c" "car_encoder.c" "car_command_stream.c")
endif()
if(CONFIG_CAR_BLE_UART)
    list(APPEND car_sources "car_ble_uart.c")
endif()
idf_component_register(SRCS ${car_sources}
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES ${car_dependencies})

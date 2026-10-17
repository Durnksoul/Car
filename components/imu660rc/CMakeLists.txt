idf_component_register(SRCS "imu660rc.c" "imu660rc_math.c" "imu660rc_fifo.c"
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS "."
                       REQUIRES esp_driver_spi esp_driver_gpio
                       PRIV_REQUIRES freertos esp_timer)

#pragma once

#include "esp_err.h"

/* Start serial command control, initially in standby. For tests with NO motors. */
esp_err_t tb6612_scope_test_start(void);

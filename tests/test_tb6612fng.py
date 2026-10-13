"""Run the actual driver C on the host, replacing only ESP-IDF GPIO/LEDC APIs.

Requires Clang and llvmlite (also used by test_angles.py). Run from any directory:
    python tests/test_tb6612fng.py [path/to/clang.exe]
Generated headers and LLVM IR stay under build/tb6612_host. No board or flash.
"""
import ctypes
from pathlib import Path
import shutil
import subprocess
import sys

from llvmlite import binding as llvm

root = Path(__file__).resolve().parents[1]
work = root / "build" / "tb6612_host"
work.mkdir(parents=True, exist_ok=True)
headers = {
    "esp_err.h": """
#pragma once
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_NO_MEM 0x101
#define ESP_ERR_INVALID_ARG 0x102
#define ESP_ERR_INVALID_STATE 0x103
""",
    "stdlib.h": """
#pragma once
#include <stddef.h>
void *calloc(size_t count, size_t size);
void free(void *pointer);
""",
    "driver/gpio.h": """
#pragma once
#include <stdint.h>
#include "esp_err.h"
typedef int gpio_num_t;
enum { GPIO_NUM_4=4, GPIO_NUM_5=5, GPIO_NUM_6=6, GPIO_NUM_7=7,
       GPIO_NUM_8=8, GPIO_NUM_15=15, GPIO_NUM_16=16 };
enum { GPIO_MODE_OUTPUT=1, GPIO_PULLDOWN_ENABLE=1, GPIO_INTR_DISABLE=0 };
#define GPIO_IS_VALID_OUTPUT_GPIO(p) ((p)>=0 && (p)<=48 && !((p)>=22 && (p)<=25))
typedef struct { uint64_t pin_bit_mask; int mode, pull_up_en, pull_down_en, intr_type; } gpio_config_t;
esp_err_t gpio_config(const gpio_config_t *config);
esp_err_t gpio_set_level(gpio_num_t pin, uint32_t level);
esp_err_t gpio_reset_pin(gpio_num_t pin);
""",
    "driver/ledc.h": """
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
typedef int ledc_channel_t;
typedef int ledc_timer_t;
enum { LEDC_CHANNEL_0=0, LEDC_CHANNEL_1=1, LEDC_CHANNEL_MAX=8,
       LEDC_TIMER_0=0, LEDC_TIMER_MAX=4, LEDC_TIMER_10_BIT=10,
       LEDC_LOW_SPEED_MODE=0, LEDC_AUTO_CLK=0, LEDC_INTR_DISABLE=0 };
typedef struct { int speed_mode, duty_resolution, timer_num; uint32_t freq_hz;
                 int clk_cfg; bool deconfigure; } ledc_timer_config_t;
typedef struct { int gpio_num, speed_mode, channel, intr_type, timer_sel;
                 uint32_t duty; int hpoint; } ledc_channel_config_t;
esp_err_t ledc_timer_config(const ledc_timer_config_t *config);
esp_err_t ledc_channel_config(const ledc_channel_config_t *config);
esp_err_t ledc_stop(int mode, ledc_channel_t channel, uint32_t idle);
esp_err_t ledc_set_duty(int mode, ledc_channel_t channel, uint32_t duty);
esp_err_t ledc_update_duty(int mode, ledc_channel_t channel);
esp_err_t ledc_timer_pause(int mode, ledc_timer_t timer);
""",
}
for name, data in headers.items():
    path = work / name
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(data, encoding="utf-8")

harness = r"""
#include "tb6612fng.h"
static int pin[49], duty[8], running[8], idle[8];
static int calls, fail_at, stops, paused, deconfigured;
static int api(void) { ++calls; return calls == fail_at ? ESP_FAIL : ESP_OK; }
esp_err_t gpio_config(const gpio_config_t *c) { (void)c; return api(); }
esp_err_t gpio_set_level(int p, uint32_t v) {
    int e=api(); if(!e) pin[p]=v; return e;
}
esp_err_t gpio_reset_pin(int p) { int e=api(); if(!e) pin[p]=0; return e; }
esp_err_t ledc_timer_config(const ledc_timer_config_t *c) {
    int e=api(); if(!e && c->deconfigure) ++deconfigured; return e;
}
esp_err_t ledc_channel_config(const ledc_channel_config_t *c) {
    int e=api(); if(!e) { duty[c->channel]=c->duty; running[c->channel]=1; } return e;
}
esp_err_t ledc_stop(int m, int c, uint32_t v) {
    (void)m; int e=api(); if(!e) { running[c]=0; idle[c]=v; ++stops; } return e;
}
esp_err_t ledc_set_duty(int m, int c, uint32_t v) {
    (void)m; int e=api(); if(!e) duty[c]=v; return e;
}
esp_err_t ledc_update_duty(int m, int c) {
    (void)m; int e=api(); if(!e) running[c]=1; return e;
}
esp_err_t ledc_timer_pause(int m, int t) {
    (void)m; (void)t; int e=api(); if(!e) ++paused; return e;
}
static int pwm(int c) { return running[c] ? duty[c] : idle[c]*1024; }
#define CHECK(condition) do { if(!(condition)) return __LINE__; } while(0)
#define OK(call) CHECK((call)==ESP_OK)

int test_driver(void) {
    tb6612fng_config_t config=TB6612FNG_CONFIG_DEFAULT();
    tb6612fng_handle_t dev=0;
    OK(tb6612fng_init(&config, &dev));
    int init_calls=calls;
    CHECK(dev && pin[16]==0 && pwm(0)==0 && pwm(1)==0);
    CHECK(tb6612fng_set_speeds(dev,250,-500)==ESP_ERR_INVALID_STATE);
    OK(tb6612fng_enable(dev,true));
    CHECK(pin[16]==1 && pin[4]==0 && pin[5]==0 && pwm(0)==1024 && pwm(1)==1024);
    OK(tb6612fng_set_speeds(dev,250,-500));
    CHECK(pin[4]==1 && pin[5]==0 && pwm(0)==256);
    CHECK(pin[7]==0 && pin[8]==1 && pwm(1)==512);
    int old_stops=stops;
    OK(tb6612fng_set_speed(dev,TB6612FNG_MOTOR_A,750));
    CHECK(pwm(0)==768 && pwm(1)==512 && stops==old_stops);
    OK(tb6612fng_enable(dev,true));
    CHECK(pwm(0)==768 && pwm(1)==512);
    int old_calls=calls;
    CHECK(tb6612fng_set_speeds(dev,100,-1001)==ESP_ERR_INVALID_ARG);
    CHECK(tb6612fng_set_speed(dev,(tb6612fng_motor_t)-1,500)==ESP_ERR_INVALID_ARG);
    CHECK(calls==old_calls && pwm(0)==768 && pwm(1)==512);
    OK(tb6612fng_set_speeds(dev,-500,750));
    CHECK(pin[4]==0 && pin[5]==1 && pwm(0)==512);
    CHECK(pin[7]==1 && pin[8]==0 && pwm(1)==768);
    OK(tb6612fng_set_speeds(dev,1000,-1000));
    CHECK(pin[4]==1 && pin[5]==0 && pwm(0)==1024 && !running[0]);
    CHECK(pin[7]==0 && pin[8]==1 && pwm(1)==1024 && !running[1]);
    OK(tb6612fng_set_speeds(dev,250,-500));
    CHECK(running[0] && running[1] && pwm(0)==256 && pwm(1)==512);
    OK(tb6612fng_set_speed(dev,TB6612FNG_MOTOR_A,0));
    CHECK(pin[4]==0 && pin[5]==0 && pwm(0)==1024 && pwm(1)==512);
    OK(tb6612fng_brake(dev,TB6612FNG_MOTOR_B));
    CHECK(pin[7]==1 && pin[8]==1 && pwm(1)==0);
    OK(tb6612fng_coast(dev,TB6612FNG_MOTOR_B));
    CHECK(pin[7]==0 && pin[8]==0 && pwm(1)==1024);
    OK(tb6612fng_enable(dev,false));
    CHECK(pin[16]==0 && pwm(0)==0 && pwm(1)==0);
    CHECK(tb6612fng_brake(dev,TB6612FNG_MOTOR_A)==ESP_ERR_INVALID_STATE);
    OK(tb6612fng_enable(dev,true));
    CHECK(pin[4]==0 && pin[5]==0 && pin[7]==0 && pin[8]==0);
    CHECK(pwm(0)==1024 && pwm(1)==1024);
    OK(tb6612fng_deinit(dev));
    CHECK(pin[16]==0 && paused && deconfigured);

    /* Every initialization API failure must return no handle, without enabling. */
    for(int fault=1;fault<=init_calls;++fault) {
        calls=0; fail_at=fault; dev=0;
        CHECK(tb6612fng_init(&config,&dev)==ESP_FAIL);
        CHECK(dev==0 && pin[16]==0);
    }
    fail_at=0; calls=0;
    tb6612fng_config_t bad=config;
    bad.motor[1].pwm_gpio=bad.motor[0].pwm_gpio;
    CHECK(tb6612fng_init(&bad,&dev)==ESP_ERR_INVALID_ARG && !dev && calls==0);
    bad=config; bad.motor[0].in1_gpio=22;
    CHECK(tb6612fng_init(&bad,&dev)==ESP_ERR_INVALID_ARG && calls==0);
    bad=config; bad.pwm_freq_hz=100001;
    CHECK(tb6612fng_init(&bad,&dev)==ESP_ERR_INVALID_ARG && calls==0);
    bad=config; bad.motor[1].ledc_channel=bad.motor[0].ledc_channel;
    CHECK(tb6612fng_init(&bad,&dev)==ESP_ERR_INVALID_ARG && calls==0);

    /* Find number of HAL operations during dual reversal, then fault each one. */
    OK(tb6612fng_init(&config,&dev)); OK(tb6612fng_enable(dev,true));
    OK(tb6612fng_set_speeds(dev,250,-500)); calls=0;
    OK(tb6612fng_set_speeds(dev,-750,250)); int reversal_calls=calls;
    OK(tb6612fng_deinit(dev));
    for(int fault=1;fault<=reversal_calls;++fault) {
        calls=0; fail_at=0;
        OK(tb6612fng_init(&config,&dev)); OK(tb6612fng_enable(dev,true));
        OK(tb6612fng_set_speeds(dev,250,-500)); calls=0; fail_at=fault;
        CHECK(tb6612fng_set_speeds(dev,-750,250)==ESP_FAIL);
        CHECK(pin[16]==0 && pwm(0)==0 && pwm(1)==0);
        CHECK(tb6612fng_set_speeds(dev,250,500)==ESP_ERR_INVALID_STATE);
        fail_at=0; OK(tb6612fng_deinit(dev));
    }
    return 0;
}
"""
source = work / "harness.c"
source.write_text(harness, encoding="utf-8")
compiler = sys.argv[1] if len(sys.argv) > 1 else (
    shutil.which("clang") or
    r"C:\Espressif\tools\esp-clang\esp-19.1.2_20250312\esp-clang\bin\clang.exe"
)
modules = []
for path in (root / "components/tb6612fng/tb6612fng.c", source):
    ir = work / (path.stem + ".ll")
    subprocess.run([compiler, "-target", llvm.get_default_triple(),
                    "-S", "-emit-llvm", "-O0", "-ffreestanding",
                    "-Wall", "-Wextra", "-Werror", "-I", str(work), "-I",
                    str(root / "components/tb6612fng/include"), str(path),
                    "-o", str(ir)], check=True)
    modules.append(llvm.parse_assembly(ir.read_text()))
llvm.initialize_native_target()
llvm.initialize_native_asmprinter()
runtime = ctypes.CDLL("ucrtbase.dll" if sys.platform == "win32" else None)
for name in ("calloc", "free", "memcpy", "memset"):
    llvm.add_symbol(name, ctypes.cast(getattr(runtime, name), ctypes.c_void_p).value)
modules[0].link_in(modules[1])
modules[0].verify()
target = llvm.Target.from_default_triple().create_target_machine()
modules[0].triple = llvm.get_default_triple()
modules[0].data_layout = str(target.target_data)
engine = llvm.create_mcjit_compiler(modules[0], target)
engine.finalize_object()
test = ctypes.CFUNCTYPE(ctypes.c_int)(engine.get_function_address("test_driver"))
result = test()
if result:
    raise AssertionError(f"Driver check failed at generated harness.c:{result}")
print("PASS: standby, direction/duty, channel independence, coast/brake, 100%, "
      "restart, invalid inputs, all init/reversal HAL failure points")

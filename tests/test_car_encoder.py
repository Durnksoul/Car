"""Execute the real encoder C with mocked PCNT and clock; no board or flashing."""
import ctypes
from pathlib import Path
import subprocess
import sys
from llvmlite import binding as llvm

root = Path(__file__).resolve().parents[1]
work = root / "build/encoder_host"
work.mkdir(parents=True, exist_ok=True)
headers = {
    "esp_err.h": "#pragma once\ntypedef int esp_err_t;\n#define ESP_OK 0\n#define ESP_ERR_INVALID_STATE 0x103\n",
    "esp_timer.h": "#include <stdint.h>\nint64_t esp_timer_get_time(void);\n",
    "driver/gpio.h": "#include \"esp_err.h\"\n#define GPIO_PULLUP_ONLY 1\nesp_err_t gpio_set_pull_mode(int pin, int mode);\n",
    "driver/pulse_cnt.h": """
#pragma once
#include <stdbool.h>
#include "esp_err.h"
typedef struct unit *pcnt_unit_handle_t;
typedef struct channel *pcnt_channel_handle_t;
typedef struct { int low_limit, high_limit; struct { bool accum_count; } flags; } pcnt_unit_config_t;
typedef struct { int max_glitch_ns; } pcnt_glitch_filter_config_t;
typedef struct { int edge_gpio_num, level_gpio_num; } pcnt_chan_config_t;
enum { PCNT_CHANNEL_EDGE_ACTION_DECREASE=-1, PCNT_CHANNEL_EDGE_ACTION_INCREASE=1,
       PCNT_CHANNEL_LEVEL_ACTION_KEEP=0, PCNT_CHANNEL_LEVEL_ACTION_INVERSE=1 };
esp_err_t pcnt_new_unit(const pcnt_unit_config_t *, pcnt_unit_handle_t *);
esp_err_t pcnt_del_unit(pcnt_unit_handle_t);
esp_err_t pcnt_unit_set_glitch_filter(pcnt_unit_handle_t, const pcnt_glitch_filter_config_t *);
esp_err_t pcnt_new_channel(pcnt_unit_handle_t, const pcnt_chan_config_t *, pcnt_channel_handle_t *);
esp_err_t pcnt_del_channel(pcnt_channel_handle_t);
esp_err_t pcnt_channel_set_edge_action(pcnt_channel_handle_t, int, int);
esp_err_t pcnt_channel_set_level_action(pcnt_channel_handle_t, int, int);
esp_err_t pcnt_unit_add_watch_point(pcnt_unit_handle_t, int);
esp_err_t pcnt_unit_enable(pcnt_unit_handle_t);
esp_err_t pcnt_unit_disable(pcnt_unit_handle_t);
esp_err_t pcnt_unit_start(pcnt_unit_handle_t);
esp_err_t pcnt_unit_stop(pcnt_unit_handle_t);
esp_err_t pcnt_unit_clear_count(pcnt_unit_handle_t);
esp_err_t pcnt_unit_get_count(pcnt_unit_handle_t, int *);
""",
}
for name, text in headers.items():
    path = work / name
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")

harness = r'''
#include "driver/pulse_cnt.h"
#include <stdint.h>
#include <stddef.h>
struct unit { int raw, index, watches, active; } hardware[2];
struct channel { struct unit *unit; int pin, level, rising, falling, high, low; } hwchan[4];
static int allocated, nchannels, calls, fail_at, configuration_errors;
static int64_t clock_us;
static int api(void) { return ++calls == fail_at ? -1 : 0; }
int64_t esp_timer_get_time(void) { return clock_us; }
int gpio_set_pull_mode(int pin, int mode) { (void)pin; (void)mode; return api(); }
int pcnt_new_unit(const pcnt_unit_config_t *c, pcnt_unit_handle_t *out) {
    int e=api(); if(e) return e;
    if(c->low_limit!=-30000 || c->high_limit!=30000 || !c->flags.accum_count) ++configuration_errors;
    hardware[allocated]=(struct unit){.index=allocated}; *out=&hardware[allocated++]; return 0;
}
int pcnt_del_unit(pcnt_unit_handle_t u) { (void)u; --allocated; return 0; }
int pcnt_unit_set_glitch_filter(pcnt_unit_handle_t u, const pcnt_glitch_filter_config_t *c) {
    (void)u; if(c->max_glitch_ns!=1000) ++configuration_errors; return api();
}
int pcnt_new_channel(pcnt_unit_handle_t u, const pcnt_chan_config_t *c, pcnt_channel_handle_t *out) {
    int e=api(); if(e) return e;
    hwchan[nchannels]=(struct channel){.unit=u,.pin=c->edge_gpio_num,.level=c->level_gpio_num};
    *out=&hwchan[nchannels++]; return 0;
}
int pcnt_del_channel(pcnt_channel_handle_t c) { (void)c; --nchannels; return 0; }
int pcnt_channel_set_edge_action(pcnt_channel_handle_t c, int r, int f) { c->rising=r; c->falling=f; return api(); }
int pcnt_channel_set_level_action(pcnt_channel_handle_t c, int h, int l) { c->high=h; c->low=l; return api(); }
int pcnt_unit_add_watch_point(pcnt_unit_handle_t u, int w) { if(w!=-30000 && w!=30000) ++configuration_errors; ++u->watches; return api(); }
int pcnt_unit_enable(pcnt_unit_handle_t u) { (void)u; return api(); }
int pcnt_unit_disable(pcnt_unit_handle_t u) { (void)u; return 0; }
int pcnt_unit_start(pcnt_unit_handle_t u) { int e=api(); if(!e) u->active=1; return e; }
int pcnt_unit_stop(pcnt_unit_handle_t u) { u->active=0; return 0; }
int pcnt_unit_clear_count(pcnt_unit_handle_t u) { u->raw=0; return api(); }
int pcnt_unit_get_count(pcnt_unit_handle_t u, int *out) { int e=api(); if(!e) *out=u->raw; return e; }
void inject(int a, int b, int64_t t) { hardware[0].raw=a; hardware[1].raw=b; clock_us=t; }
void tick(int64_t t) { clock_us=t; }
int resources(void) { return allocated+nchannels; }
int errors(void) { return configuration_errors; }
int total_calls(void) { return calls; }
void reset_mock(int failure) { allocated=nchannels=calls=configuration_errors=0; fail_at=failure; clock_us=0; }
int get_pin(int channel) { return hwchan[channel].pin; }
int watch_count(int i) { return hardware[i].watches; }
void cycle(int wheel, int reverse) {
    /* Quadrature phase states: 00,10,11,01,00 (A leads B). */
    const int sequence[5]={0,2,3,1,0};
    for(int step=1; step<5; ++step) {
        int before=sequence[reverse ? 5-step : step-1];
        int after=sequence[reverse ? 4-step : step];
        for(int j=0; j<2; ++j) {
            struct channel *c=&hwchan[wheel*2+j]; int mask=j==0 ? 2:1;
            if((before & mask)==(after & mask)) continue;
            int action=(after & mask) ? c->rising:c->falling;
            int invert=(after & (j==0 ? 1:2)) ? c->high:c->low;
            c->unit->raw += invert ? -action:action;
        }
    }
}
#include "car_encoder.c"
'''
(work / "harness.c").write_text(harness, encoding="utf-8")
compiler = sys.argv[1] if len(sys.argv) > 1 else r"C:\Espressif\tools\esp-clang\esp-19.1.2_20250312\esp-clang\bin\clang.exe"
ir = work / "encoder.ll"
subprocess.run([compiler, "-target", llvm.get_default_triple(), "-S", "-emit-llvm", "-O0",
                "-ffreestanding", "-Wall", "-Wextra", "-Werror", "-I", str(work),
                "-I", str(root / "main"), str(work / "harness.c"), "-o", str(ir)], check=True)
llvm.initialize_native_target(); llvm.initialize_native_asmprinter()
runtime = ctypes.CDLL("ucrtbase.dll" if sys.platform == "win32" else None)
for name in ("memcpy", "memset"):
    llvm.add_symbol(name, ctypes.cast(getattr(runtime, name), ctypes.c_void_p).value)
module = llvm.parse_assembly(ir.read_text()); module.verify()
target = llvm.Target.from_default_triple().create_target_machine()
module.triple = llvm.get_default_triple(); module.data_layout = str(target.target_data)
engine = llvm.create_mcjit_compiler(module, target); engine.finalize_object()
def fn(name, result, *args):
    return ctypes.CFUNCTYPE(result, *args)(engine.get_function_address(name))
class Sample(ctypes.Structure):
    _fields_ = [("count", ctypes.c_int64 * 2), ("cps", ctypes.c_float * 2)]
init=fn("car_encoder_init", ctypes.c_int)
deinit=fn("car_encoder_deinit", None)
sample=fn("car_encoder_sample", ctypes.c_int, ctypes.POINTER(Sample))
zero=fn("car_encoder_zero", ctypes.c_int)
inject=fn("inject", None, ctypes.c_int, ctypes.c_int, ctypes.c_int64)
reset=fn("reset_mock", None, ctypes.c_int)
resources=fn("resources", ctypes.c_int)
calls=fn("total_calls", ctypes.c_int)
cycle=fn("cycle", None, ctypes.c_int, ctypes.c_int)
reset(0); assert init()==0
init_calls=calls()
assert fn("errors",ctypes.c_int)()==0
assert [fn("get_pin",ctypes.c_int,ctypes.c_int)(i) for i in range(4)]==[17,18,1,2]
assert [fn("watch_count",ctypes.c_int,ctypes.c_int)(i) for i in range(2)]==[2,2]
cycle(0,0); cycle(1,1); fn("tick",None,ctypes.c_int64)(200000)
out=Sample(); assert sample(ctypes.byref(out))==0
assert list(out.count)==[4,-4] and list(out.cps)==[20,-20]
inject(60004,-60004,400000); assert sample(ctypes.byref(out))==0
assert list(out.count)==[60004,-60004] and list(out.cps)==[300000,-300000]
assert zero()==0
inject(60014,-60024,600000); assert sample(ctypes.byref(out))==0
assert list(out.count)==[10,-20] and list(out.cps)==[50,-100]
deinit(); assert resources()==0 and sample(ctypes.byref(out))==0x103
# Driver count wrapping is interpreted as a small signed increment/decrement.
reset(0); assert init()==0
inject(2147483645,-2147483646,200000); assert sample(ctypes.byref(out))==0
inject(-2147483645,2147483646,400000); assert sample(ctypes.byref(out))==0
assert list(out.count)==[2147483651,-2147483650] and list(out.cps)==[30,-20]
deinit()
for failure in range(1,init_calls+1):
    reset(failure); assert init()!=0, failure
    assert resources()==0, failure
print("PASS: GPIO mapping, quadrature configuration, signed x4 counts, CPS, zero, wrap and init failure cleanup")

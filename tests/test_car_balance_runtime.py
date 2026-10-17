"""Run actual motor-command/PID runtime functions with a fake clock and bridge."""
import ctypes
from pathlib import Path
import re
import subprocess
from llvmlite import binding as llvm

root = Path(__file__).resolve().parents[1]
work = root / 'build/control_host'
work.mkdir(parents=True, exist_ok=True)
headers = {
'esp_err.h': '#pragma once\ntypedef int esp_err_t;\n#define ESP_OK 0\n#define ESP_ERR_INVALID_STATE 0x103\n#define ESP_ERR_INVALID_ARG 0x102\nconst char *esp_err_to_name(int);\n',
'driver/gpio.h': '#pragma once\ntypedef int gpio_num_t;\n',
'driver/ledc.h': '#pragma once\ntypedef int ledc_channel_t; typedef int ledc_timer_t;\n',
'math.h': '#pragma once\n#define isfinite(x) __builtin_isfinite(x)\n#define fabsf(x) __builtin_fabsf(x)\nfloat fminf(float,float); float fmaxf(float,float); long lroundf(float);\n',
}
for name, value in headers.items():
    path = work / name
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(value, encoding='utf-8')
text = (root / 'main/car_motor_control.c').read_text(encoding='utf-8')
# Exclude device startup and I/O polling; retain actual mailbox, command and tick code.
body = text[:text.index('static bool poll_input')]
body = re.sub(r'^#include .*$', '', body, flags=re.M)
start, end = body.index('static void reply('), body.index('static void print_encoder(')
body = body[:start] + 'static void reply(const char *fmt, ...) { (void)fmt; }\n' + body[end:]
startup = text[text.index('    control_t control = {.motor = argument};'):text.index('    car_command_stream_t primary')]
preamble = r'''
#include <stdbool.h>
#include <stdint.h>
#define PRId64 "lld"
#include <stddef.h>
#include <math.h>
#include "car_motor_control.h"
#include "tb6612_command.h"
#include "tb6612fng.h"
#include "pid_controller.c"
#include "balance_controller.c"
#include "tb6612_command.c"
#define CONFIG_CAR_BLE_UART 0
#define portMUX_TYPE int
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(x) ((void)(x))
#define portEXIT_CRITICAL(x) ((void)(x))
int sscanf(const char *,const char *,...);
int strcmp(const char *,const char *);
static int64_t clock_us=1000000;
static bool bridge;
static int pwm[2];
static int bridge_error;
int64_t esp_timer_get_time(void) { return clock_us; }
const char *esp_err_to_name(int err) { (void)err; return "mock"; }
int tb6612fng_enable(tb6612fng_handle_t h,bool on) {
    (void)h; if(!on) { bridge=false; pwm[0]=pwm[1]=0; }
    if(bridge_error) return bridge_error;
    bridge=on; return 0;
}
int tb6612fng_set_speeds(tb6612fng_handle_t h,int a,int b) {
    (void)h; pwm[0]=a; pwm[1]=b; return bridge_error;
}
int tb6612fng_set_speed(tb6612fng_handle_t h,tb6612fng_motor_t m,int p) {
    (void)h; pwm[m]=p; return 0;
}
int tb6612fng_coast(tb6612fng_handle_t h,tb6612fng_motor_t m) { return tb6612fng_set_speed(h,m,0); }
int tb6612fng_brake(tb6612fng_handle_t h,tb6612fng_motor_t m) { return tb6612fng_set_speed(h,m,0); }
int car_encoder_zero(void) { return 0; }
'''
tests = r'''
#define CHECK(x) do { if(!(x)) return __LINE__; } while(0)
static void feed(float tilt, float rate, bool valid) {
    clock_us+=10000;
    encoder_us=clock_us;
    car_motor_control_imu(clock_us,tilt,rate,0,valid);
}
int test_runtime(void) {
    control_t c=new_control();
    CHECK(!c.enabled && c.balance.state==BALANCE_DISARMED);
    receive_line("balance arm",&c); CHECK(!bridge);
    feed(-6.1f,0,true);
    receive_line("balance zero",&c); CHECK(c.trim_confirmed);
    CHECK(fabsf(c.balance.config.trim_deg+6.1f)<.001f);
    receive_line("balance arm",&c); CHECK(!bridge); /* zero Kp */
    receive_line("pid attitude 1 0 0.03",&c);
    CHECK(c.balance.config.attitude.kp==1);
    receive_line("pid attitude nan 0 0",&c);
    receive_line("pid attitude 2 0 0 trailing",&c);
    receive_line("pid attitude -2 0 0",&c);
    CHECK(c.balance.config.attitude.kp==1);
    receive_line("balance arm",&c); CHECK(bridge && c.enabled);
    CHECK(balance_tick(&c)==0 && pwm[0]==0 && pwm[1]==0); /* same sample */
    feed(-4.1f,1,true); CHECK(balance_tick(&c)==0);
    CHECK(pwm[0]>0 && pwm[1]<0 && c.balance_output.left_percent>0);
    int before=pwm[0];
    receive_line("ab -100 -100",&c); CHECK(pwm[0]==before);
    receive_line("pid attitude 50 0 0",&c); CHECK(c.balance.config.attitude.kp==1);
    clock_us+=60000; CHECK(balance_tick(&c)==0);
    CHECK(!bridge && !c.enabled && c.balance.state==BALANCE_FAULT_STALE);
    feed(-6.1f,0,true); CHECK(balance_tick(&c)==0 && !bridge); /* latched */
    receive_line("balance arm",&c); CHECK(bridge);
    feed(20,0,true); CHECK(balance_tick(&c)==0);
    CHECK(!bridge && c.balance.state==BALANCE_FAULT_FALL);
    feed(-6.1f,0,true); receive_line("balance arm",&c); CHECK(bridge);
    feed(0,0,false); CHECK(balance_tick(&c)==0);
    CHECK(!bridge && c.balance.state==BALANCE_FAULT_INPUT);
    feed(-6.1f,0,true); receive_line("balance arm",&c); CHECK(bridge);
    receive_line("standby",&c); CHECK(!bridge && c.balance.state==BALANCE_DISARMED);
    receive_line("balance motor 1",&c);
    receive_line("balance arm",&c); feed(-4.1f,0,true);
    CHECK(balance_tick(&c)==0 && pwm[0]<0 && pwm[1]>0);
    receive_line("balance off",&c); CHECK(!bridge);
    feed(-6.1f,0,true); bridge_error=-1;
    receive_line("balance arm",&c); CHECK(!c.enabled && !bridge);
    bridge_error=0;
    receive_line("balance motor -1",&c);
    receive_line("pid attitude 10 0 0",&c);
    receive_line("balance arm",&c); CHECK(bridge);
    feed(10,0,true); CHECK(balance_tick(&c)==0);
    CHECK(pwm[0]==200 && pwm[1]==-200); /* runtime output cap */
    receive_line("balance off",&c);
    feed(-6.1f,0,true);
    receive_line("pid speed 0.01 0 0",&c);
    receive_line("balance arm",&c); CHECK(bridge);
    encoder.cps[0]=encoder.cps[1]=100;
    feed(-6.1f,0,true); CHECK(balance_tick(&c)==0);
    feed(-6.1f,0,true); CHECK(balance_tick(&c)==0);
    feed(-6.1f,0,true); CHECK(balance_tick(&c)==0);
    /* Forward motion requests backward lean; the inner loop first moves the
     * wheels forward to establish that lean, rather than directly braking. */
    CHECK(c.balance_output.target_tilt_deg<c.balance.config.trim_deg && pwm[0]>0);
    encoder_us=clock_us-110000;
    CHECK(balance_tick(&c)==0 && !bridge && c.balance.state==BALANCE_FAULT_STALE);
    return 0;
}
'''
source = preamble + body + '\nstatic control_t new_control(void) { void *argument=NULL;\n' + startup + '\nreturn control; }\n' + tests
(work/'harness.c').write_text(source, encoding='utf-8')
compiler = r'C:\Espressif\tools\esp-clang\esp-19.1.2_20250312\esp-clang\bin\clang.exe'
ir=work/'control.ll'
subprocess.run([compiler,'-target',llvm.get_default_triple(),'-S','-emit-llvm','-O0','-ffreestanding',
                '-Wall','-Wextra','-Werror','-Wno-unused-function','-Wno-unused-variable',
                '-I',str(work),'-I',str(root/'main'),'-I',str(root/'components/tb6612fng/include'),
                str(work/'harness.c'),'-o',str(ir)],check=True)
llvm.initialize_native_target(); llvm.initialize_native_asmprinter()
runtime=ctypes.CDLL('ucrtbase.dll')
legacy=ctypes.CDLL('msvcrt.dll')
for name in ('memcpy','memset','sscanf','strcmp','fminf','fmaxf','lroundf'):
    library=legacy if name=='sscanf' else runtime
    llvm.add_symbol(name,ctypes.cast(getattr(library,name),ctypes.c_void_p).value)
module=llvm.parse_assembly(ir.read_text()); module.verify()
target=llvm.Target.from_default_triple().create_target_machine()
module.triple=llvm.get_default_triple(); module.data_layout=str(target.target_data)
engine=llvm.create_mcjit_compiler(module,target); engine.finalize_object()
result=ctypes.CFUNCTYPE(ctypes.c_int)(engine.get_function_address('test_runtime'))()
assert result==0, f'runtime check failed at generated harness line {result}'
print('PASS: zero/arm gates, PID commands, direction, mode exclusion, stale/fall/invalid stop, explicit rearm, bridge error')

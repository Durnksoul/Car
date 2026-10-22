"""Run actual motor-command/PID runtime functions with a fake clock and bridge."""
import ctypes
from pathlib import Path
import re
import subprocess
import sys
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
body = body[:start] + 'static int reply_count;\nstatic void reply(const char *fmt, ...) { (void)fmt; ++reply_count; }\n' + body[end:]
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
#define CONFIG_CAR_PID_TUNER 1
#define CONFIG_CAR_PID_TUNER_AUTO_ARM 1
#define CONFIG_CAR_PID_TUNER_PERIOD_MS 50
#define CAR_HOST_TEST 1
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
#if CONFIG_CAR_PID_TUNER
int test_tuner(void) {
    bridge=false; pwm[0]=pwm[1]=0; bridge_error=0;
    encoder.cps[0]=encoder.cps[1]=0;
    control_t c=new_control();
    feed(-6.0f,0,true);
    receive_line("SET P:3.5 I:0.2 D:0.4",&c);
    CHECK(bridge && c.enabled && c.balance.state==BALANCE_ARMED);
    CHECK(fabsf(c.balance.config.attitude.kp-3.5f)<1e-6f);
    CHECK(fabsf(c.trim_base_deg+6.0f)<.001f);
    receive_line("SET P:1000 I:100 D:100",&c);
    CHECK(c.balance.config.attitude.kp==50 && c.balance.config.attitude.ki==20 &&
          c.balance.config.attitude.kd==5);
    receive_line("SET P:0 I:0 D:0",&c);
    CHECK(c.balance.config.attitude.kp==50);
    receive_line("SET P:2 I:0 D:0 trailing",&c);
    CHECK(c.balance.config.attitude.kp==50);
    receive_line("SET P:nan I:0 D:0",&c);
    CHECK(c.balance.config.attitude.kp==50);
    receive_line("STATUS",&c);
    receive_line("SETPOINT:2.5",&c);
    CHECK(fabsf(c.tuner_setpoint_deg-2.5f)<1e-6f);
    CHECK(fabsf(c.balance.config.trim_deg+3.5f)<1e-5f);
    receive_line("SETPOINT:50",&c);
    CHECK(fabsf(c.tuner_setpoint_deg-5.0f)<1e-6f);
    CHECK(fabsf(c.balance.config.trim_deg+1.0f)<1e-5f);
    feed(-4.0f,1,true);
    CHECK(balance_tick(&c)==0 && pwm[0]!=0);
    receive_line("STANDBY",&c);
    CHECK(!bridge && !c.enabled && c.balance.state==BALANCE_DISARMED);
    return 0;
}
#endif
int test_modes(void) {
    bridge=false; pwm[0]=pwm[1]=0; bridge_error=0;
    encoder.cps[0]=encoder.cps[1]=0;
    control_t c=new_control();
    CHECK(c.control_mode==CAR_MODE_STANDBY && !c.motion_owned);
    feed(0,0,true);
    receive_line("mode hold",&c); CHECK(!c.enabled); /* No calibration/gains. */
    receive_line("balance zero",&c);
    receive_line("pid attitude 4 0.1 0",&c);
    receive_line("mode drive",&c); CHECK(!c.enabled); /* No speed loop. */
    receive_line("mode hold",&c);
    CHECK(bridge && c.enabled && c.control_mode==CAR_MODE_HOLD && c.motion_owned);
    CHECK(c.balance.config.attitude.kp==4 && c.balance.config.attitude.ki==0.1f);
    feed(1,0,true); CHECK(balance_tick(&c)==0);
    CHECK(pwm[0]>0 && pwm[1]<0 && c.target_cps==0 && c.target_yaw_dps==0);
    float integral=c.balance.attitude.i;
    int64_t previous=c.balance.previous_imu_us;
    receive_line("mode hold",&c); /* Idempotent: do not reset PID or bridge. */
    CHECK(c.balance.attitude.i==integral && c.balance.previous_imu_us==previous && bridge);
    receive_line("move 100 0",&c); CHECK(c.requested_cps==0);
    receive_line("mode drive",&c); CHECK(c.control_mode==CAR_MODE_HOLD);
    receive_line("SET P:20 I:2 D:1",&c); CHECK(c.balance.config.attitude.kp==4);
    receive_line("RESET",&c); CHECK(c.balance.attitude.i==integral);
    receive_line("SETPOINT:2",&c); CHECK(c.balance.config.trim_deg==0);
    receive_line("mode tune",&c); CHECK(c.motion_owned);
    receive_line("standby",&c);
    CHECK(!bridge && c.motion_owned && c.control_mode==CAR_MODE_STANDBY);
    receive_line("SET P:20 I:2 D:1",&c); CHECK(!bridge && c.balance.config.attitude.kp==4);
    receive_line("balance arm",&c); CHECK(!bridge);
    receive_line("enable",&c); CHECK(!bridge);
    receive_line("pid speed 0.01 0 0",&c);
    receive_line("pid turn 0.1 0 0",&c);
    CHECK(c.balance.config.turn.kp==0.1f);
    feed(0,0,true);
    receive_line("mode drive",&c);
    CHECK(bridge && c.control_mode==CAR_MODE_DRIVE && c.target_cps==0);
    receive_line("move 100 20",&c);
    CHECK(c.requested_cps==100 && c.requested_yaw_dps==20 && c.target_cps==0);
    feed(0,0,true); CHECK(balance_tick(&c)==0);
    CHECK(fabsf(c.target_cps-3)<0.001f && fabsf(c.target_yaw_dps-0.9f)<0.001f);
    /* The outer speed loop runs every 20 ms, not every 10 ms IMU tick. */
    feed(0,0,true); CHECK(balance_tick(&c)==0);
    CHECK(c.balance_output.target_tilt_deg>0 && c.balance_output.turn_percent>0);
    CHECK(fabsf(c.balance_output.left_percent)<=20 && fabsf(c.balance_output.right_percent)<=20);
    float speed=c.target_cps, yaw=c.target_yaw_dps;
    CHECK(balance_tick(&c)==0 && c.target_cps==speed && c.target_yaw_dps==yaw); /* No duplicate sample. */
    int64_t last=c.last_move_us;
    const char *bad[]={"move nan 0","move 10 inf","move 301 0","move -301 0", "move 10 46",
        "move 10 -46", "move 10", "move 10 0 tail", "mode drive tail", "mode unknown"};
    for(unsigned i=0;i<sizeof(bad)/sizeof(bad[0]);++i) receive_line(bad[i],&c);
    CHECK(c.requested_cps==100 && c.requested_yaw_dps==20 && c.last_move_us==last);
    for(int i=0;i<10;++i) { feed(0,0,true); CHECK(balance_tick(&c)==0); }
    integral=c.balance.attitude.i; previous=c.balance.previous_imu_us; speed=c.target_cps;
    float outer=c.balance.speed_tilt_deg;
    receive_line("mode hold",&c);
    CHECK(bridge && c.control_mode==CAR_MODE_HOLD && c.requested_cps==0);
    CHECK(c.target_cps==speed && c.balance.attitude.i==integral && c.balance.previous_imu_us==previous);
    CHECK(c.balance.speed_tilt_deg==outer); /* Do not clear the outer loop either. */
    feed(0,0,true); CHECK(balance_tick(&c)==0 && c.target_cps<speed && c.target_cps>0);
    for(int i=0;i<20;++i) { feed(0,0,true); CHECK(balance_tick(&c)==0); }
    CHECK(c.target_cps==0 && c.target_yaw_dps==0 && bridge);
    receive_line("mode drive",&c);
    receive_line("move -100 -20",&c);
    for(int i=0;i<20;++i) { feed(0,0,true); CHECK(balance_tick(&c)==0); }
    CHECK(c.target_cps<0 && c.target_yaw_dps<0);
    /* A successful move refreshes the lease; STATUS/invalid commands do not. */
    receive_line("move -100 -20",&c);
    for(int i=0;i<40;++i) { feed(0,0,true); CHECK(balance_tick(&c)==0); }
    CHECK(c.control_mode==CAR_MODE_DRIVE);
    receive_line("mode status",&c);
    for(int i=0;i<11;++i) { feed(0,0,true); CHECK(balance_tick(&c)==0); }
    CHECK(c.control_mode==CAR_MODE_HOLD && c.requested_cps==0 && bridge);
    for(int i=0;i<60;++i) { feed(0,0,true); CHECK(balance_tick(&c)==0); }
    CHECK(c.target_cps==0 && c.target_yaw_dps==0 && c.balance.state==BALANCE_ARMED);
    publish_snapshot(&c);
    CHECK(snapshot.control_mode==CAR_MODE_HOLD && snapshot.target_cps==0);
    receive_line("mode drive",&c); receive_line("move 100 0",&c);
    feed(30,0,true); CHECK(balance_tick(&c)==0);
    CHECK(!bridge && !c.enabled && c.balance.state==BALANCE_FAULT_FALL);
    CHECK(c.control_mode==CAR_MODE_STANDBY && c.target_cps==0 && c.requested_cps==0);
    feed(0,0,true);
    receive_line("move 100 0",&c); receive_line("SET P:4 I:0.1 D:0",&c);
    CHECK(!bridge); /* Old joystick packets and tuning host cannot restart a fault. */
    receive_line("mode hold",&c); CHECK(bridge && c.control_mode==CAR_MODE_HOLD);
    feed(0,0,false); CHECK(balance_tick(&c)==0);
    CHECK(!bridge && c.balance.state==BALANCE_FAULT_INPUT);
    feed(0,0,true); receive_line("mode hold",&c); CHECK(bridge);
    clock_us+=60000; CHECK(balance_tick(&c)==0);
    CHECK(!bridge && c.balance.state==BALANCE_FAULT_STALE);
    feed(0,0,true); bridge_error=-1; receive_line("mode hold",&c);
    CHECK(!bridge && !c.enabled && c.control_mode==CAR_MODE_STANDBY);
    bridge_error=0;
    feed(0,0,true); receive_line("mode hold",&c); CHECK(bridge);
    receive_line("mode status",&c); receive_line("standby",&c);
    CHECK(!bridge && c.target_cps==0 && c.requested_yaw_dps==0);
    receive_line("mode tune",&c); CHECK(!c.motion_owned && !bridge);
    return 0;
}

int test_telemetry(void) {
    control_t c=new_control();
    CHECK(c.telemetry_mode==TELEMETRY_SLOW);
    c.telemetry_last_us=clock_us;
    CHECK(!telemetry_due(&c,clock_us+999999));
    CHECK(telemetry_due(&c,clock_us+1000000));
    CHECK(!telemetry_due(&c,clock_us+1000000));
    int replies=reply_count;
    receive_line("telemetry off",&c);
    CHECK(c.telemetry_mode==TELEMETRY_OFF && reply_count==replies+1);
    CHECK(!telemetry_due(&c,clock_us+10000000));
    CHECK(!car_motor_control_telemetry_fast());
    receive_line("mode status",&c); CHECK(reply_count>replies+1);
    receive_line("telemetry fast",&c);
    CHECK(c.telemetry_mode==TELEMETRY_FAST && car_motor_control_telemetry_fast());
#if CONFIG_CAR_PID_TUNER
    const int64_t fast_period=CONFIG_CAR_PID_TUNER_PERIOD_MS*1000;
#else
    const int64_t fast_period=TELEMETRY_ENCODER_PERIOD_US;
#endif
    CHECK(!telemetry_due(&c,clock_us+fast_period-1));
    CHECK(telemetry_due(&c,clock_us+fast_period));
    int64_t last=c.telemetry_last_us;
    receive_line("telemetry status",&c);
    receive_line("telemetry fast tail",&c);
    receive_line("telemetry",&c);
    receive_line("telemetry other",&c);
    CHECK(c.telemetry_mode==TELEMETRY_FAST && c.telemetry_last_us==last);
    receive_line("telemetry slow",&c);
    CHECK(c.telemetry_mode==TELEMETRY_SLOW && !car_motor_control_telemetry_fast());
    CHECK(!telemetry_due(&c,clock_us+999999));
#if CONFIG_CAR_PID_TUNER
    receive_line("STATUS",&c);
    CHECK(c.telemetry_mode==TELEMETRY_FAST && car_motor_control_telemetry_fast());
#endif
    /* Data suppression never suppresses control commands or fault replies. */
    bridge=false; bridge_error=0; encoder.cps[0]=encoder.cps[1]=0;
    feed(0,0,true);
    receive_line("balance zero",&c); receive_line("pid attitude 4 0.1 0",&c);
    receive_line("mode hold",&c); CHECK(bridge && c.enabled);
    c.balance.attitude.i=0.25f;
    int64_t previous=c.balance.previous_imu_us;
    receive_line("telemetry off",&c);
    receive_line("telemetry slow",&c);
    receive_line("telemetry fast",&c);
    receive_line("telemetry off",&c);
    CHECK(bridge && c.balance.attitude.i==0.25f && c.balance.previous_imu_us==previous);
    CHECK(c.control_mode==CAR_MODE_HOLD && c.balance.config.attitude.kp==4);
    replies=reply_count;
    feed(30,0,true); CHECK(balance_tick(&c)==0);
    CHECK(!bridge && c.balance.state==BALANCE_FAULT_FALL && reply_count>replies);
    CHECK(c.telemetry_mode==TELEMETRY_OFF);
    receive_line("standby",&c);
    CHECK(c.telemetry_mode==TELEMETRY_OFF && !bridge);
    return 0;
}

int test_diagnostic(void) {
    bridge=false; bridge_error=0; encoder.cps[0]=encoder.cps[1]=0;
    control_t c=new_control();
    receive_line("telemetry fast",&c);
    feed(0,0,true);
    c.balance.attitude.i=0.25f;
    receive_line("diag start",&c);
    const int64_t started=clock_us, end=c.diag_end_us;
    CHECK(end==started+DIAG_DURATION_US && !bridge && !c.enabled);
    CHECK(!car_motor_control_telemetry_fast() && c.telemetry_mode==TELEMETRY_FAST);
    CHECK(!diagnostic_due(&c,started+DIAG_PERIOD_US-1,started+DIAG_PERIOD_US-1));
    CHECK(diagnostic_due(&c,started+DIAG_PERIOD_US,started+DIAG_PERIOD_US));
    CHECK(!diagnostic_due(&c,started+2*DIAG_PERIOD_US,started+DIAG_PERIOD_US));
    CHECK(!diagnostic_due(&c,started+100000,started)); /* stale sample */
    CHECK(!diagnostic_due(&c,started+100000,started+100001)); /* future sample */
    CHECK(!diagnostic_due(&c,started+100000,0));
    receive_line("diag start tail",&c);
    receive_line("diag unknown",&c);
    CHECK(c.diag_end_us==end && c.balance.attitude.i==0.25f);
    CHECK(diagnostic_due(&c,end-1,end-1));
    CHECK(!diagnostic_due(&c,end,end) && c.diag_end_us==0);
    CHECK(c.telemetry_mode==TELEMETRY_FAST && c.telemetry_last_us==end);
    CHECK(!bridge && c.balance.attitude.i==0.25f);
    receive_line("diag start",&c);
    receive_line("diag stop",&c); CHECK(c.diag_end_us==0 && !bridge);
    receive_line("diag start",&c);
    receive_line("telemetry off",&c); CHECK(c.diag_end_us==0 && c.telemetry_mode==TELEMETRY_OFF);
    receive_line("balance zero",&c); receive_line("pid attitude 4 0.1 0",&c);
    receive_line("mode hold",&c); CHECK(bridge);
    feed(1,2,true); CHECK(balance_tick(&c)==0);
    CHECK(c.control_sample_us==clock_us && c.control_tilt_deg==1 && c.control_rate_dps==2);
    float integral=c.balance.attitude.i;
    int64_t previous=c.balance.previous_imu_us;
    receive_line("diag start",&c);
    CHECK(bridge && c.balance.attitude.i==integral && c.balance.previous_imu_us==previous);
    CHECK(c.balance.config.attitude.kp==4 && c.balance.config.attitude.ki==0.1f);
    receive_line("standby",&c);
    CHECK(!bridge && c.control_sample_us==0 && c.diag_end_us>0);
    receive_line("diag stop",&c);
    CHECK(!bridge && c.telemetry_mode==TELEMETRY_OFF);
    return 0;
}

'''
if '--no-tuner' in sys.argv:
    preamble = preamble.replace('#define CONFIG_CAR_PID_TUNER 1', '#define CONFIG_CAR_PID_TUNER 0')
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
if '--no-tuner' not in sys.argv:
    tuner=ctypes.CFUNCTYPE(ctypes.c_int)(engine.get_function_address('test_tuner'))()
    assert tuner==0, f'tuner check failed at generated harness line {tuner}'
print('PASS: zero/arm gates, PID commands, direction, mode exclusion, stale/fall/invalid stop, explicit rearm, bridge error')
if '--no-tuner' not in sys.argv:
    print('PASS: LLM-PID-Tuner SET/STATUS/SETPOINT/STANDBY, live gain clamp, auto-arm and trailing-token rejection')

modes=ctypes.CFUNCTYPE(ctypes.c_int)(engine.get_function_address('test_modes'))()
assert modes==0, f'mode check failed at generated harness line {modes}'
print('PASS: HOLD/DRIVE arm gates, PID preservation, target slew, command lease, mode exclusion, fault/manual rearm, turn setup, telemetry')

telemetry=ctypes.CFUNCTYPE(ctypes.c_int)(engine.get_function_address('test_telemetry'))()
assert telemetry==0, f'telemetry check failed at generated harness line {telemetry}'
print('PASS: telemetry OFF/SLOW/FAST cadence, strict commands, host handshake, replies/faults retained, PID/motor state preserved')

diagnostic=ctypes.CFUNCTYPE(ctypes.c_int)(engine.get_function_address('test_diagnostic'))()
assert diagnostic==0, f'diagnostic check failed at generated harness line {diagnostic}'
print('PASS: diagnostic 50 Hz/10 s bounds, unique/fresh samples, output restoration, early cancel, motor/PID isolation and matched control input')

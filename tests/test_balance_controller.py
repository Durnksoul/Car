"""Run actual PID/balance C with Clang + LLVM; no serial, board or flashing."""
import ctypes
from pathlib import Path
import subprocess
import sys
from llvmlite import binding as llvm

ROOT = Path(__file__).resolve().parents[1]
WORK = ROOT / "build/balance_host"
WORK.mkdir(parents=True, exist_ok=True)
# The embedded Clang installation has no Windows SDK headers. Use compiler
# builtins for classification/constants, and the host C runtime for min/max.
(WORK / "math.h").write_text('''#pragma once
#define isfinite(x) __builtin_isfinite(x)
#define fabsf(x) __builtin_fabsf(x)
#define NAN __builtin_nanf("")
#define INFINITY __builtin_inff()
float fminf(float, float);
float fmaxf(float, float);
''', encoding="utf-8")
source = r'''
#include <math.h>
#include <float.h>
#include "pid_controller.c"
#include "balance_controller.c"

#define CHECK(x) do { if (!(x)) return __LINE__; } while (0)
static int near(float a, float b) { return fabsf(a-b) < 0.0001f; }

int test_pid(void) {
    pid_config_t cfg = {.kp=2, .output_min=-10, .output_max=10,
                        .integral_min=-5, .integral_max=5};
    pid_controller_t p; float out;
    CHECK(pid_controller_init(&p, &cfg));
    CHECK(pid_controller_update(&p, 3, 1, 0.01f, &out) && near(out, 4));
    CHECK(pid_controller_update(&p, -3, 1, 0.01f, &out) && near(out, -8));
    CHECK(pid_controller_update(&p, 99, 1, 0.01f, &out) && near(out, 10));

    cfg.kp=0; cfg.kd=1;
    CHECK(pid_controller_init(&p, &cfg));
    CHECK(pid_controller_update(&p, 0, 1, 0.1f, &out) && near(out, 0));
    CHECK(pid_controller_update(&p, 100, 1, 0.1f, &out) && near(out, 0)); /* no kick */
    CHECK(pid_controller_update(&p, 100, 1.5f, 0.1f, &out) && near(out, -5));
    pid_controller_reset(&p);
    CHECK(pid_controller_update_with_rate(&p, 0, 0, 3, 0.01f, &out) && near(out, -3));
    cfg.derivative_cutoff_hz=1;
    CHECK(pid_controller_init(&p, &cfg));
    CHECK(pid_controller_update_with_rate(&p, 0, 0, 3, 0.01f, &out));
    CHECK(out < 0 && out > -3);

    cfg.kd=0; cfg.ki=1; cfg.derivative_cutoff_hz=0;
    CHECK(pid_controller_init(&p, &cfg));
    for (int n=0;n<1000;++n) CHECK(pid_controller_update(&p, 1, 0, 0.01f, &out));
    CHECK(near(p.i, 5)); /* independent integral clamp */
    cfg.kp=2;
    CHECK(pid_controller_init(&p, &cfg));
    for (int n=0;n<100;++n) CHECK(pid_controller_update(&p, 20, 0, 0.01f, &out));
    CHECK(near(out, 10) && near(p.i, 0)); /* no windup at output limit */
    CHECK(pid_controller_update(&p, 0, 0, 0.01f, &out) && near(out, 0));
    p.i=5;
    CHECK(pid_controller_update(&p, -1, 0, 0.01f, &out) && p.i<5); /* unwind */
    CHECK(!pid_controller_update(&p, NAN, 0, 0.01f, &out) && near(out, 0));
    CHECK(!pid_controller_update(&p, 0, 0, 0, &out) && near(out, 0));
    CHECK(!pid_controller_update(&p, 0, 0, -1, &out));
    CHECK(!pid_controller_update_with_rate(&p, 0, 0, INFINITY, .01f, &out));
    CHECK(!pid_controller_update(&p, 0, 0, .01f, 0));
    cfg.kp=FLT_MAX;
    CHECK(pid_controller_init(&p, &cfg));
    CHECK(!pid_controller_update(&p, 2, 0, .01f, &out) && near(out, 0));
    cfg.output_max=NAN;
    CHECK(!pid_controller_init(&p, &cfg));
    CHECK(!pid_controller_update(&p, 0, 0, .01f, &out));
    return 0;
}

static balance_input_t sample(void) {
    return (balance_input_t){.now_us=1000000, .imu_us=1000000, .encoder_us=1000000};
}
static void advance(balance_input_t *in, int us) {
    in->now_us+=us; in->imu_us=in->encoder_us=in->now_us;
}
int test_balance(void) {
    balance_config_t cfg=balance_config_default();
    balance_controller_t c; balance_output_t out;
    balance_input_t in=sample();
    CHECK(balance_controller_init(&c,&cfg));
    CHECK(!balance_controller_arm(&c,&in)); /* unconfirmed axes */
    cfg.mapping_confirmed=true;
    CHECK(balance_controller_init(&c,&cfg));
    CHECK(!balance_controller_arm(&c,&in)); /* no pretend-ready gains */
    cfg.attitude.kp=2; cfg.attitude.kd=1; cfg.attitude.derivative_cutoff_hz=0;
    CHECK(balance_controller_init(&c,&cfg));
    CHECK(!balance_controller_update(&c,&in,&out) && near(out.left_percent,0));
    in.tilt_deg=6;
    CHECK(!balance_controller_arm(&c,&in));
    in.tilt_deg=0;
    CHECK(balance_controller_arm(&c,&in));
    advance(&in,8333); in.tilt_deg=2; in.tilt_rate_dps=3;
    CHECK(balance_controller_update(&c,&in,&out));
    CHECK(near(out.attitude_percent,-7));
    CHECK(near(out.left_percent,-7) && near(out.right_percent,-7));
    CHECK(!balance_controller_update(&c,&in,&out)); /* repeated IMU timestamp */
    CHECK(c.state==BALANCE_FAULT_TIMING && near(out.right_percent,0));
    advance(&in,8333);
    CHECK(!balance_controller_update(&c,&in,&out)); /* fault stays latched */

    in=sample(); CHECK(balance_controller_arm(&c,&in));
    advance(&in,8333); in.tilt_deg=30;
    CHECK(!balance_controller_update(&c,&in,&out) && out.state==BALANCE_FAULT_FALL);
    CHECK(near(out.left_percent,0) && near(out.right_percent,0));
    in=sample(); CHECK(balance_controller_arm(&c,&in));
    in.now_us+=200000; in.imu_us=in.now_us;
    CHECK(!balance_controller_update(&c,&in,&out) && out.state==BALANCE_FAULT_STALE);
    in=sample(); CHECK(balance_controller_arm(&c,&in));
    advance(&in,60000);
    CHECK(!balance_controller_update(&c,&in,&out) && out.state==BALANCE_FAULT_TIMING);
    in=sample(); CHECK(balance_controller_arm(&c,&in));
    advance(&in,8333); in.left_cps=NAN;
    CHECK(!balance_controller_update(&c,&in,&out) && out.state==BALANCE_FAULT_INPUT);
    in=sample(); in.imu_us=in.now_us+1;
    CHECK(!balance_controller_arm(&c,&in)); /* future timestamp */

    cfg.speed.kp=.01f; cfg.encoder_sign[1]=-1;
    cfg.attitude.kd=0; cfg.turn.kp=1; cfg.turn.ki=1;
    CHECK(balance_controller_init(&c,&cfg));
    in=sample(); CHECK(balance_controller_arm(&c,&in));
    in.left_cps=100; in.right_cps=-100;
    advance(&in,10000);
    CHECK(balance_controller_update(&c,&in,&out) && near(out.target_tilt_deg,0));
    advance(&in,10000);
    CHECK(balance_controller_update(&c,&in,&out));
    CHECK(near(out.average_cps,100) && near(out.target_tilt_deg,-1));
    CHECK(near(out.left_percent,-2) && near(out.right_percent,-2));

    /* Mixer preserves balance authority and limits the turn component. */
    in=sample(); CHECK(balance_controller_arm(&c,&in));
    advance(&in,10000); in.tilt_deg=-14; in.target_yaw_rate_dps=10;
    CHECK(balance_controller_update(&c,&in,&out));
    CHECK(near(out.attitude_percent,28) && near(out.turn_percent,2));
    CHECK(near(out.left_percent,26) && near(out.right_percent,30));
    CHECK(near(c.turn.i,0)); /* no differential integral buildup */
    cfg.attitude_to_motor_sign=-1;
    CHECK(balance_controller_init(&c,&cfg));
    in=sample(); CHECK(balance_controller_arm(&c,&in));
    advance(&in,10000); in.tilt_deg=2;
    CHECK(balance_controller_update(&c,&in,&out) && near(out.attitude_percent,4));
    balance_controller_disarm(&c);
    CHECK(c.state==BALANCE_DISARMED && near(c.speed.i,0));
    cfg.max_output_percent=101;
    CHECK(!balance_controller_init(&c,&cfg));
    cfg=balance_config_default(); cfg.encoder_sign[0]=0;
    CHECK(!balance_controller_init(&c,&cfg));
    return 0;
}
'''
(WORK / "harness.c").write_text(source, encoding="utf-8")
compiler = sys.argv[1] if len(sys.argv) > 1 else (
    r"C:\Espressif\tools\esp-clang\esp-19.1.2_20250312\esp-clang\bin\clang.exe")
ir = WORK / "balance.ll"
subprocess.run([compiler, "-target", llvm.get_default_triple(), "-S", "-emit-llvm",
                "-O0", "-ffreestanding", "-Wall", "-Wextra", "-Werror",
                "-I", str(WORK), "-I", str(ROOT / "main"),
                str(WORK / "harness.c"), "-o", str(ir)], check=True)
llvm.initialize_native_target()
llvm.initialize_native_asmprinter()
runtime = ctypes.CDLL("ucrtbase.dll" if sys.platform == "win32" else None)
for symbol in ("memcpy", "memset", "fminf", "fmaxf"):
    llvm.add_symbol(symbol, ctypes.cast(getattr(runtime, symbol), ctypes.c_void_p).value)
module = llvm.parse_assembly(ir.read_text())
module.verify()
target = llvm.Target.from_default_triple().create_target_machine()
module.triple = llvm.get_default_triple()
module.data_layout = str(target.target_data)
engine = llvm.create_mcjit_compiler(module, target)
engine.finalize_object()
for name in ("test_pid", "test_balance"):
    result = ctypes.CFUNCTYPE(ctypes.c_int)(engine.get_function_address(name))()
    assert result == 0, f"{name}: C check failed at harness line {result}"
    print(f"PASS: {name}")

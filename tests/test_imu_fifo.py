"""Actual FIFO C against mocked register reads; never opens a serial port."""
import ctypes
from pathlib import Path
import subprocess
import sys
from llvmlite import binding as llvm

root = Path(__file__).resolve().parents[1]
work = root / "build/imu_fifo_host"
work.mkdir(parents=True, exist_ok=True)
(work / "esp_err.h").write_text('''#pragma once
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_ERR_INVALID_STATE 0x103
#define ESP_ERR_NOT_FOUND 0x105
#define ESP_ERR_INVALID_RESPONSE 0x108
''', encoding="utf-8")
(work / "math.h").write_text('''#pragma once
#define isfinite(x) __builtin_isfinite(x)
float ldexpf(float, int);
float sqrtf(float);
float atan2f(float, float);
float asinf(float);
float fminf(float, float);
float fmaxf(float, float);
''', encoding="utf-8")
(work / "string.h").write_text('''#pragma once
#include <stddef.h>
void *memcpy(void *, const void *, size_t);
void *memset(void *, int, size_t);
''', encoding="utf-8")
harness = r'''
#include "imu660rc_math.c"
#include "imu660rc_fifo.c"
#define CHECK(x) do { if (!(x)) return __LINE__; } while (0)
static unsigned records, reads, fail_record, status_error, overflow;
static uint8_t packets[511][7];
static esp_err_t mock(void *ctx, uint8_t reg, uint8_t *data, size_t len) {
    (void)ctx;
    if (reg==0x1b && len==2) {
        if (status_error) return -41;
        data[0]=records & 255; data[1]=(records >> 8) | overflow; return 0;
    }
    if (reg!=0x78 || len!=7 || reads>=records) return -42;
    if (++reads==fail_record) return -43;
    memcpy(data,packets[reads-1],7); return 0;
}
static void setup(unsigned count) {
    records=count; reads=fail_record=status_error=overflow=0;
    memset(packets,0,sizeof(packets));
    for (unsigned i=0;i<count;++i) packets[i][0]=0x13<<3;
}
int run_tests(void) {
    float out[3]={999,999,999}; uint8_t last[7]={0}, status[2]={0};
    setup(0);
    CHECK(imu660rc_fifo_read(mock,0,out,last,status)==ESP_ERR_NOT_FOUND);
    CHECK(out[0]==999 && reads==0);
    setup(1);
    CHECK(imu660rc_fifo_read(mock,0,out,last,status)==0 && out[0]==0 && out[2]==0);
    CHECK(reads==1 && last[0]==0x98);
    setup(3);
    packets[0][0]=0x16<<3; /* skip gyro bias, don't mistake it for a quaternion */
    packets[1][0]|=6; /* tag counter bits must not change sensor tag */
    packets[1][5]=0; packets[1][6]=0x38; /* Z=.5, yaw=60 degrees */
    packets[2][0]=0x17<<3; /* later non-attitude data doesn't replace it */
    CHECK(imu660rc_fifo_read(mock,0,out,last,status)==0 && reads==3);
    CHECK(out[2]>59.99f && out[2]<60.01f);
    setup(2); /* select newest attitude, not the first */
    packets[1][5]=0; packets[1][6]=0x38;
    CHECK(imu660rc_fifo_read(mock,0,out,last,status)==0 && out[2]>59.99f);
    setup(2); /* newest invalid cannot be hidden by an older valid sample */
    packets[1][1]=0; packets[1][2]=0x7c;
    out[0]=out[1]=out[2]=999;
    CHECK(imu660rc_fifo_read(mock,0,out,last,status)==ESP_ERR_INVALID_RESPONSE);
    CHECK(out[0]==999 && out[2]==999 && last[2]==0x7c);
    setup(1); packets[0][0]=0x16<<3;
    CHECK(imu660rc_fifo_read(mock,0,out,last,status)==ESP_ERR_NOT_FOUND);
    setup(511); /* high count bit, bounded complete snapshot */
    CHECK(imu660rc_fifo_read(mock,0,out,last,status)==0 && reads==511);
    setup(3); overflow=0x40; out[0]=999;
    CHECK(imu660rc_fifo_read(mock,0,out,last,status)==ESP_ERR_INVALID_STATE);
    CHECK(out[0]==999 && reads==3);
    setup(1); overflow=0x08;
    CHECK(imu660rc_fifo_read(mock,0,out,last,status)==ESP_ERR_INVALID_STATE);
    setup(1); status_error=1;
    CHECK(imu660rc_fifo_read(mock,0,out,last,status)==-41);
    setup(3); fail_record=2; out[0]=999;
    CHECK(imu660rc_fifo_read(mock,0,out,last,status)==-43 && out[0]==999);
    setup(1);
    CHECK(imu660rc_fifo_read(mock,0,out,last,status)==0); /* recover on next fresh batch */
    return 0;
}
'''
(work / "harness.c").write_text(harness, encoding="utf-8")
compiler = r"C:\Espressif\tools\esp-clang\esp-19.1.2_20250312\esp-clang\bin\clang.exe"
ir = work / "fifo.ll"
subprocess.run([compiler, "-target", llvm.get_default_triple(), "-S", "-emit-llvm",
                "-O0", "-ffreestanding", "-Wall", "-Wextra", "-Werror", "-I", str(work),
                "-I", str(root / "components/imu660rc"), str(work / "harness.c"),
                "-o", str(ir)], check=True)
llvm.initialize_native_target()
llvm.initialize_native_asmprinter()
runtime = ctypes.CDLL("ucrtbase.dll" if sys.platform == "win32" else None)
for name in ("sqrtf", "atan2f", "asinf", "fminf", "fmaxf", "memcpy", "memset"):
    llvm.add_symbol(name, ctypes.cast(getattr(runtime,name),ctypes.c_void_p).value)
# ldexpf is not exported by all Windows CRT versions.
import math
@ctypes.CFUNCTYPE(ctypes.c_float,ctypes.c_float,ctypes.c_int)
def ldexp(value, exponent):
    return math.ldexp(value, exponent)
llvm.add_symbol("ldexpf",ctypes.cast(ldexp,ctypes.c_void_p).value)
module = llvm.parse_assembly(ir.read_text()); module.verify()
target = llvm.Target.from_default_triple().create_target_machine()
module.triple = llvm.get_default_triple(); module.data_layout = str(target.target_data)
engine = llvm.create_mcjit_compiler(module,target); engine.finalize_object()
result = ctypes.CFUNCTYPE(ctypes.c_int)(engine.get_function_address("run_tests"))()
assert result==0, f"C check failed at harness line {result}"
print("PASS: atomic 7-byte FIFO records, tags/counters, newest sample, invalid/overrun/error handling, 511-record bound")

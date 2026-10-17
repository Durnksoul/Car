"""Host verification of the actual C math function via Clang IR + llvmlite.

Usage: python tests/test_angles.py [prebuilt-LLVM-IR-path]
No sensor, serial connection or firmware flashing is involved.
"""
import ctypes
import math
from pathlib import Path
import struct
import sys
import subprocess
import random

from llvmlite import binding as llvm

root = Path(__file__).resolve().parents[1]
if len(sys.argv) > 1:
    ir_path = Path(sys.argv[1])
else:
    work = root / "build/imu_math_host"
    work.mkdir(parents=True, exist_ok=True)
    (work / "math.h").write_text('''#pragma once
#define isfinite(x) __builtin_isfinite(x)
float ldexpf(float, int);
float sqrtf(float);
float atan2f(float, float);
float asinf(float);
float fmaxf(float, float);
float fminf(float, float);
''', encoding="utf-8")
    (work / "string.h").write_text('''#pragma once
#include <stddef.h>
void *memcpy(void *, const void *, size_t);
void *memset(void *, int, size_t);
''', encoding="utf-8")
    ir_path = work / "math.ll"
    compiler = r"C:\Espressif\tools\esp-clang\esp-19.1.2_20250312\esp-clang\bin\clang.exe"
    subprocess.run([compiler, "-target", llvm.get_default_triple(), "-S", "-emit-llvm",
                    "-O0", "-ffreestanding", "-Wall", "-Wextra", "-Werror", "-I", str(work),
                    str(root / "components/imu660rc/imu660rc_math.c"), "-o", str(ir_path)], check=True)

llvm.initialize_native_target()
llvm.initialize_native_asmprinter()
runtime = ctypes.CDLL("ucrtbase.dll")
for symbol in ("sqrtf", "atan2f", "asinf", "fminf", "fmaxf", "memcpy", "memset"):
    llvm.add_symbol(symbol, ctypes.cast(getattr(runtime, symbol), ctypes.c_void_p).value)

@ctypes.CFUNCTYPE(ctypes.c_float, ctypes.c_float, ctypes.c_int)
def host_ldexpf(value, exponent):
    return math.ldexp(value, exponent)

llvm.add_symbol("ldexpf", ctypes.cast(host_ldexpf, ctypes.c_void_p).value)

# Newlib's isfinite macro calls __fpclassifyf; supply its enum contract on host.
@ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_float)
def classify(value):
    if math.isnan(value):
        return 0
    if math.isinf(value):
        return 1
    return 2 if value == 0 else 4

llvm.add_symbol("__fpclassifyf", ctypes.cast(classify, ctypes.c_void_p).value)
module = llvm.parse_assembly(ir_path.read_text())
module.verify()
target = llvm.Target.from_default_triple().create_target_machine()
module.triple = llvm.get_default_triple()
module.data_layout = str(target.target_data)
engine = llvm.create_mcjit_compiler(module, target)
engine.finalize_object()
decode = ctypes.CFUNCTYPE(ctypes.c_bool, ctypes.POINTER(ctypes.c_uint8),
                          ctypes.POINTER(ctypes.c_float))(
    engine.get_function_address("imu660rc_decode_angles"))

def run(data):
    output = (ctypes.c_float * 3)(999, 999, 999)
    valid = decode((ctypes.c_uint8 * 8).from_buffer_copy(data), output)
    return valid, list(output)

def check(values, expected, tolerance=0.03):
    valid, actual = run(struct.pack("<4e", *values))
    assert valid, values
    for got, want in zip(actual, expected):
        assert math.isfinite(got) and abs(got - want) < tolerance, (values, actual, expected)

check((0, 0, 0, 1), (0, 0, 0))
check((0, 0, 0, -1), (0, 0, 0))
s = math.sqrt(0.5)
check((s, 0, 0, s), (0, 0, 90))
check((-s, 0, 0, s), (0, 0, 270))
check((0, s, 0, s), (0, -90, 0))
check((0, -s, 0, s), (0, 90, 0))
check((0, 0, s, s), (90, 0, 0))
check((0, 0, -s, s), (-90, 0, 0))

# Sweep every binary16 encoding, including subnormals, NaN and infinity.
# With only raw component 0 and W populated, yaw has an independent analytic
# reference: 2*atan2(raw0, W), wrapped to [0,360).
for bits in range(65536):
    data = struct.pack("<4H", bits, 0, 0, 0x3c00)
    value = struct.unpack("<e", struct.pack("<H", bits))[0]
    valid, actual = run(data)
    if not math.isfinite(value):
        assert not valid and actual == [999, 999, 999], bits
        continue
    expected = math.degrees(2 * math.atan2(value, 1.0)) % 360
    difference = abs((actual[2] - expected + 180) % 360 - 180)
    assert valid and difference < 0.03, (bits, actual, expected)

for values in ((0, 0, 0, 0), (0, 0, 0, 2**-24),
               (math.nan, 0, 0, 1), (0, math.inf, 0, 1)):
    valid, actual = run(struct.pack("<4e", *values))
    assert not valid and actual == [999, 999, 999], (values, actual)
print("PASS: axis rotations, pitch singularity, 65536 binary16 encodings, invalid data")

decode_fifo = ctypes.CFUNCTYPE(ctypes.c_bool, ctypes.POINTER(ctypes.c_uint8),
                               ctypes.POINTER(ctypes.c_float))(
    engine.get_function_address("imu660rc_decode_fifo_angles"))

def fifo_run(values):
    data = struct.pack("<3e", *values)
    output = (ctypes.c_float * 3)(999, 999, 999)
    valid = decode_fifo((ctypes.c_uint8 * 6).from_buffer_copy(data), output)
    return valid, list(output)

# Native XYZ corresponds to legacy page order Z/X/Y/W. Verify that the
# source change retains existing output axes rather than rotating the viewer.
for values, expected in [((0, 0, 0), (0, 0, 0)),
                         ((s, 0, 0), (0, -90, 0)),
                         ((-s, 0, 0), (0, 90, 0)),
                         ((0, s, 0), (90, 0, 0)),
                         ((0, -s, 0), (-90, 0, 0)),
                         ((0, 0, s), (0, 0, 90)),
                         ((0, 0, -s), (0, 0, 270))]:
    valid, actual = fifo_run(values)
    assert valid and all(abs(a-b) < .03 for a,b in zip(actual,expected)), (values,actual)

# Quantization can push a 180-degree quaternion slightly outside the unit
# sphere. Accept a bounded rounding excess, reject grossly invalid values.
valid, actual = fifo_run((s, s, 0))
assert valid and all(map(math.isfinite, actual))
for values in ((math.inf,-math.inf,math.inf), (math.nan,0,0), (1,1,1), (2,0,0)):
    valid, actual = fifo_run(values)
    assert not valid and actual == [999,999,999], (values,actual)
print("PASS: FIFO XYZ identity/axis/sign mapping, 180-degree rounding, Inf/NaN/range rejection")

decode_balance = ctypes.CFUNCTYPE(ctypes.c_bool, ctypes.POINTER(ctypes.c_uint8),
                                 ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float),
                                 ctypes.POINTER(ctypes.c_float))(
    engine.get_function_address("imu660rc_decode_balance"))

def qmul(a, b):
    ax,ay,az,aw = a; bx,by,bz,bw = b
    return (aw*bx+ax*bw+ay*bz-az*by,
            aw*by-ax*bz+ay*bw+az*bx,
            aw*bz+ax*by-ay*bx+az*bw,
            aw*bw-ax*bx-ay*by-az*bz)

def axis_q(axis, degrees):
    half = math.radians(degrees) / 2
    v = [0,0,0,math.cos(half)]; v[axis] = math.sin(half)
    return tuple(v)

def balance_run(q, gyro=(0,0,0)):
    if q[3] < 0: q = tuple(-v for v in q)
    data = struct.pack("<3e", *q[:3])
    angle, rate = ctypes.c_float(999), ctypes.c_float(999)
    valid = decode_balance((ctypes.c_uint8*6).from_buffer_copy(data),
                           (ctypes.c_float*3)(*gyro), ctypes.byref(angle), ctypes.byref(rate))
    return valid, angle.value, rate.value, data

def delta_angle(a, b):
    return (a-b+180) % 360 - 180

# The same physical forward tilt at different world headings must give the
# same balance angle, even though the old display Euler angles can couple.
for heading in (-170,-120,-55,0,55,120,170,180):
    for tilt in (-45,-20,-5,0,5,20,45):
        q = qmul(axis_q(2,heading), axis_q(1,tilt))
        valid, angle, rate, _ = balance_run(q,(0,12,0))
        assert valid and abs(delta_angle(angle,tilt)) < .2, (heading,tilt,angle)
        assert abs(rate-12) < .001, (heading,tilt,rate)
valid, angle, rate, _ = balance_run(axis_q(1,180))
assert valid and abs(delta_angle(angle,-180)) < .01

# Independent finite-difference reference: evolve the quaternion with a body
# angular-rate rotation. Check the derivative including off-axis components.
def projected_angle(q):
    x,y,z,w = q
    return math.degrees(math.atan2(2*(w*y-x*z), 1-2*(x*x+y*y)))

random.seed(660)
for _ in range(300):
    q = qmul(qmul(axis_q(2,random.uniform(-120,120)),
                  axis_q(1,random.uniform(-40,40))),
                  axis_q(0,random.uniform(-30,30)))
    gyro = tuple(random.uniform(-30,30) for _ in range(3))
    valid, angle, rate, data = balance_run(q,gyro)
    assert valid
    # Use the quantized quaternion reconstructed from the exact C input.
    xyz = list(struct.unpack("<3e",data))
    total = sum(v*v for v in xyz)
    if total > 1:
        xyz = [v/math.sqrt(total) for v in xyz]; total=1
    qh = tuple(xyz+[math.sqrt(1-total)])
    assert abs(delta_angle(angle,projected_angle(qh))) < .005
    epsilon = 1e-5
    norm = math.sqrt(sum(v*v for v in gyro))
    half = math.radians(norm)*epsilon/2
    dq = tuple(v/norm*math.sin(half) for v in gyro)+(math.cos(half),)
    plus = projected_angle(qmul(qh,dq))
    minus = projected_angle(qmul(qh,tuple(-v for v in dq[:3])+(dq[3],)))
    reference = delta_angle(plus,minus)/(2*epsilon)
    assert abs(rate-reference) < .005, (gyro,rate,reference)

for q, gyro in [(axis_q(0,90),(0,0,0)), # wheel axle vertical: projection undefined
                ((math.inf,0,0,1),(0,0,0)),
                ((0,0,0,1),(0,math.nan,0)),
                ((0,0,0,1),(0,0,math.inf))]:
    valid, angle, rate, _ = balance_run(q,gyro)
    assert not valid and angle==999 and rate==999, (q,gyro,valid,angle,rate)
print("PASS: balance tilt heading invariance/sign, 300 projected-rate references, singular/invalid inputs")

"""Host verification of the actual C math function via Clang IR + llvmlite.

Usage: python tests/test_angles.py build/imu660rc_math_host.ll
No sensor, serial connection or firmware flashing is involved.
"""
import ctypes
import math
from pathlib import Path
import struct
import sys

from llvmlite import binding as llvm

llvm.initialize_native_target()
llvm.initialize_native_asmprinter()
runtime = ctypes.CDLL("ucrtbase.dll")
for symbol in ("sqrtf", "atan2f", "asinf"):
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
module = llvm.parse_assembly(Path(sys.argv[1]).read_text())
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

"""Host test of the actual serial command parser via Clang + llvmlite.
Run: python tests/test_tb6612_command.py [path/to/clang.exe]
No serial port, board or firmware flashing is involved.
"""
import ctypes
from pathlib import Path
import shutil
import subprocess
import sys

from llvmlite import binding as llvm

root = Path(__file__).resolve().parents[1]
work = root / "build/tb6612_command_host"
work.mkdir(parents=True, exist_ok=True)
(work / "esp_err.h").write_text(
    "typedef int esp_err_t;\n#define ESP_OK 0\n#define ESP_ERR_INVALID_ARG 0x102\n",
    encoding="utf-8")
compiler = sys.argv[1] if len(sys.argv) > 1 else (shutil.which("clang") or
    r"C:\Espressif\tools\esp-clang\esp-19.1.2_20250312\esp-clang\bin\clang.exe")
ir = work / "command.ll"
subprocess.run([compiler, "-target", llvm.get_default_triple(), "-S", "-emit-llvm",
                "-O0", "-ffreestanding", "-Wall", "-Wextra", "-Werror", "-I",
                str(work), str(root / "main/tb6612_command.c"), "-o", str(ir)], check=True)
llvm.initialize_native_target()
llvm.initialize_native_asmprinter()
runtime = ctypes.CDLL("ucrtbase.dll" if sys.platform == "win32" else None)
for name in ("memcpy", "memset"):
    llvm.add_symbol(name, ctypes.cast(getattr(runtime, name), ctypes.c_void_p).value)
module = llvm.parse_assembly(ir.read_text())
stream_ir = work / "stream.ll"
subprocess.run([compiler, "-target", llvm.get_default_triple(), "-S", "-emit-llvm",
                "-O0", "-ffreestanding", "-Wall", "-Wextra", "-Werror", "-I",
                str(work), str(root / "main/car_command_stream.c"), "-o", str(stream_ir)], check=True)
module.link_in(llvm.parse_assembly(stream_ir.read_text()))
module.verify()
target = llvm.Target.from_default_triple().create_target_machine()
module.triple = llvm.get_default_triple()
module.data_layout = str(target.target_data)
engine = llvm.create_mcjit_compiler(module, target)
engine.finalize_object()

class Command(ctypes.Structure):
    _fields_ = [("kind", ctypes.c_int), ("mask", ctypes.c_uint),
                ("a", ctypes.c_int), ("b", ctypes.c_int)]

parse = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_char_p, ctypes.POINTER(Command))(
    engine.get_function_address("tb6612_command_parse"))

valid = {
    b"enc": (7, 0, 0, 0), b"ENCZERO": (8, 0, 0, 0),
    b"help": (0, 0, 0, 0), b"status": (1, 0, 0, 0),
    b"enable": (2, 0, 0, 0), b"standby": (3, 0, 0, 0),
    b"a 25": (4, 1, 25, 0), b"b -50": (4, 2, 0, -50),
    b"ab 25 -50": (4, 3, 25, -50), b"AB +100 -100": (4, 3, 100, -100),
    b"a 0": (4, 1, 0, 0), b"\t B\t-0 \r\n": (4, 2, 0, 0),
    b"coast a": (5, 1, 0, 0), b"coast b": (5, 2, 0, 0),
    b"coast all": (5, 3, 0, 0), b"brake A": (6, 1, 0, 0),
    b"brake b": (6, 2, 0, 0), b"brake all": (6, 3, 0, 0),
}
for text, expected in valid.items():
    output = Command()
    assert parse(text, ctypes.byref(output)) == 0, text
    assert (output.kind, output.mask, output.a, output.b) == expected, text
for value in range(-100, 101):
    output = Command()
    assert parse(f"ab {value} {-value}".encode(), ctypes.byref(output)) == 0
    assert (output.a, output.b) == (value, -value)
invalid = [b"", b" ", b"auto", b"a", b"a 101", b"b -101", b"a 25.5", b"a 25%",
           b"enc all", b"enczero 1",
           b"a +", b"a --1", b"a 1x", b"ab 25", b"ab 25 -50 extra", b"help extra",
           b"enable 1", b"coast", b"coast c", b"brake both", b"A25", b"a 0x10",
           b"ab 25 999999999999999999999", b"a " + b"0" * 62, None]
for text in invalid:
    output = Command(123, 456, 789, 987)
    assert parse(text, ctypes.byref(output)) == 0x102, text
    assert (output.kind, output.mask, output.a, output.b) == (123, 456, 789, 987), text
assert parse(b"help", None) == 0x102

class Stream(ctypes.Structure):
    _fields_ = [("line", ctypes.c_char * 128), ("used", ctypes.c_size_t), ("discard", ctypes.c_bool)]

lines = []
callback_type = ctypes.CFUNCTYPE(None, ctypes.c_char_p, ctypes.c_void_p)
callback = callback_type(lambda line, ctx: lines.append(line))
feed = ctypes.CFUNCTYPE(None, ctypes.POINTER(Stream), ctypes.c_char_p, ctypes.c_size_t,
                       callback_type, ctypes.c_void_p)(engine.get_function_address("car_command_stream_feed"))
stream = Stream()
def send(data):
    feed(ctypes.byref(stream), data, len(data), callback, None)

# Every packet boundary preserves complete commands; CRLF executes only once.
message = b"ab 50 -50\r\nenc\nstandby\r"
for split in range(len(message) + 1):
    stream = Stream()
    lines.clear()
    send(message[:split]); send(message[split:])
    assert lines == [b"ab 50 -50", b"enc", b"standby"], split
lines.clear(); stream = Stream()
send(b"a 5"); assert not lines
send(b"1\b0\n"); assert lines == [b"a 50"]
lines.clear(); stream = Stream()
send(b"a " + b"0" * 126 + b"\b\nenc\n")
assert lines == [None, b"enc"]
lines.clear(); stream = Stream()
send(b"ab 50\x00 50\nenc\n")
assert lines == [None, b"enc"]
# Connection changes discard partial data, and separate transports never mix.
lines.clear(); stream = Stream()
send(b"ab 50 "); stream = Stream(); send(b"standby\n")
assert lines == [b"standby"]
lines.clear(); stream = Stream(); send(b"a 5")
other = Stream()
feed(ctypes.byref(other), b"b 20\n", 5, callback, None)
send(b"0\n")
assert lines == [b"b 20", b"a 50"]
print("PASS: all commands, percentages -100..100, case/whitespace, malformed/overflow "
      "inputs rejected; BLE packet splits, CRLF, backspace, invalid/long lines, stream reset and isolation")

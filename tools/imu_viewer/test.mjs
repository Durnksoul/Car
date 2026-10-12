import assert from "node:assert/strict";
import { parseAngles, LineDecoder } from "./dist/protocol.js";
import { rotateVector, moduleToScene } from "./dist/scene.js";
import { SerialConnection } from "./dist/serial.js";
import { connectionMessage } from "./dist/connection-message.js";

const encoder = new TextEncoder();
assert.deepEqual(parseAngles("roll=1.23,pitch=-2.34,yaw=45.67"), {roll:1.23,pitch:-2.34,yaw:45.67});
assert.deepEqual(parseAngles("\x1b[32m roll=+1e1, pitch=.5, yaw=-179.9\x1b[0m"), {roll:10,pitch:.5,yaw:-179.9});
for (const line of ["I (100) imu_app: Yaw zero recorded: 170", "roll=nan,pitch=0,yaw=0", "roll=1e999,pitch=0,yaw=0", "roll=999,pitch=0,yaw=0", "roll=1,pitch=2", "roll=1,pitch=2,yaw=3 trailing"]) assert.equal(parseAngles(line), null);
const received = [];
const decoder = new LineDecoder(line => received.push(line));
const text = "启动提示\r\nroll=1,pitch=2,yaw=-179.9\r\nroll=3,pitch=4,yaw=179.9\n";
for (const byte of encoder.encode(text)) decoder.push(Uint8Array.of(byte));
assert.deepEqual(received, ["启动提示","roll=1,pitch=2,yaw=-179.9","roll=3,pitch=4,yaw=179.9"]);
decoder.push(encoder.encode("x".repeat(5000))); decoder.push(encoder.encode("roll=9,pitch=9,yaw=9\nroll=0,pitch=0,yaw=0\n"));
assert.equal(received.at(-1), "roll=0,pitch=0,yaw=0");
assert.equal(received.length, 4);
function near(actual, expected) { actual.forEach((value,i)=>assert.ok(Math.abs(value-expected[i]) < 1e-6)); }
near(rotateVector([1,0,0], {roll:0,pitch:0,yaw:90}), [0,1,0]);
near(rotateVector([0,1,0], {roll:0,pitch:90,yaw:0}), [0,0,-1]);
near(rotateVector([1,0,0], {roll:90,pitch:0,yaw:0}), [0,0,-1]);
// Match the module's silkscreen, including signs and positive-axis rotations.
near(moduleToScene([1,0,0]), [0,1,0]);
near(moduleToScene([0,1,0]), [-1,0,0]);
near(moduleToScene([0,0,1]), [0,0,1]);
near(rotateVector(moduleToScene([0,1,0]), {roll:90,pitch:0,yaw:0}), [0,0,1]);
near(rotateVector(moduleToScene([1,0,0]), {roll:0,pitch:90,yaw:0}), [0,0,-1]);
near(rotateVector(moduleToScene([1,0,0]), {roll:0,pitch:0,yaw:90}), moduleToScene([0,1,0]));
const v1 = rotateVector([1,0,0], {roll:0,pitch:0,yaw:179.9});
const v2 = rotateVector([1,0,0], {roll:0,pitch:0,yaw:-179.9});
assert.ok(Math.hypot(...v1.map((x,i)=>x-v2[i])) < .004);

class MockPort {
  async open(options) {
    assert.equal(options.baudRate,115200);
    assert.equal(options.flowControl,"none");
    this.readable = new ReadableStream({ start: controller => { this.controller = controller; } });
    this.closed = false;
  }
  async setSignals(signals) { assert.deepEqual(signals,{dataTerminalReady:false,requestToSend:false}); }
  async close() { assert.equal(this.readable.locked,false); this.closed = true; }
}
let port = new MockPort();
let openCount = 0, closeCount = 0, lastError = null;
const lines = [];
const serial = new SerialConnection({requestPort:async()=>port}, {
  onOpen:()=>{ openCount++; }, onLine:line=>lines.push(line), onClose:error=>{ closeCount++; lastError=error; },
});
await serial.connect(115200);
port.controller.enqueue(encoder.encode("roll=0,pitch=0,"));
port.controller.enqueue(encoder.encode("yaw=0\r\n"));
await new Promise(resolve=>setTimeout(resolve,0));
assert.deepEqual(lines,["roll=0,pitch=0,yaw=0"]);
await serial.disconnect();
assert.equal(port.closed,true); assert.equal(serial.port,null); assert.equal(lastError,null);
port = new MockPort(); await serial.connect(115200);
port.controller.error(new Error("USB removed")); await serial.readPromise;
assert.equal(port.closed,true); assert.equal(serial.port,null); assert.match(lastError.message,/USB removed/);
port = new MockPort(); await serial.connect(115200); await serial.disconnect();
assert.equal(openCount,3); assert.equal(closeCount,3);
const denied = new SerialConnection({requestPort:async()=>{throw new Error("cancelled");}}, {onOpen(){},onLine(){},onClose(){}});
await assert.rejects(denied.connect(115200),/cancelled/); assert.equal(denied.port,null);

// A browser which never returns its chooser must not leave the UI promise pending.
const shortTimeouts = {select:10,open:10,signals:10};
const emptyCallbacks = {onOpen(){},onLine(){},onClose(){}};
let finishSelection;
let lateOpenCalls = 0;
const stalled = new SerialConnection({requestPort:()=>new Promise(resolve=>{finishSelection=resolve;})}, emptyCallbacks, shortTimeouts);
await assert.rejects(stalled.connect(115200), error => {
  assert.equal(error.code,"select_timeout");
  assert.equal(error.stage,"select");
  assert.match(connectionMessage(error).message,/独立浏览器/);
  return true;
});
assert.equal(stalled.connecting,false);
await assert.rejects(stalled.connect(115200), error=>error.code==="previous_request_pending");
finishSelection({open(){lateOpenCalls++;}});
await new Promise(resolve=>setTimeout(resolve,0));
assert.equal(lateOpenCalls,0); assert.equal(stalled.pickerPending,false);
let finishOpen;
const delayedPort = new MockPort();
delayedPort.open = ()=>new Promise(resolve=>{finishOpen=resolve;});
delayedPort.close = async()=>{delayedPort.closed=true;};
const slowOpen = new SerialConnection({requestPort:async()=>delayedPort}, emptyCallbacks, shortTimeouts);
await assert.rejects(slowOpen.connect(115200), error=>error.code==="open_timeout");
assert.equal(slowOpen.port,null); assert.equal(slowOpen.connecting,false);
finishOpen(); await new Promise(resolve=>setTimeout(resolve,0));
assert.equal(delayedPort.closed,true); assert.equal(slowOpen.openPending,false);
const occupiedPort = new MockPort();
occupiedPort.open = async()=>{throw new DOMException("Access denied","NetworkError");};
const occupied = new SerialConnection({requestPort:async()=>occupiedPort}, emptyCallbacks, shortTimeouts);
await assert.rejects(occupied.connect(115200), error=> {
  assert.equal(error.stage,"open"); assert.match(connectionMessage(error).message,/其他程序占用/); return true;
});
const cancelled = new SerialConnection({requestPort:async()=>{throw new DOMException("Cancelled","NotFoundError");}},emptyCallbacks,shortTimeouts);
await assert.rejects(cancelled.connect(115200), error=>connectionMessage(error).status==="未连接");
console.log("PASS: framing, axes, reconnect, chooser timeout, late selection, late open cleanup and error messages");

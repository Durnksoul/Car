import { parseAngles } from "./protocol.js";
import { SerialConnection } from "./serial.js";
import { AttitudeScene } from "./scene.js";
import { connectionMessage } from "./connection-message.js";

const $ = id => document.getElementById(id);
const localViewer = ["127.0.0.1", "localhost", "[::1]"].includes(location.hostname);
const browserAdvice = localViewer ? "点击「用独立浏览器打开」" : "点击「复制网址」，粘贴到独立 Chrome 或 Edge 地址栏";
$("open-browser").textContent = localViewer ? "用独立浏览器打开" : "复制网址";
const scene = new AttitudeScene($("attitude"));
let mode = "idle", busy = false, demoTimer = null;
let busyLabel = "请稍候…";
let lastSampleAt = 0, rateStartedAt = 0, frames = 0, frameRate = 0;
const logLines = [];

function message(text = "") { $("message").textContent = text; $("message").hidden = !text; }
function status(text, kind, note) {
  $("status").textContent = text; $("status-dot").className = `status-dot ${kind}`;
  $("connection-note").textContent = note;
}
function controls() {
  $("connect").textContent = busy ? busyLabel : mode === "serial" ? "断开连接" : "连接串口";
  $("connect").disabled = busy || !navigator.serial || !window.isSecureContext;
  $("baud").disabled = busy || mode === "serial";
  $("demo").disabled = busy || mode === "serial";
  $("demo").textContent = mode === "demo" ? "停止演示" : "查看演示";
  $("demo-label").hidden = mode !== "demo";
}
function showSample(angles) {
  const now = performance.now();
  if (!lastSampleAt) rateStartedAt = now;
  lastSampleAt = now;
  frames++;
  if (now - rateStartedAt >= 1000) {
    frameRate = frames * 1000 / (now - rateStartedAt);
    frames = 0; rateStartedAt = now;
  }
  for (const [name, range] of [["roll",180],["pitch",90],["yaw",180]]) {
    const angle = Math.abs(angles[name]) < .005 ? 0 : angles[name];
    $(name).textContent = angle.toFixed(2);
    $(`${name}-indicator`).style.left = `${50 + Math.max(-1, Math.min(1, angle / range)) * 50}%`;
  }
  $("scene-overlay").hidden = true;
  $("source-label").textContent = mode === "demo" ? "演示数据" : "串口实时数据";
  scene.update(angles);
  if (mode === "serial") status("已连接 · 正在接收", "live", "上电零点由 ESP32 记录，网页显示相对姿态。");
}
function addLine(line) {
  logLines.push(line);
  if (logLines.length > 80) logLines.shift();
  $("log").textContent = logLines.join("\n");
  $("log").scrollTop = $("log").scrollHeight;
}
function resetDisplay() {
  lastSampleAt = 0; frames = 0; frameRate = 0;
  for (const name of ["roll","pitch","yaw"]) {
    $(name).textContent = "—"; $(`${name}-indicator`).style.left = "50%";
  }
  scene.update({roll:0,pitch:0,yaw:0});
  $("source-label").textContent = "等待数据";
  $("scene-overlay").textContent = "连接串口，开始观察";
  $("scene-overlay").hidden = false;
  $("rate").textContent = "— 帧/秒";
}
function stopDemo() {
  if (demoTimer !== null) clearInterval(demoTimer);
  demoTimer = null;
  if (mode === "demo") { mode = "idle"; resetDisplay(); }
}

const connection = new SerialConnection(navigator.serial, {
  onStage(stage) {
    if (stage === "select") {
      busyLabel = "等待选择…";
      status("等待选择串口", "paused", "请在浏览器弹出的设备窗口中选择开发板端口。");
    } else {
      message();
      busyLabel = "正在打开…";
      status("正在打开串口", "paused", "已选择设备，正在打开端口。");
    }
    controls();
  },
  onOpen() {
    message();
    mode = "serial"; resetDisplay();
    logLines.length = 0; $("log").textContent = "等待串口消息…";
    status("已连接 · 等待数据", "paused", "保持静止，等待 ESP32 完成启动归零。");
    $("scene-overlay").textContent = "等待 ESP32 输出姿态";
    controls();
  },
  onLine(line) {
    addLine(line);
    const angles = parseAngles(line);
    if (angles) showSample(angles);
  },
  onClose(error) {
    mode = "idle";
    status("已断开", "", "重新连接前，请确认串口没有被其他软件占用。");
    $("source-label").textContent = lastSampleAt ? "最后一帧 · 已断开" : "等待数据";
    $("rate").textContent = "— 帧/秒";
    if (error) message(`串口读取中断：${error.message}。请检查 USB 连接后重新连接。`);
    controls();
  },
});

$("connect").addEventListener("click", async () => {
  if (busy) return;
  stopDemo(); message(); busy = true; controls();
  busyLabel = mode === "serial" ? "正在断开…" : "等待选择…";
  controls();
  const chooserHint = setTimeout(() => {
    if (busy && busyLabel === "等待选择…") {
      message(`还没有出现设备选择窗口？${browserAdvice}，在普通浏览器窗口中连接开发板。`);
    }
  }, 8000);
  try {
    if (mode === "serial") await connection.disconnect();
    else await connection.connect(Number($("baud").value));
  } catch (error) {
    const detail = connectionMessage(error, browserAdvice);
    status(detail.status, detail.message ? "paused" : "", detail.note);
    message(detail.message);
  } finally { clearTimeout(chooserHint); busy = false; controls(); }
});

$("open-browser").addEventListener("click", async () => {
  const button = $("open-browser");
  button.disabled = true;
  try {
    if (!localViewer) {
      const url = location.origin + location.pathname;
      try { await navigator.clipboard.writeText(url); }
      catch { message(`请复制 ${url} 到独立 Chrome 或 Edge 地址栏后连接。`); return; }
      message("网址已复制，请粘贴到独立 Chrome 或 Edge 地址栏，打开后连接开发板串口。");
      return;
    }
    const response = await fetch("/open-browser", {
      method: "POST", headers: { "Content-Type": "application/json" }, body: "{}",
      signal: AbortSignal.timeout(5000),
    });
    if (!response.ok) throw new Error("请关闭网页启动窗口，再双击 StartViewer.cmd 更新服务");
    const result = await response.json();
    if (!result.ok) throw new Error(result.error);
    message(`已请求在 ${result.browser} 打开当前页面，请在新窗口点击「连接串口」。`);
  } catch (error) {
    message(`打开独立浏览器失败：${error.message}。也可复制 ${location.origin} 到 Chrome 或 Edge 地址栏。`);
  } finally { button.disabled = false; }
});

$("demo").addEventListener("click", () => {
  if (mode === "serial" || busy) return;
  if (mode === "demo") {
    stopDemo(); status("未连接", "", "连接前，请关闭 VS Code 串口监视器。");
  } else {
    message(); resetDisplay(); mode = "demo";
    status("演示模式", "demo", "当前为模拟姿态，未读取传感器。");
    const start = performance.now();
    const tick = () => {
      const t = (performance.now() - start) / 1000;
      showSample({roll:Math.sin(t*.65)*30,pitch:Math.sin(t*.8)*22,yaw:Math.sin(t*.3)*100});
    };
    tick(); demoTimer = setInterval(tick, 50);
  }
  controls();
});

setInterval(() => {
  if (mode !== "serial" && mode !== "demo") return;
  if (lastSampleAt && performance.now() - lastSampleAt > 1500) {
    status("已连接 · 数据暂停", "paused", "显示最后一帧；检查传感器接线或 ESP32 的启动提示。");
    $("source-label").textContent = "最后一帧 · 数据暂停";
    $("rate").textContent = "0 帧/秒";
  } else if (lastSampleAt) {
    $("rate").textContent = `${frameRate.toFixed(0)} 帧/秒`;
  }
}, 500);
window.addEventListener("pagehide", () => { stopDemo(); void connection.disconnect(); });
if (!navigator.serial || !window.isSecureContext) {
  message("此浏览器无法连接串口。请用 Chrome 或 Edge 打开本地页面；仍可点击「查看演示」预览。");
}
controls();

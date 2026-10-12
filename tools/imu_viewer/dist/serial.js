import { LineDecoder } from "./protocol.js";

export class ConnectionError extends Error {
  constructor(message, stage, code = "connection_failed", cause) {
    super(message, { cause });
    this.stage = stage;
    this.code = code;
  }
}

function withTimeout(promise, milliseconds, stage, onTimeout = () => {}) {
  return new Promise((resolve, reject) => {
    const timer = setTimeout(() => {
      onTimeout();
      reject(new ConnectionError("等待串口操作超时", stage, `${stage}_timeout`));
    }, milliseconds);
    Promise.resolve(promise).then(
      value => { clearTimeout(timer); resolve(value); },
      error => { clearTimeout(timer); reject(error); },
    );
  });
}

export class SerialConnection {
  constructor(serial, { onOpen, onLine, onClose, onStage = () => {} },
              timeouts = { select: 30000, open: 10000, signals: 1000 }) {
    this.serial = serial;
    this.onOpen = onOpen;
    this.onLine = onLine;
    this.onClose = onClose;
    this.onStage = onStage;
    this.timeouts = timeouts;
    this.port = null;
    this.reader = null;
    this.readPromise = null;
    this.closing = false;
    this.connecting = false;
    this.pickerPending = false;
    this.openPending = false;
  }
  async connect(baudRate) {
    if (this.port || this.connecting) throw new ConnectionError("串口已经连接或正在连接", "open", "already_connecting");
    if (this.pickerPending || this.openPending) {
      throw new ConnectionError("上一次浏览器请求仍未结束，请关闭选择窗口或刷新页面", "select", "previous_request_pending");
    }
    this.connecting = true;
    let stage = "select";
    try {
      this.onStage(stage);
      // Keep requestPort in the original click gesture, before any await.
      const selection = this.serial.requestPort();
      this.pickerPending = true;
      selection.then(() => { this.pickerPending = false; }, () => { this.pickerPending = false; });
      const port = await withTimeout(selection, this.timeouts.select, stage);
      // A chooser result arriving after timeout is never opened.
      stage = "open";
      this.onStage(stage);
      this.openPending = true;
      let abandoned = false;
      let opening;
      try {
        opening = port.open({ baudRate, dataBits: 8, stopBits: 1, parity: "none", flowControl: "none" });
      } catch (error) { this.openPending = false; throw error; }
      const opened = Promise.resolve(opening).then(async () => {
        if (abandoned) {
          // Opening itself cannot be cancelled by Web Serial. Close a port
          // that eventually opens after timeout so it cannot remain occupied.
          try { await port.close(); } finally { this.openPending = false; }
        } else { this.openPending = false; }
      }, error => { this.openPending = false; throw error; });
      await withTimeout(opened, this.timeouts.open, stage, () => { abandoned = true; });
      this.port = port;
      this.closing = false;
      // Some ports do not expose control signals; this optional operation
      // must not block the whole connection forever either.
      try {
        await withTimeout(port.setSignals({ dataTerminalReady: false, requestToSend: false }),
                          this.timeouts.signals, "signals");
      } catch { /* Continue without optional control signals. */ }
      this.onOpen();
      this.readPromise = this.readLoop(port);
    } catch (error) {
      if (error instanceof ConnectionError) throw error;
      throw new ConnectionError(error.message, stage, "connection_failed", error);
    } finally { this.connecting = false; }
  }
  async readLoop(port) {
    const lines = new LineDecoder(this.onLine);
    let error = null;
    try {
      if (!port.readable) throw new Error("串口没有可读取的数据流");
      this.reader = port.readable.getReader();
      while (!this.closing) {
        const { value, done } = await this.reader.read();
        if (done) break;
        lines.push(value);
      }
    } catch (err) {
      if (!this.closing) error = err;
    } finally {
      this.reader?.releaseLock();
      this.reader = null;
      try { await port.close(); } catch (err) { error ??= err; }
      this.port = null;
      this.onClose(error);
    }
  }
  async disconnect() {
    if (!this.port) return;
    this.closing = true;
    try { await this.reader?.cancel(); } catch { /* A removed USB device may already have closed the stream. */ }
    await this.readPromise;
  }
}

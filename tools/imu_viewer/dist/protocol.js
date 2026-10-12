// Accept the firmware's complete angle lines; ignore ESP-IDF boot/log messages.
const number = String.raw`([+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?)`;
const anglesPattern = new RegExp(`^\\s*roll\\s*=\\s*${number}\\s*,\\s*pitch\\s*=\\s*${number}\\s*,\\s*yaw\\s*=\\s*${number}\\s*$`, "i");
export function stripAnsi(text) {
  return text.replace(/\x1b\[[0-9;]*[A-Za-z]/g, "");
}
export function parseAngles(line) {
  const match = anglesPattern.exec(stripAnsi(line));
  if (!match) return null;
  const [roll, pitch, yaw] = match.slice(1).map(Number);
  if (![roll, pitch, yaw].every(Number.isFinite)) return null;
  if (Math.abs(roll) > 180.1 || Math.abs(pitch) > 90.1 || Math.abs(yaw) > 360.1) return null;
  return { roll, pitch, yaw };
}
export class LineDecoder {
  constructor(onLine) {
    this.decoder = new TextDecoder();
    this.pending = "";
    this.onLine = onLine;
    this.discarding = false;
  }
  push(bytes) {
    this.pending += this.decoder.decode(bytes, { stream: true });
    let newline;
    while ((newline = this.pending.indexOf("\n")) >= 0) {
      const line = this.pending.slice(0, newline).replace(/\r$/, "");
      this.pending = this.pending.slice(newline + 1);
      if (!this.discarding && line.length <= 4096) this.onLine(stripAnsi(line));
      this.discarding = false;
    }
    // Discard oversized/incomplete garbage through its next line break.
    if (this.pending.length > 4096) {
      this.pending = "";
      this.discarding = true;
    }
  }
}

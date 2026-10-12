export function connectionMessage(error, browserAdvice = "点击「用独立浏览器打开」") {
  const nativeName = error.cause?.name || error.name;
  if (nativeName === "NotFoundError" && error.stage === "select") {
    return { status: "未连接", note: "未选择串口，点击连接可重新选择。", message: "" };
  }
  if (error.code === "select_timeout" || error.code === "previous_request_pending") {
    return {
      status: "串口选择未完成", note: "浏览器没有返回设备选择结果。",
      message: `如果没有出现设备选择窗口，${browserAdvice}。如果选择窗口还开着，请先关闭它，再重试；必要时刷新页面。`,
    };
  }
  if (error.code === "open_timeout") {
    return { status: "打开串口超时", note: "设备已选择，但未完成打开。",
      message: "关闭 VS Code 串口监视器和其他串口软件，检查 USB 连接后重试。" };
  }
  if (nativeName === "SecurityError" || nativeName === "NotAllowedError") {
    return { status: "浏览器未允许连接", note: "检查浏览器的设备授权窗口。",
      message: "请在独立 Chrome 或 Edge 普通窗口中连接，并允许访问开发板串口。" };
  }
  if (error.stage === "select") {
    return { status: "无法选择串口", note: "尚未打开任何串口。",
      message: `浏览器没有完成设备选择：${error.message}。请${browserAdvice}后重试。` };
  }
  return { status: "无法打开串口", note: "设备已选择，打开时失败。",
    message: `打开失败：${error.message}。确认端口未被其他程序占用、USB 连接正常，波特率为 115200。` };
}

const rad = Math.PI / 180;
// Reconstruct the current driver's Euler convention: Ry(roll)*Rx(-pitch)*Rz(yaw).
// These are the original scene coordinates, used for the board geometry.
// In the module's silkscreen frame, Roll is around +X and Pitch around +Y.
export function rotateVector([x, y, z], { roll, pitch, yaw }) {
  const cy = Math.cos(yaw * rad), sy = Math.sin(yaw * rad);
  [x, y] = [x * cy - y * sy, x * sy + y * cy];
  const cp = Math.cos(-pitch * rad), sp = Math.sin(-pitch * rad);
  [y, z] = [y * cp - z * sp, y * sp + z * cp];
  const cr = Math.cos(roll * rad), sr = Math.sin(roll * rad);
  return [x * cr + z * sr, y, -x * sr + z * cr];
}

// Module silkscreen: new +X = old +Y, new +Y = old -X, +Z unchanged.
// Change the local basis, keeping the board and serial attitude aligned.
export function moduleToScene([x, y, z]) {
  return [-y, x, z];
}

export class AttitudeScene {
  constructor(canvas) {
    this.canvas = canvas;
    this.context = canvas.getContext("2d");
    this.angles = { roll: 0, pitch: 0, yaw: 0 };
    this.pending = false;
    this.observer = new ResizeObserver(() => this.requestDraw());
    this.observer.observe(canvas);
    this.requestDraw();
  }
  update(angles) { this.angles = angles; this.requestDraw(); }
  requestDraw() {
    if (this.pending) return;
    this.pending = true;
    requestAnimationFrame(() => { this.pending = false; this.draw(); });
  }
  draw() {
    const ctx = this.context;
    const { width, height } = this.canvas.getBoundingClientRect();
    const ratio = Math.min(window.devicePixelRatio || 1, 2);
    if (this.canvas.width !== Math.round(width * ratio) || this.canvas.height !== Math.round(height * ratio)) {
      this.canvas.width = Math.round(width * ratio);
      this.canvas.height = Math.round(height * ratio);
    }
    ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
    ctx.clearRect(0, 0, width, height);
    const scale = Math.min(width / 9.2, height / 7.2);
    const origin = [width / 2, height * .51];
    const project = ([x, y, z]) => [origin[0] + (x - y) * .7071 * scale,
      origin[1] + ((x + y) * .3394 - z * .88) * scale,
      (x + y) * .6223 + z * .48];
    const pose = v => project(rotateVector(v, this.angles));
    const line = (a, b, color, thickness = 1) => {
      ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]);
      ctx.strokeStyle = color; ctx.lineWidth = thickness; ctx.stroke();
    };
    // Fixed world grid and soft ground shadow.
    for (let i = -5; i <= 5; i++) {
      line(project([i, -5, -1.2]), project([i, 5, -1.2]), "#edf1f7");
      line(project([-5, i, -1.2]), project([5, i, -1.2]), "#edf1f7");
    }
    ctx.save(); ctx.translate(origin[0], origin[1] + scale * 1.35); ctx.scale(1, .4);
    const shadow = ctx.createRadialGradient(0, 0, 12, 0, 0, scale * 3.2);
    shadow.addColorStop(0, "#bbc9dd55"); shadow.addColorStop(1, "#bbc9dd00");
    ctx.fillStyle = shadow; ctx.beginPath(); ctx.arc(0, 0, scale * 3.2, 0, Math.PI * 2); ctx.fill(); ctx.restore();
    const vertices = [[-1.3,-2,0],[1.3,-2,0],[1.3,2,0],[-1.3,2,0],
      [-1.3,-2,.16],[1.3,-2,.16],[1.3,2,.16],[-1.3,2,.16]];
    const projected = vertices.map(pose);
    const faces = [
      { indices: [0,3,2,1], fill: "#24495c" },
      { indices: [0,1,5,4], fill: "#345669" },
      { indices: [1,2,6,5], fill: "#3d6276" },
      { indices: [2,3,7,6], fill: "#345669" },
      { indices: [3,0,4,7], fill: "#3d6276" },
      { indices: [4,5,6,7], fill: "#467386", top: true },
    ];
    faces.sort((a, b) => a.indices.reduce((s,i)=>s+projected[i][2],0) - b.indices.reduce((s,i)=>s+projected[i][2],0));
    const polygon = (points, fill, stroke = "#ffffff22") => {
      ctx.beginPath(); points.forEach((p,i) => i ? ctx.lineTo(p[0],p[1]) : ctx.moveTo(p[0],p[1])); ctx.closePath();
      ctx.fillStyle = fill; ctx.fill(); ctx.strokeStyle = stroke; ctx.lineWidth = 1; ctx.stroke();
    };
    for (const face of faces) {
      polygon(face.indices.map(i => projected[i]), face.fill);
      if (face.top) {
        // Chip, contacts and a front arrow all stay attached to the board.
        polygon([[-.55,-.65,.17],[.55,-.65,.17],[.55,.55,.17],[-.55,.55,.17]].map(pose), "#203548");
        for (let i = 0; i < 6; i++) {
          polygon([[-1.06+i*.36,-1.75,.17],[-.87+i*.36,-1.75,.17],[-.87+i*.36,-1.43,.17],[-1.06+i*.36,-1.43,.17]].map(pose), "#e4c87c");
        }
        polygon([[0,1.64,.17],[-.28,1.16,.17],[.28,1.16,.17]].map(pose), "#d8f3eb");
      }
    }
    const center = pose([0,0,.22]);
    for (const [vector, label, color] of [[[2.85,0,.22],"X","#e36469"],[[0,2.25,.22],"Y","#25ab83"],[[0,0,2.3],"Z","#447aec"]]) {
      const end = pose(moduleToScene(vector)); line(center, end, color, 2);
      ctx.fillStyle = color; ctx.beginPath(); ctx.arc(end[0],end[1],3.5,0,Math.PI*2); ctx.fill();
      ctx.font = "600 13px Segoe UI, sans-serif"; ctx.fillText(label,end[0]+9,end[1]+4);
    }
  }
}

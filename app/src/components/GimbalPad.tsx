// Fallback gimbal buttons (the main control is dragging on the video).
import { api } from "../api/client";
import { useCommand } from "../hooks/useCommand";
import { useRobot } from "../state/robotStore";

export function GimbalPad(): JSX.Element {
  const g = useRobot((s) => s.telemetry?.gimbal);
  const pan = useCommand("pan");
  const step = (dp: number, dt: number): void => {
    const p = (((g?.pan_deg ?? 0) + dp + 540) % 360) - 180;
    const t = Math.max(-90, Math.min(90, (g?.tilt_deg ?? 0) + dt));
    void pan.send(p, () => api.gimbal(p, t));
  };
  const b = "min-h-touch min-w-touch rounded-lg bg-white/15 text-lg";
  return (
    <div className="grid grid-cols-3 gap-1 w-36" aria-label="Camera pan and tilt">
      <span />
      <button className={b} aria-label="tilt up" onClick={() => step(0, 15)}>▲</button>
      <span />
      <button className={b} aria-label="pan left" onClick={() => step(-30, 0)}>◀</button>
      <button className={b} aria-label="centre camera" onClick={() => void pan.send(0, () => api.gimbal(0, 0))}>●</button>
      <button className={b} aria-label="pan right" onClick={() => step(30, 0)}>▶</button>
      <span />
      <button className={b} aria-label="tilt down" onClick={() => step(0, -15)}>▼</button>
      <span className="text-[10px] text-hud-dim self-center">{pan.pending ? "..." : ""}</span>
    </div>
  );
}

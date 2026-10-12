// Thumb joystick: vertical = speed along the pipe, horizontal = bend assist. While held it
// streams drive commands at 10 Hz (short TTL); releasing or losing the page stops the robot.
import { useEffect, useRef, useState } from "react";

import { ControlSocket } from "../api/telemetrySocket";
import { api } from "../api/client";
import { useRobot } from "../state/robotStore";

const RADIUS = 56;

export function DriveJoystick({ disabled }: { disabled?: boolean }): JSX.Element {
  const vmax = useRobot((s) => s.config?.max_speed_mm_s ?? 60);
  const [knob, setKnob] = useState({ x: 0, y: 0 });
  const held = useRef(false);
  const vec = useRef({ x: 0, y: 0 });
  const sock = useRef(new ControlSocket());
  const origin = useRef({ x: 0, y: 0 });

  useEffect(() => {
    const s = sock.current;
    s.open();
    const id = setInterval(() => {
      if (!held.current) return;
      const v = -vec.current.y * vmax;
      const turn = vec.current.x * 15;
      const msg = { type: "drive", linear_mm_s: Math.round(v), turn_deg_s: Math.round(turn), bend_dir_deg: turn >= 0 ? 90 : 270, ttl_ms: 300 };
      if (!s.send(msg)) void api.drive(msg.linear_mm_s, Math.abs(msg.turn_deg_s), msg.bend_dir_deg);
    }, 100);
    const stop = () => {
      if (held.current) void api.stop();
      held.current = false;
    };
    window.addEventListener("blur", stop);
    document.addEventListener("visibilitychange", stop);
    return () => {
      clearInterval(id);
      window.removeEventListener("blur", stop);
      document.removeEventListener("visibilitychange", stop);
      s.close();
    };
  }, [vmax]);

  const update = (cx: number, cy: number): void => {
    let x = (cx - origin.current.x) / RADIUS;
    let y = (cy - origin.current.y) / RADIUS;
    const m = Math.hypot(x, y);
    if (m > 1) {
      x /= m;
      y /= m;
    }
    if (Math.abs(x) < 0.15) x = 0; // dead zones
    if (Math.abs(y) < 0.1) y = 0;
    vec.current = { x, y };
    setKnob({ x: x * RADIUS, y: y * RADIUS });
  };
  const release = (): void => {
    held.current = false;
    vec.current = { x: 0, y: 0 };
    setKnob({ x: 0, y: 0 });
    void api.stop();
  };
  const speed = Math.round(-knob.y / RADIUS * vmax);

  return (
    <div className="flex flex-col items-center gap-1">
      <div
        role="slider"
        aria-label="Drive joystick"
        aria-valuenow={speed}
        aria-valuemin={-vmax}
        aria-valuemax={vmax}
        aria-disabled={disabled}
        className={`relative w-36 h-36 rounded-full bg-white/10 border border-white/25 touch-none ${disabled ? "opacity-40" : ""}`}
        onPointerDown={(e) => {
          if (disabled) return;
          (e.target as Element).setPointerCapture(e.pointerId);
          const r = e.currentTarget.getBoundingClientRect();
          origin.current = { x: r.left + r.width / 2, y: r.top + r.height / 2 };
          held.current = true;
          update(e.clientX, e.clientY);
        }}
        onPointerMove={(e) => held.current && update(e.clientX, e.clientY)}
        onPointerUp={release}
        onPointerCancel={release}
      >
        <div className="absolute left-1/2 top-2 -translate-x-1/2 text-[10px] text-hud-dim">FWD</div>
        <div className="absolute left-1/2 bottom-2 -translate-x-1/2 text-[10px] text-hud-dim">REV</div>
        <div
          className="absolute w-16 h-16 rounded-full bg-white/80 shadow-lg left-1/2 top-1/2"
          style={{ transform: `translate(calc(-50% + ${knob.x}px), calc(-50% + ${knob.y}px))` }}
        />
      </div>
      <div className="text-xs hud-text">{speed !== 0 ? `${speed} mm/s` : "hold to drive"}</div>
    </div>
  );
}

// Always-visible E-STOP. Requires a 500 ms press (no accidental triggers), shows progress,
// then a confirmed state. While active it offers CLEAR (needs a second long-press).
import { useEffect, useRef, useState } from "react";

import { api } from "../api/client";
import { useRobot } from "../state/robotStore";

export const HOLD_MS = 500;

export function EStopButton({ onTrigger = api.estop, onClear = api.clearFault }: {
  onTrigger?: () => Promise<{ status: string }>;
  onClear?: () => Promise<{ status: string }>;
}): JSX.Element {
  const faults = useRobot((s) => s.telemetry?.faults ?? []);
  const active = faults.includes("ESTOP");
  const [progress, setProgress] = useState(0);
  const [sent, setSent] = useState<"" | "sending" | "confirmed" | "failed">("");
  const start = useRef<number | null>(null);
  const raf = useRef<number | null>(null);

  useEffect(() => () => {
    if (raf.current !== null) cancelAnimationFrame(raf.current);
  }, []);
  useEffect(() => {
    if (active) setSent("confirmed");
  }, [active]);

  const tick = (): void => {
    if (start.current === null) return;
    const p = Math.min(1, (performance.now() - start.current) / HOLD_MS);
    setProgress(p);
    if (p >= 1) {
      start.current = null;
      setProgress(0);
      setSent("sending");
      (active ? onClear() : onTrigger()).then((r) => {
        if (active) setSent(r.status === "OK" ? "" : "failed");
        else setSent(r.status === "ESTOPPED" ? "confirmed" : "failed");
      });
      return;
    }
    raf.current = requestAnimationFrame(tick);
  };
  const down = (): void => {
    start.current = performance.now();
    raf.current = requestAnimationFrame(tick);
  };
  const up = (): void => {
    start.current = null;
    setProgress(0);
    if (raf.current !== null) cancelAnimationFrame(raf.current);
  };
  const label = active ? "HOLD TO CLEAR" : sent === "sending" ? "STOPPING..." : sent === "failed" ? "RETRY!" : "E-STOP";
  return (
    <button
      type="button"
      aria-label={active ? "E-stop active. Hold to clear" : "Emergency stop. Hold half a second"}
      data-testid="estop"
      className={`relative w-24 h-24 rounded-full font-black text-white shadow-2xl select-none touch-none border-4 ${active ? "bg-red-900 border-yellow-300" : "bg-red-600 border-red-300"}`}
      onPointerDown={down}
      onPointerUp={up}
      onPointerLeave={up}
      onPointerCancel={up}
      onKeyDown={(e) => (e.key === " " || e.key === "Enter") && !start.current && down()}
      onKeyUp={up}
    >
      <svg className="absolute inset-0" viewBox="0 0 100 100" aria-hidden>
        <circle cx="50" cy="50" r="46" fill="none" stroke="white" strokeWidth="6" strokeDasharray={`${progress * 289} 289`} transform="rotate(-90 50 50)" />
      </svg>
      <span className="relative text-sm leading-tight">{label}</span>
      {active && <span className="absolute -bottom-5 left-0 right-0 text-[10px] text-yellow-300">STOPPED</span>}
    </button>
  );
}

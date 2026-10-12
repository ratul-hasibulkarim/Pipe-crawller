// Arm control: grip (force mode), go to a diameter, retract. Shows the robot's measured
// diameter next to the requested one; the request stays "pending" until confirmed.
import { useState } from "react";

import { api } from "../api/client";
import { useCommand } from "../hooks/useCommand";
import { useRobot } from "../state/robotStore";
import { fmt } from "./format";

export function ExpansionControl({ disabled }: { disabled?: boolean }): JSX.Element {
  const cfg = useRobot((s) => s.config);
  const t = useRobot((s) => s.telemetry);
  const lo = cfg?.pipe_id_min_mm ?? 100;
  const hi = cfg?.pipe_id_max_mm ?? 200;
  const [target, setTarget] = useState<number>(Math.round((lo + hi) / 2));
  const dia = useCommand("diameter");
  const [msg, setMsg] = useState("");
  const busy = t?.state === "EXPANDING" || t?.state === "RETRACTING";
  const run = async (fn: () => Promise<{ status: string }>, label: string): Promise<void> => {
    const r = await fn();
    setMsg(r.status === "OK" ? label : r.status);
  };
  return (
    <div className="flex flex-col gap-2 w-44 text-sm">
      <div className="flex justify-between hud-text">
        <span>Ø {fmt(t?.est_diameter_mm, 0, "mm")}</span>
        <span className={dia.pending ? "text-hud-warn" : dia.failed ? "text-hud-bad" : "text-hud-dim"}>
          {dia.pending ? `→ ${target}` : dia.failed ? dia.failed : ""}
        </span>
      </div>
      <input
        type="range"
        min={lo}
        max={hi}
        step={1}
        value={target}
        disabled={disabled}
        aria-label="Target pipe diameter (mm)"
        className="w-full h-11 accent-emerald-400"
        onChange={(e) => setTarget(Number(e.target.value))}
        onPointerUp={() => !disabled && void dia.send(target, () => api.expand("DIAMETER", target))}
        onKeyUp={(e) => e.key.startsWith("Arrow") && !disabled && void dia.send(target, () => api.expand("DIAMETER", target))}
      />
      <div className="grid grid-cols-2 gap-2">
        <button
          className="min-h-touch rounded-lg bg-emerald-600 font-semibold disabled:opacity-40"
          disabled={disabled}
          onClick={() => run(() => api.expand("FORCE", cfg?.default_grip_n), "gripping")}
        >
          GRIP
        </button>
        <button
          className="min-h-touch rounded-lg bg-white/15 font-semibold disabled:opacity-40"
          disabled={disabled || t?.state === "CRAWLING"}
          onClick={() => run(() => api.expand("RETRACT"), "retracting")}
        >
          RETRACT
        </button>
      </div>
      <div className="text-xs text-hud-dim hud-text h-4">{busy ? `${t?.state?.toLowerCase()}...` : msg}</div>
    </div>
  );
}

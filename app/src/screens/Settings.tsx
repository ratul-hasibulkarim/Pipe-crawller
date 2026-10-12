import { useState } from "react";

import { api } from "../api/client";
import { useRobot } from "../state/robotStore";

const PARAMS: { name: string; label: string; step: number }[] = [
  { name: "DRIVE_KP", label: "Drive PID Kp", step: 0.05 },
  { name: "DRIVE_KI", label: "Drive PID Ki", step: 0.1 },
  { name: "DRIVE_KD", label: "Drive PID Kd", step: 0.01 },
  { name: "EXPAND_KP", label: "Grip force gain (rad/s per N)", step: 0.001 },
  { name: "DRIVE_CURRENT_A", label: "Drive current limit (A)", step: 0.05 },
  { name: "EXPAND_CURRENT_A", label: "Expansion current limit (A)", step: 0.05 },
  { name: "WHEEL_DIA_MM", label: "Wheel diameter calibration (mm)", step: 0.1 },
  { name: "GRIP_TARGET_N", label: "Grip target per arm (N)", step: 1 },
];

export function Settings(): JSX.Element {
  const settings = useRobot((s) => s.settings);
  const setSettings = useRobot((s) => s.setSettings);
  const cfg = useRobot((s) => s.config);
  const [values, setValues] = useState<Record<string, string>>({});
  const [result, setResult] = useState<Record<string, string>>({});
  const apply = async (name: string): Promise<void> => {
    const v = Number(values[name]);
    if (!Number.isFinite(v)) return;
    const r = await api.setParam(name, v);
    setResult({ ...result, [name]: r.status });
  };
  return (
    <div className="p-3 h-full overflow-auto space-y-4 max-w-xl mx-auto">
      <h2 className="font-semibold">Settings</h2>
      <section className="space-y-2">
        <h3 className="text-sm text-hud-dim">Display</h3>
        <label className="flex justify-between items-center min-h-touch">Units
          <select className="bg-white/10 rounded min-h-touch px-2" value={settings.units} onChange={(e) => setSettings({ units: e.target.value as "metric" | "imperial" })}>
            <option value="metric">metric</option>
            <option value="imperial">imperial</option>
          </select>
        </label>
        <label className="flex justify-between items-center min-h-touch">HUD overlay
          <input type="checkbox" className="w-6 h-6" checked={settings.hud} onChange={(e) => setSettings({ hud: e.target.checked })} />
        </label>
      </section>
      <section className="space-y-1 text-sm">
        <h3 className="text-sm text-hud-dim">Robot ({cfg?.size ?? "?"}) - limits are enforced on the MCU</h3>
        <div>Drive TTL {cfg?.drive_ttl_ms} ms · max speed {cfg?.max_speed_mm_s} mm/s · grip ≤ {cfg?.max_grip_n} N · tether {cfg?.tether_length_m} m</div>
      </section>
      <section className="space-y-2">
        <h3 className="text-sm text-hud-dim">Tuning / calibration (sent to the MCU, confirmed by ACK)</h3>
        {PARAMS.map((p) => (
          <div key={p.name} className="flex items-center gap-2">
            <label className="flex-1 text-sm" htmlFor={p.name}>{p.label}</label>
            <input id={p.name} type="number" step={p.step} className="w-24 min-h-touch bg-white/10 rounded px-2"
              value={values[p.name] ?? ""} onChange={(e) => setValues({ ...values, [p.name]: e.target.value })} />
            <button className="min-h-touch rounded bg-white/15 px-3" onClick={() => void apply(p.name)}>Set</button>
            <span className={`w-20 text-xs ${result[p.name] === "OK" ? "text-hud-ok" : "text-hud-bad"}`}>{result[p.name] ?? ""}</span>
          </div>
        ))}
        <button className="min-h-touch rounded bg-white/15 px-3" onClick={() => void api.setParam("ODOM_RESET", 1)}>Reset odometry to 0 m</button>
      </section>
    </div>
  );
}

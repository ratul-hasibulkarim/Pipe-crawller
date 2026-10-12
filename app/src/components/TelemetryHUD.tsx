// Telemetry overlay. Values come only from robot telemetry; when stale they are dimmed and
// labelled, never shown as live.
import { useStale } from "../hooks/useTelemetry";
import { useRobot } from "../state/robotStore";
import { fmt, mmToIn } from "./format";

function Gauge({ deg }: { deg: number }): JSX.Element {
  const a = (Math.max(0, Math.min(90, deg)) * Math.PI) / 180;
  const x = 34 + 28 * Math.cos(Math.PI - a);
  const y = 34 - 28 * Math.sin(Math.PI - a);
  return (
    <svg width="72" height="40" viewBox="0 0 72 40" aria-label={`pipe orientation ${deg.toFixed(0)} degrees`}>
      <path d="M6 34 A28 28 0 0 1 34 6" fill="none" stroke="#a9b4bd" strokeWidth="3" />
      <line x1="34" y1="34" x2={x} y2={y} stroke="#3ddc84" strokeWidth="3" strokeLinecap="round" />
      <text x="40" y="36" fill="#f2f5f7" fontSize="11">{deg.toFixed(0)}°</text>
    </svg>
  );
}

function Bars({ values, max, labels }: { values: number[]; max: number; labels: string[] }): JSX.Element {
  return (
    <div className="flex gap-1 items-end h-8" aria-label="motor currents">
      {values.map((v, i) => (
        <div key={labels[i]} className="flex flex-col items-center">
          <div className="w-3 bg-white/15 h-6 relative rounded-sm overflow-hidden">
            <div className={`absolute bottom-0 w-full ${v / max > 0.85 ? "bg-hud-bad" : "bg-hud-ok"}`} style={{ height: `${Math.min(100, (v / max) * 100)}%` }} />
          </div>
          <span className="text-[9px] text-hud-dim">{labels[i]}</span>
        </div>
      ))}
    </div>
  );
}

export function TelemetryHUD(): JSX.Element | null {
  const t = useRobot((s) => s.telemetry);
  const cfg = useRobot((s) => s.config);
  const units = useRobot((s) => s.settings.units);
  const show = useRobot((s) => s.settings.hud);
  const stale = useStale();
  if (!show) return null;
  const d = t?.est_diameter_mm;
  const dia = units === "imperial" && d !== undefined ? `${fmt(mmToIn(d), 2)} in` : fmt(d, 0, "mm");
  const grip = t?.grip_force_n ?? [];
  const low = cfg?.grip_low_n ?? 0;
  const bat = t?.battery;
  return (
    <div className={`absolute top-10 left-2 right-2 flex justify-between pointer-events-none hud-text text-sm ${stale ? "opacity-60" : ""}`}>
      <div className="bg-hud-bg rounded-lg p-2 space-y-1 min-w-[150px]">
        <div className="text-lg font-semibold">{fmt(t?.linear_mm_s, 0, "mm/s")}</div>
        <div>dist {fmt(t?.distance_m, 2, "m")} <span className="text-hud-dim">conf {fmt((t?.odom_confidence ?? 0) * 100, 0, "%")}</span></div>
        <div className="flex items-center gap-1"><Gauge deg={t?.pipe_orientation_deg ?? 0} /><span className="text-xs text-hud-dim">p {fmt(t?.imu?.pitch, 0)} r {fmt(t?.imu?.roll, 0)}</span></div>
        <div>Ø {dia}</div>
        <div className="flex gap-2">
          {grip.map((g, i) => (
            <span key={i} className={g > 0 && g < low ? "text-hud-warn" : g === 0 ? "text-hud-dim" : ""}>{["A", "B", "C"][i]} {fmt(g, 0)}N</span>
          ))}
        </div>
        {t?.axis_offset_mm !== undefined && t.axis_offset_mm > 2 && <div className="text-xs text-hud-dim">axis sag {fmt(t.axis_offset_mm, 1, "mm")}</div>}
      </div>
      <div className="bg-hud-bg rounded-lg p-2 space-y-1 text-right min-w-[130px]">
        <div className={bat?.supply === "BATTERY" && (bat?.pct ?? 100) < 25 ? "text-hud-bad" : ""}>
          {bat?.supply === "BATTERY" ? `🔋 ${bat?.pct ?? "--"}%` : "⚡ tether"} {fmt(bat?.v, 1, "V")}
        </div>
        <Bars values={t?.motor_current_a ?? [0, 0, 0, 0]} max={3} labels={["A", "B", "C", "E"]} />
        <div>{fmt(t?.env?.temp_c, 0, "°C")} · {fmt(t?.env?.humidity_pct, 0, "%RH")}</div>
        <div>{fmt(t?.env?.pressure_kpa, 1, "kPa")}</div>
        <div className="text-xs text-hud-dim">front {t?.tof_front_mm && t.tof_front_mm < 65535 ? fmt(t.tof_front_mm / 1000, 2, "m") : "--"}</div>
        <div className="text-xs text-hud-dim">{t?.state ?? "NO DATA"}{t?.rtt_ms ? ` · ${t.rtt_ms.toFixed(0)} ms` : ""}</div>
      </div>
    </div>
  );
}

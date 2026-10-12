import { useCallback, useEffect, useState } from "react";

import { api, type Health, type PreflightCheck } from "../api/client";
import { useRobot } from "../state/robotStore";

export function Connect({ onReady }: { onReady: () => void }): JSX.Element {
  const [health, setHealth] = useState<Health | null>(null);
  const [checks, setChecks] = useState<PreflightCheck[]>([]);
  const [err, setErr] = useState("");
  const [testing, setTesting] = useState(false);
  const operator = useRobot((s) => s.settings.operator);
  const setSettings = useRobot((s) => s.setSettings);
  const refresh = useCallback(async () => {
    try {
      setHealth(await api.health());
      setChecks((await api.preflight()).checks);
      setErr("");
    } catch (e) {
      setErr(`Cannot reach the reel access point (${String(e)}). Join the PIPEBOT Wi-Fi.`);
    }
  }, []);
  useEffect(() => {
    void refresh();
    const id = setInterval(refresh, 2000);
    return () => clearInterval(id);
  }, [refresh]);
  const gripTest = async (): Promise<void> => {
    setTesting(true);
    await api.expand("FORCE");
    setTimeout(() => {
      setTesting(false);
      void refresh();
    }, 4000);
  };
  const ok = checks.length > 0 && checks.every((c) => c.ok);
  return (
    <div className="p-4 max-w-xl mx-auto space-y-4 overflow-auto h-full">
      <h1 className="text-2xl font-bold">PIPEBOT-X</h1>
      {err && <div className="bg-red-800 rounded p-3">{err}</div>}
      <div className="grid grid-cols-2 gap-2 text-sm bg-white/5 rounded-lg p-3">
        <span>Robot size</span><b>{health?.size ?? "--"}</b>
        <span>Firmware link</span><b className={health?.link ? "text-hud-ok" : "text-hud-bad"}>{health?.link ? "connected" : "down"}</b>
        <span>Server version</span><b>{health?.version ?? "--"} (protocol v{health?.protocol_version ?? "?"})</b>
        <span>Tether length</span><b>{health?.tether_m ?? "--"} m</b>
        <span>Round trip</span><b>{health?.rtt_ms !== null && health?.rtt_ms !== undefined ? `${health.rtt_ms.toFixed(1)} ms` : "--"}</b>
      </div>
      <p className="text-xs text-hud-dim">
        Video and control run over the tether. Radio does not pass through steel pipe or borewell casing: the phone joins the
        Wi-Fi access point at the reel, never the robot.
      </p>
      <label className="block text-sm">Operator
        <input className="mt-1 w-full min-h-touch rounded bg-white/10 px-3" value={operator} onChange={(e) => setSettings({ operator: e.target.value })} />
      </label>
      <h2 className="font-semibold">Pre-flight checklist</h2>
      <ul className="space-y-1">
        {checks.map((c) => (
          <li key={c.name} className="flex justify-between bg-white/5 rounded px-3 py-2 text-sm">
            <span>{c.ok ? "✅" : "❌"} {c.name}</span><span className="text-hud-dim truncate max-w-[50%]">{c.detail}</span>
          </li>
        ))}
      </ul>
      <div className="flex gap-2">
        <button className="flex-1 min-h-touch rounded-lg bg-white/15" onClick={gripTest} disabled={testing}>{testing ? "Gripping..." : "Run grip test"}</button>
        <button className="flex-1 min-h-touch rounded-lg bg-emerald-600 font-semibold disabled:opacity-40" disabled={!health?.link} onClick={onReady}>
          {ok ? "Go live" : "Go live (checks failing)"}
        </button>
      </div>
    </div>
  );
}

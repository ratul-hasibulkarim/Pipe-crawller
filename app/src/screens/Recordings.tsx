import { useEffect, useRef, useState } from "react";

import { api, type RunEvent, type RunPoint, type RunSummary } from "../api/client";
import { StripChart } from "../components/StripChart";

export function Recordings(): JSX.Element {
  const [runs, setRuns] = useState<RunSummary[]>([]);
  const [sel, setSel] = useState<string | null>(null);
  const [series, setSeries] = useState<RunPoint[]>([]);
  const [events, setEvents] = useState<RunEvent[]>([]);
  const [cursor, setCursor] = useState(0);
  const [clip, setClip] = useState<[number, number]>([0, 10]);
  const video = useRef<HTMLVideoElement>(null);
  useEffect(() => {
    api.recordings().then(setRuns).catch(() => undefined);
  }, []);
  useEffect(() => {
    if (!sel) return;
    api.recordingSeries(sel).then(setSeries).catch(() => setSeries([]));
    api.recording(sel).then((r) => setEvents(r.events)).catch(() => setEvents([]));
  }, [sel]);
  const t = series.map((p) => p.t);
  const run = runs.find((r) => r.id === sel);
  const markers = events.filter((e) => e.type === "MARK" && run).map((e) => ({ t: e.t_wall - (run?.started ?? 0), label: e.label ?? "" }));
  const scrub = (v: number): void => {
    setCursor(v);
    if (video.current) video.current.currentTime = Math.max(0, v);
  };
  return (
    <div className="p-3 h-full overflow-auto space-y-3">
      <h2 className="font-semibold">Recordings</h2>
      <ul className="space-y-1">
        {runs.map((r) => (
          <li key={r.id}>
            <button className={`w-full min-h-touch text-left rounded px-3 ${sel === r.id ? "bg-white/20" : "bg-white/5"}`} onClick={() => setSel(r.id)}>
              {r.id} · {r.operator || "—"} · {(r.distance_m ?? 0).toFixed(2)} m · {r.events} events
            </button>
          </li>
        ))}
        {runs.length === 0 && <li className="text-hud-dim text-sm">No runs yet.</li>}
      </ul>
      {sel && (
        <div className="space-y-2">
          <video ref={video} src={api.videoUrl(sel)} controls className="w-full rounded bg-black" onTimeUpdate={(e) => setCursor(e.currentTarget.currentTime)} />
          <StripChart t={t} cursor={cursor} onScrub={scrub} markers={markers}
            series={[{ name: "distance m", color: "#3ddc84", values: series.map((p) => Number(p.distance_m)) },
              { name: "orientation °/10", color: "#7cc7ff", values: series.map((p) => Number(p.pipe_orientation_deg) / 10) }]} />
          <StripChart t={t} cursor={cursor} onScrub={scrub}
            series={[{ name: "grip A N", color: "#ffb020", values: series.map((p) => Number(p.grip0)) }, { name: "Ø mm/10", color: "#c38cff", values: series.map((p) => Number(p.est_diameter_mm) / 10) }]} />
          <div className="flex flex-wrap gap-2 items-center text-sm">
            <a className="min-h-touch inline-flex items-center rounded bg-white/15 px-3" href={api.csvUrl(sel)} download>Export CSV</a>
            <label>from <input type="number" className="w-16 bg-white/10 rounded px-1" value={clip[0]} onChange={(e) => setClip([Number(e.target.value), clip[1]])} /> s</label>
            <label>to <input type="number" className="w-16 bg-white/10 rounded px-1" value={clip[1]} onChange={(e) => setClip([clip[0], Number(e.target.value)])} /> s</label>
            <a className="min-h-touch inline-flex items-center rounded bg-white/15 px-3" href={api.clipUrl(sel, clip[0], clip[1])} download>Clip MP4</a>
            <a className="min-h-touch inline-flex items-center rounded bg-white/15 px-3" href={api.csvUrl(sel, clip[0], clip[1])} download>Clip CSV</a>
          </div>
        </div>
      )}
    </div>
  );
}

// Dependency-free SVG strip chart with event markers and an optional cursor (scrubbing).
export interface Series {
  name: string;
  color: string;
  values: number[];
}

export function StripChart({ t, series, markers = [], cursor, onScrub, height = 120, unit = "", minSpan = 1 }: {
  t: number[];
  series: Series[];
  markers?: { t: number; label: string }[];
  cursor?: number;
  onScrub?: (t: number) => void;
  height?: number;
  unit?: string;
  /** Smallest y range shown, so sensor noise is not magnified into fake events. */
  minSpan?: number;
}): JSX.Element {
  const w = 600;
  const t0 = t[0] ?? 0;
  const t1 = t[t.length - 1] ?? 1;
  const all = series.flatMap((s) => s.values).filter((v) => Number.isFinite(v));
  const rawLo = all.length ? Math.min(...all) : 0;
  const rawHi = all.length ? Math.max(...all) : 1;
  const pad = Math.max(0, minSpan - (rawHi - rawLo)) / 2;
  const lo = rawLo - pad;
  const hi = rawHi + pad;
  const span = hi - lo || 1;
  const x = (v: number) => ((v - t0) / (t1 - t0 || 1)) * w;
  const y = (v: number) => height - 6 - ((v - lo) / span) * (height - 12);
  return (
    <svg
      viewBox={`0 0 ${w} ${height}`}
      className="w-full bg-white/5 rounded touch-none"
      role="img"
      aria-label={series.map((s) => s.name).join(", ")}
      onPointerDown={(e) => onScrub && onScrub(t0 + ((e.clientX - e.currentTarget.getBoundingClientRect().left) / e.currentTarget.getBoundingClientRect().width) * (t1 - t0))}
      onPointerMove={(e) => e.buttons && onScrub && onScrub(t0 + ((e.clientX - e.currentTarget.getBoundingClientRect().left) / e.currentTarget.getBoundingClientRect().width) * (t1 - t0))}
    >
      {markers.map((m, i) => (
        <g key={i}>
          <line x1={x(m.t)} x2={x(m.t)} y1={0} y2={height} stroke="#ffb020" strokeDasharray="3 3" />
          <text x={x(m.t) + 2} y={10} fontSize="9" fill="#ffb020">{m.label}</text>
        </g>
      ))}
      {series.map((s) => (
        <polyline key={s.name} fill="none" stroke={s.color} strokeWidth="1.5" points={s.values.map((v, i) => `${x(t[i])},${y(v)}`).join(" ")} />
      ))}
      {cursor !== undefined && <line x1={x(cursor)} x2={x(cursor)} y1={0} y2={height} stroke="white" />}
      <text x={4} y={height - 2} fontSize="9" fill="#a9b4bd">{lo.toFixed(1)}{unit} – {hi.toFixed(1)}{unit}</text>
      <g>
        {series.map((s, i) => (
          <text key={s.name} x={w - 4} y={12 + i * 11} fontSize="9" fill={s.color} textAnchor="end">{s.name}</text>
        ))}
      </g>
    </svg>
  );
}

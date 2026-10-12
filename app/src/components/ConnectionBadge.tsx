import { useStale } from "../hooks/useTelemetry";
import { useRobot } from "../state/robotStore";

export function ConnectionBadge(): JSX.Element {
  const stale = useStale();
  const t = useRobot((s) => s.telemetry);
  const open = useRobot((s) => s.socketOpen);
  const text = !open ? "NO SERVER" : !t?.link ? "NO ROBOT LINK" : stale ? "STALE" : "LIVE";
  return (
    <span className={`inline-flex items-center gap-1 rounded-full px-2 py-0.5 text-xs font-semibold ${stale ? "bg-red-700" : "bg-emerald-700"}`} data-testid="conn">
      <span className={`w-2 h-2 rounded-full ${stale ? "bg-red-300" : "bg-emerald-300 animate-pulse"}`} />
      {text}
      {!stale && t?.age_ms !== null && t?.age_ms !== undefined ? ` ${Math.round(t.age_ms)} ms` : ""}
    </span>
  );
}

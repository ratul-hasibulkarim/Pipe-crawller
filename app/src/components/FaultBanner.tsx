import { useRobot } from "../state/robotStore";
import { FAULT_TEXT, SEVERE_FAULTS } from "./format";

export function FaultBanner(): JSX.Element | null {
  const faults = useRobot((s) => s.telemetry?.faults ?? []);
  const dismissed = useRobot((s) => s.dismissedFaults);
  const dismiss = useRobot((s) => s.dismissFaults);
  const visible = faults.filter((f) => !dismissed.includes(f));
  if (visible.length === 0) return null;
  const severe = visible.some((f) => SEVERE_FAULTS.includes(f));
  return (
    <div role="alert" className={`absolute top-0 left-0 right-0 z-30 flex items-start gap-2 px-3 py-2 text-sm ${severe ? "bg-red-700" : "bg-amber-600"} text-white`}>
      <ul className="flex-1 space-y-0.5">
        {visible.map((f) => (
          <li key={f}><b>{f}</b> {FAULT_TEXT[f] ?? ""}</li>
        ))}
      </ul>
      <button className="min-h-touch min-w-touch rounded bg-black/30 px-3" onClick={() => dismiss(visible)} aria-label="Dismiss faults">✕</button>
    </div>
  );
}

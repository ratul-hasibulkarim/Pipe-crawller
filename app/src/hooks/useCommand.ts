// Send a command and track it until the robot confirms it (pending -> confirmed / failed).
import { useCallback } from "react";

import { type CommandResult, isOk } from "../api/client";
import { useRobot } from "../state/robotStore";

export const CONFIRM_TIMEOUT_MS = 3000;

export function useCommand(key: string): {
  send: (value: number, fn: () => Promise<CommandResult>) => Promise<CommandResult>;
  pending: boolean;
  failed: string | undefined;
} {
  const setPending = useRobot((s) => s.setPending);
  const failPending = useRobot((s) => s.failPending);
  const p = useRobot((s) => s.pending[key]);
  const send = useCallback(
    async (value: number, fn: () => Promise<CommandResult>) => {
      setPending(key, value);
      const r = await fn();
      if (!isOk(r)) failPending(key, r.status);
      else
        setTimeout(() => {
          const still = useRobot.getState().pending[key];
          if (still && still.value === value && !still.failed) failPending(key, "NOT_CONFIRMED");
        }, CONFIRM_TIMEOUT_MS);
      return r;
    },
    [key, setPending, failPending],
  );
  return { send, pending: !!p && !p.failed, failed: p?.failed };
}

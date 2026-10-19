import { useEffect, useRef, useState } from 'react';
import { useCatch } from '../../reactHelper';

const ACK_ACCEPTED = 'ACCEPTED and EXECUTED'; // server/config/status.js vehicleCmdResults[0]

/**
 * Sends commands to one device: logs them in the raw stream, shows "<label> — sent" and, for
 * PX4 devices, "<label> — accepted" (or the rejection) when vehicle_command_ack arrives.
 * `send` resolves true when the server accepted the command, false otherwise.
 * `labelOf(type)` names a command in the toasts (useDeviceCapabilities' `label`).
 */
export const useDeviceCommands = (deviceId, position, log, labelOf) => {
  const [toast, setToast] = useState(null);
  const lastLabelRef = useRef(null);

  const notify = (message, severity = 'info') => setToast({ message, severity, key: Date.now() });

  const send = useCatch(async (type, attributes = {}, label = labelOf(type)) => {
    log('command', `→ ${type} ${JSON.stringify(attributes)}`);
    const response = await fetch('/api/commands/send', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ deviceId: parseInt(deviceId, 10), type, attributes }),
    });
    if (!response.ok) {
      throw Error(await response.text());
    }
    const result = await response.json();
    if (result?.state === 'error' || result?.state === 'warning') {
      notify(`${label} — ${result.msg}`, 'error');
      return false;
    }
    lastLabelRef.current = label;
    notify(`${label} — sent`);
    return true;
  });

  const ack = position?.attributes?.commandAck;
  const ackResult = position?.attributes?.resultCmdAck;
  useEffect(() => {
    // Only acks after a command sent from here: the cached one from before is not ours.
    if (!ack || !lastLabelRef.current) return;
    const accepted = ackResult === ACK_ACCEPTED;
    setToast({
      message: `${lastLabelRef.current} — ${accepted ? 'accepted' : ackResult}`,
      severity: accepted ? 'success' : 'warning',
      key: Date.now(),
    });
  }, [ack, ackResult]);

  return { send, toast, closeToast: () => setToast(null), notify };
};

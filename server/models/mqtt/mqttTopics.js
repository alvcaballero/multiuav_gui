// MQTT topic contract (see MQTT_PLAN.md). The drone side (mqtt_client fork) and the GCS
// agree on these shapes; keep this file as the only place that builds or parses them.
//
//   muav/<device>/pub/<slot>                 telemetry   drone -> GCS
//   muav/<device>/srv/<name>/request         call        GCS   -> drone
//   muav/<device>/srv/<name>/response        result      drone -> GCS
//   muav/<device>/status                     presence    drone -> GCS (retained, LWT)

export const ROOT = 'muav';

export const SUBSCRIPTIONS = [`${ROOT}/+/pub/+`, `${ROOT}/+/srv/+/response`, `${ROOT}/+/status`];

export const requestTopic = (device, name) => `${ROOT}/${device}/srv/${name}/request`;

/**
 * @param {string} topic
 * @returns {{ kind: 'pub', device: string, slot: string }
 *   | { kind: 'response', device: string, name: string }
 *   | { kind: 'status', device: string }
 *   | null}
 */
export function parseTopic(topic) {
  const parts = topic.split('/');
  if (parts[0] !== ROOT) return null;
  if (parts.length === 4 && parts[2] === 'pub') return { kind: 'pub', device: parts[1], slot: parts[3] };
  if (parts.length === 5 && parts[2] === 'srv' && parts[4] === 'response') {
    return { kind: 'response', device: parts[1], name: parts[3] };
  }
  if (parts.length === 3 && parts[2] === 'status') return { kind: 'status', device: parts[1] };
  return null;
}

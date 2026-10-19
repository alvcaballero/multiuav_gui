import { useEffect, useState } from 'react';
import { CommandType } from '../../common/util/commandTypes';

// Statuses that keep a mission/task on screen (server/config/status.js).
const MISSION_ALIVE = ['init', 'planning', 'running'];
const TASK_TERMINAL = ['complete', 'end', 'cancelled', 'error', 'skipped'];

/**
 * What the device can do, from the server's own catalogs:
 *   - commandTypes: GET /api/commands/types?deviceId (already filtered by the category's
 *     services/actions/publishers and resolved against command_schema — the same list the
 *     command dialog shows)
 *   - category:     GET /api/category/:category (the devices_msg block), for telemetry
 *     that is not a command (camera stream, obstacle_info)
 * `obstacle_info` can't be read off the position: the server fills it with 100s by default.
 */
export const useDeviceCapabilities = (device) => {
  const [commandTypes, setCommandTypes] = useState([]);
  const [category, setCategory] = useState(null);

  const deviceId = device?.id;
  const categoryName = device?.category;

  useEffect(() => {
    if (deviceId == null) return undefined;
    const controller = new AbortController();
    fetch(`/api/commands/types?${new URLSearchParams({ deviceId })}`, { signal: controller.signal })
      .then((response) => (response.ok ? response.json() : []))
      .then(setCommandTypes)
      .catch(() => {});
    return () => controller.abort();
  }, [deviceId]);

  useEffect(() => {
    if (!categoryName) return undefined;
    const controller = new AbortController();
    fetch(`/api/category/${encodeURIComponent(categoryName)}`, { signal: controller.signal })
      .then((response) => (response.ok ? response.json() : null))
      .then(setCategory)
      .catch(() => {});
    return () => controller.abort();
  }, [categoryName]);

  const commands = new Map(commandTypes.map((c) => [c.type, c]));
  const types = new Set(commands.keys());
  const subscribers = category?.subscribers ?? {};

  return {
    commandTypes,
    has: (type) => types.has(type),
    // The command as the server resolved it for this device (server/config/devices/command_schema.yaml):
    // label, severity, confirm, params…; undefined if the device doesn't have it.
    command: (type) => commands.get(type),
    label: (type) => commands.get(type)?.label ?? type,
    camera: Boolean(device?.camera?.length) || 'camera' in subscribers,
    gimbal: types.has(CommandType.GIMBAL),
    obstacle: 'obstacle_info' in subscribers,
    control: types.has(CommandType.MOVE_RELATIVE),
  };
};

/** The alive mission (and this device's open task in it), or null. */
export const selectDeviceMission = (deviceId) => (state) => {
  const id = Number(deviceId);
  for (const mission of Object.values(state.activeMissions.items)) {
    if (!MISSION_ALIVE.includes(mission.status)) continue;
    const task = Object.values(mission.tasks).find(
      (t) => t.deviceId === id && !TASK_TERMINAL.includes(t.status),
    );
    if (task) return { mission, task };
  }
  return null;
};

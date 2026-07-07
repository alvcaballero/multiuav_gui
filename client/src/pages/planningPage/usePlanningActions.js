import { useCallback } from 'react';
import YAML from 'yaml';

import { planningToLegacy } from '../../store';
import { useCatch } from '../../reactHelper';
import { validateUniqueDevices, transformLocationsForAPI } from '../../services/planningService';

export const usePlanningActions = ({
  SendTask,
  markers,
  myhostname,
  setNotification,
  setRequestPlanning,
}) => {
  const fetchDevices = useCallback(async () => {
    const response = await fetch('/api/devices');
    if (!response.ok) throw new Error(await response.text());
    return await response.json();
  }, []);

  const mapAssignmentToTaskDevice = useCallback((assignment, devices, markers) => {
    const device = devices.find((d) => d.id === Number(assignment.device.id));
    if (!device) return null;
    const base = markers.bases.find((b) => b.id === assignment.baseId);
    if (!base) return null;
    return {
      id: device.name,
      category: device.category,
      settings: { ...assignment.settings, base: Object.values(base), landing_mode: 2 },
    };
  }, []);

  const mapAssignmentsToDevices = useCallback(
    (assignments, devices, markers) =>
      assignments.flatMap((a) => {
        if (a.device.id === '') return [];
        const taskDevice = mapAssignmentToTaskDevice(a, devices, markers);
        return taskDevice ? [taskDevice] : [];
      }),
    [mapAssignmentToTaskDevice],
  );

  const buildTaskPayload = useCallback(
    (legacyPlanning, taskDevices) => ({
      id: legacyPlanning.id,
      name: legacyPlanning.name,
      case: legacyPlanning.objetivo.case,
      meteo: legacyPlanning.meteo,
      locations: transformLocationsForAPI(legacyPlanning.loc),
      devices: taskDevices,
    }),
    [],
  );

  const SendPlanning = useCatch(async () => {
    setNotification('');
    const legacyPlanning = planningToLegacy(SendTask, markers);
    const assignments = SendTask.assignments || [];

    const validation = validateUniqueDevices(assignments);
    if (!validation.isValid) {
      setNotification(validation.errorMsg);
      return null;
    }

    if (legacyPlanning.loc.length === 0) {
      setNotification('No elements to inspection');
      return null;
    }

    const devices = await fetchDevices();
    const taskDevices = mapAssignmentsToDevices(assignments, devices, markers);
    const taskPayload = buildTaskPayload(legacyPlanning, taskDevices);

    const response = await fetch(`http://${myhostname}:8004/mission_request`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(taskPayload),
    });
    if (!response.ok) throw new Error(await response.text());

    setRequestPlanning(0);
    return true;
  });

  const MissionTask = useCatch(async () => {
    const myTask = {
      id: SendTask.id,
      name: SendTask.name,
      objetivo: SendTask.objetivo.id,
      meteo: SendTask.meteo,
      locations: SendTask.loc.map((group) => ({
        name: group.name,
        items: group.items.map(({ latitude, longitude }) => ({ latitude, longitude })),
      })),
    };

    const response = await fetch('/api/missions/sendTask', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(myTask),
    });
    if (!response.ok) throw new Error(await response.text());
  });

  const SavePlanning = useCallback((value) => {
    const blob = new Blob([YAML.stringify(value)], { type: 'text/plain' });
    const url = URL.createObjectURL(blob);
    const link = document.createElement('a');
    link.download = `${value.name}.yaml`;
    link.href = url;
    link.click();
  }, []);

  const setDefaultPlanning = useCatch(async (value) => {
    const response = await fetch('api/planning/setDefault', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(value),
    });
    if (!response.ok) throw new Error(await response.text());
  });

  return { SendPlanning, MissionTask, SavePlanning, setDefaultPlanning };
};

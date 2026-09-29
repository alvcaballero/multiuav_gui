import { routesToTasks } from '../map/MissionConvert';

// Command an already-loaded mission by its id. The mission + tasks were created
// by commandLoadMission; the server commands each LOADED root task and the
// scheduler runs the rest. Returns { missionId, results: [{ deviceId, taskKey, state, msg }] }.
export const commandMission = async (missionId) => {
  if (missionId == null) {
    throw new Error('No hay misión cargada: cargá o seleccioná una misión antes de comandar');
  }

  const response = await fetch('/api/missions/command', {
    method: 'POST',
    body: JSON.stringify({ missionId }),
    headers: { 'Content-Type': 'application/json' },
  });
  if (response.ok) {
    const myresponse = await response.json();
    if (myresponse.state === 'error') {
      throw new Error(myresponse.msg);
    }
    return myresponse;
  }
  throw new Error('Error in commandMission: ' + (await response.text()));
};

// Load a mission: creates MissionPlan + Mission + every task, and loads the root
// tasks (no dependencies) on their devices. The editor's routes are sent as tasks[]
// so dependencies of a plan loaded from a task graph survive the round trip.
// Returns { missionId, planId, results: [{ deviceId, name, taskKey, state, msg }] }.
export const commandLoadMission = async (missions) => {
  const data = { name: missions.name, tasks: routesToTasks(missions.route) };

  const response = await fetch('/api/missions/load', {
    method: 'POST',
    body: JSON.stringify(data),
    headers: { 'Content-Type': 'application/json' },
  });
  if (response.ok) {
    const myresponse = await response.json();
    if (myresponse.state === 'error') {
      throw new Error(myresponse.msg);
    }
    return myresponse;
  }
  // An invalid task graph comes back as 400 { error, errors }: show the message, not raw JSON.
  const body = await response.text();
  let message = body;
  try {
    message = JSON.parse(body).error ?? body;
  } catch {
    // not JSON — show the body as is
  }
  throw new Error(message);
};

export const addDevice = async (device) => {
  const response = await fetch('/api/devices', {
    method: 'POST',
    body: JSON.stringify(device),
    headers: { 'Content-Type': 'application/json' },
  });
  if (response.ok) {
    const myresponse = await response.json();
    if (myresponse.state === 'error') {
      throw new Error(myresponse.msg);
    }
    return myresponse;
  }
  throw new Error(await response.text());
};

export const commandStopMission = async (devices) => {
  const listDeviceId = Object.values(devices).map((d) => d.id);
  const requests = listDeviceId.map((id) =>
    fetch('/api/commands/send', {
      method: 'POST',
      body: JSON.stringify({ deviceId: id, type: 'StopMission' }),
      headers: { 'Content-Type': 'application/json' },
    }),
  );
  await Promise.all(requests);
};

export const commandPauseMission = async (devices) => {
  const listDeviceId = Object.values(devices).map((d) => d.id);
  const requests = listDeviceId.map((id) =>
    fetch('/api/commands/send', {
      method: 'POST',
      body: JSON.stringify({ deviceId: id, type: 'Pausemission' }),
      headers: { 'Content-Type': 'application/json' },
    }),
  );
  await Promise.all(requests);
};

export const commandResumeMission = async (devices) => {
  const listDeviceId = Object.values(devices).map((d) => d.id);
  const requests = listDeviceId.map((id) =>
    fetch('/api/commands/send', {
      method: 'POST',
      body: JSON.stringify({ deviceId: id, type: 'ResumeMission' }),
      headers: { 'Content-Type': 'application/json' },
    }),
  );
  await Promise.all(requests);
};

export const connectRos = async () => {
  const response = await fetch('/api/rosConnect', {
    method: 'POST',
  });
  if (response.ok) {
    return await response.json();
  }
  throw new Error(await response.text());
};

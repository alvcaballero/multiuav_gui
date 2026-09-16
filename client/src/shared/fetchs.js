// Command an already-loaded mission by its id. The mission + routes were created
// by commandLoadMission; the server commands each LOADED route. Returns
// { missionId, results: [{ deviceId, state, msg }] }.
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

// Load a mission: creates MissionPlan + Mission + Routes (LOADED) and loads each
// drone. Returns { missionId, planId, results: [{ deviceId, name, state, msg }] }.
export const commandLoadMission = async (missions) => {
  const data = { route: missions.route };

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
  throw new Error(await response.text());
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

export const loadMission = async (mission) => {
  const response = await fetch('/api/mission', {
    method: 'POST',
    body: new URLSearchParams(`mission=${encodeURIComponent(mission)}`),
  });
  if (response.ok) {
    return await response.json();
  }
  throw new Error(await response.text());
};

export const commandMission = async (missions, devices) => {
  const listUAV = missions.route.map((element) => element.uav);
  const listDeviceId = listUAV.map((name) => {
    const item = Object.values(devices).find((d) => d.name === name);
    return item ? item.id : null;
  });

  const command2send = {
    deviceId: listDeviceId.length > 0 ? listDeviceId : -1,
    type: 'commandMission',
  };

  const response = await fetch('/api/commands/send', {
    method: 'POST',
    body: JSON.stringify(command2send),
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

export const commandLoadMission = async (missions) => {
  const data = { deviceId: -1, type: 'loadMission', attributes: missions.route };

  const response = await fetch('/api/commands/send', {
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

export const connectRos = async () => {
  const response = await fetch('/api/rosConnect', {
    method: 'POST',
  });
  if (response.ok) {
    return await response.json();
  }
  throw new Error(await response.text());
};

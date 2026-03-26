import { useDispatch } from 'react-redux';
import store, { errorsActions, sessionActions } from '../store';
import { useStore } from 'react-redux';

export const loadMission = async (mission) => {
  try {
    const response = await fetch('/api/mission', {
      method: 'POST',
      body: new URLSearchParams(`mission=${encodeURIComponent(mission)}`),
    });
    if (response.ok) {
      const data = await response.json();
      return data;
    } else {
      throw Error(await response.text());
    }
  } catch (error) {
    store.dispatch(errorsActions.push(error.message));
  }
};
export const commandMission = async () => {
  let missions = store.getState().mission;
  let devices = store.getState().devices.items;
  let commandDevice = null;
  let listUAV = missions['route'].map((element) => element.uav);
  console.log('devices', devices);
  let listDeviceId = listUAV.map((name) => {
    let item = Object.values(devices).find((mydevice) => mydevice.name == name);
    if (item) {
      return item.id;
    }
    return null;
  });
  console.log('command mission ');
  if (listDeviceId && listDeviceId.length > 0) {
    commandDevice = listDeviceId;
  }
  const command2send = {
    deviceId: commandDevice ? commandDevice : -1,
    type: 'commandMission',
  };

  try {
    const response = await fetch('/api/commands/send', {
      method: 'POST',
      body: JSON.stringify(command2send),
      headers: { 'Content-Type': 'application/json' },
    });
    if (response.ok) {
      const myresponse = await response.json();
      if (myresponse.state === 'connect') {
        console.log('success', myresponse.msg);
      }
      if (myresponse.state === 'error') {
        throw Error(await myresponse.msg);
      }
    } else {
      throw Error('Error in commandMission: ' + (await response.text()));
    }
  } catch (error) {
    store.dispatch(errorsActions.push(error.message));
  }
};

export const commandLoadMission = async (mission) => {
  let missions = store.getState().mission;
  console.log(missions);

  const data = { deviceId: -1, type: 'loadMission', attributes: missions['route'] };

  try {
    const response = await fetch('/api/commands/send', {
      method: 'POST',
      body: JSON.stringify(data),
      headers: { 'Content-Type': 'application/json' },
    });
    if (response.ok) {
      const myresponse = await response.json();
      if (myresponse.state === 'connect') {
        console.log('success', myresponse.msg);
      }
      if (myresponse.state === 'error') {
        throw Error(await myresponse.msg);
      }
    } else {
      throw Error(await response.text());
    }
  } catch (error) {
    store.dispatch(errorsActions.push(error.message));
  }
};

export const addDevice = async (device) => {
  console.log(device);
  console.log(device.name + '-' + device.category);
  try {
    const response = await fetch('/api/devices', {
      method: 'POST',
      body: JSON.stringify(device),
      headers: {
        'Content-Type': 'application/json',
      },
    });
    if (response.ok) {
      let myresponse = await response.json();
      if (myresponse.state === 'connect') {
        console.log('success', myresponse.msg);
      }
      if (myresponse.state === 'error') {
        throw Error(await myresponse.msg);
      }
    } else {
      throw Error(await response.text());
    }
  } catch (error) {
    throw Error(await response.text());
  }
};

export const connectRos = async () => {
  try {
    const response = await fetch('/api/rosConnect', {
      method: 'POST',
      body: new URLSearchParams(`rosState=${encodeURIComponent(rosState)}`),
    });
    if (response.ok) {
      let myresponse = await response.json();
      if (myresponse.state === 'connect') {
        store.dispatch(sessionActions.updateServerROS(true));
      }
      if (myresponse.state === 'disconnect') {
        store.dispatch(sessionActions.updateServerROS(false));
      }
      if (myresponse.state === 'error') {
        throw Error(await myresponse.msg);
      }
    } else {
      throw Error(await response.text());
    }
  } catch (error) {
    throw Error(await response.text());
  }
};

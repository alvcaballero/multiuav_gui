import React, { useState, useRef, useEffect } from 'react';
import { useDispatch, useSelector, connect } from 'react-redux';
import { missionActions } from '../store'; // here update device action with position of uav for update in map
import YAML from 'yaml';

export const RosContext = React.createContext();

var plan_mission = '';

export const RosControl = ({ children, notification }) => {
  const devices = useSelector((state) => state.devices.items);
  const missions = useSelector((state) => state.mission);
  const serverState = useSelector((state) => state.session.server.rosState);
  const dispatch = useDispatch();

  const [rosState, setrosState] = useState(false);
  const [confirmMission, setconfirmMission] = useState(false);
  const [textmission, settextmission] = useState('');

  useEffect(() => {
    setrosState(serverState);
  }, [serverState]);

  const serverConecRos = async (event) => {
    //event.preventDefault();
    try {
      const response = await fetch('/api/rosConnect', {
        method: 'POST',
        body: new URLSearchParams(`rosState=${encodeURIComponent(rosState)}`),
      });
      if (response.ok) {
        let myresponse = await response.json();
        if (myresponse.state === 'connect') {
          notification('success', myresponse.msg);
          setrosState(true);
        }
        if (myresponse.state === 'error') {
          setrosState(false);
          notification('danger', myresponse.msg);
        }
        if (myresponse.state === 'disconnect') {
          notification('danger', myresponse.msg);
          setrosState(false);
        }
        console.log(myresponse);
      } else {
        throw Error(await response.text());
      }
    } catch (error) {
      setrosState(false);
    }
  };
  const serverAddUAV = async (device) => {
    //event.preventDefault();
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
          notification('success', myresponse.msg);
        }
        if (myresponse.state === 'error') {
          notification('danger', myresponse.msg);
        }
        console.log(myresponse);
      } else {
        throw Error(await response.text());
      }
    } catch (error) {}
  };
  const serverRMUAV = async (uav_ns) => {
    console.log(uav_ns);
    try {
      const response = await fetch('/api/disconectdevice', {
        method: 'POST',
        body: JSON.stringify({ uav_ns: uav_ns }),
        headers: {
          'Content-Type': 'application/json',
        },
      });
      if (response.ok) {
        let myresponse = await response.json();
        if (myresponse.state === 'connect') {
          notification('success', myresponse.msg);
        }
        if (myresponse.state === 'error') {
          notification('danger', myresponse.msg);
        }
        console.log(myresponse);
      } else {
        throw Error(await response.text());
      }
    } catch (error) {
      console.log(error);
    }
  };

  const serverloadmission = async () => {
    try {
      const response = await fetch('/api/commands/send', {
        method: 'POST',
        body: JSON.stringify({ deviceId: -1, type: 'loadMission', attributes: missions['route'] }),
        headers: {
          'Content-Type': 'application/json',
        },
      });
      if (response.ok) {
        let myresponse = await response.json();
        if (myresponse.state === 'connect') {
          notification('success', myresponse.msg);
        }
        if (myresponse.state === 'error') {
          notification('danger', myresponse.msg);
        }
        console.log(myresponse);
      } else {
        throw Error(await response.text());
      }
    } catch (error) {
      console.log(error);
    }
  };
  const servercommandmission = async () => {
    // event.preventDefault();
    let commandDevice = null;
    let listUAV = missions['route'].map((element) => element.uav);

    let listDeviceId = listUAV.map(
      (name) => Object.values(devices).find((mydevice) => mydevice.name == name).id
    );
    console.log('command mission ');
    // console.log(listUAV);
    // console.log(listDeviceId);
    // find uav in mission loaded if not found send mission to all uav
    if (listDeviceId && listDeviceId.length > 0) {
      commandDevice = listDeviceId;
    }

    try {
      const response = await fetch('/api/commands/send', {
        method: 'POST',
        body: JSON.stringify({
          deviceId: commandDevice ? commandDevice : -1,
          type: 'commandMission',
        }),
        headers: {
          'Content-Type': 'application/json',
        },
      });
      if (response.ok) {
        const myresponse = await response.json();
        if (myresponse.state === 'connect') {
          notification('success', myresponse.msg);
        }
        if (myresponse.state === 'error') {
          notification('danger', myresponse.msg);
        }
        console.log(myresponse);
      } else {
        throw Error(await response.text());
      }
    } catch (error) {
      console.log(error);
    }
  };

  const openMision = (name_mission, text_mission) => {
    if (name_mission.endsWith('.yaml')) {
      plan_mission = YAML.parse(text_mission);
      dispatch(missionActions.updateMission({ ...plan_mission, name: name_mission.slice(0, -5) }));
    } else if (name_mission.endsWith('.waypoints')) {
      let mission_line = text_mission.split('\n');
      let mission_array = mission_line.map((x) => x.split('\t'));
      let mission_yaml = { uav_n: 1, uav_1: {} };
      let count_wp = 0;
      mission_array.forEach((element) => {
        if (element[3] == '16') {
          mission_yaml.uav_1['wp_' + count_wp] = [element[8], element[9], element[10]];
          count_wp = count_wp + 1;
        }
      });
      mission_yaml.uav_1['wp_n'] = count_wp;
      //console.log(mission_yaml)
      dispatch(missionActions.updateMission({ ...mission_yaml, name: name_mission.slice(0, -10) }));
    } else if (name_mission.endsWith('.kml')) {
      let xmlDocument = new DOMParser().parseFromString(text_mission, 'text/xml');
      let missionxml = xmlDocument.getElementsByTagName('coordinates');
      let mission_line2 = Object.values(missionxml).map((x) => {
        let mywp = x.textContent
          .replace('\t1', '')
          .replace(/(\r\n|\n|\r|\t)/gm, '')
          .split(' ');
        return mywp.map((point) => point.split(','));
      });
      let mission_yaml = { uav_n: mission_line2.length };
      let count_uav = 1;
      mission_line2.forEach((route) => {
        console.log(route);
        let count_wp = 0;
        mission_yaml['uav_' + count_uav] = {};
        route.forEach((element) => {
          //console.log(element);
          if (element.length == 3) {
            mission_yaml['uav_' + count_uav]['wp_' + count_wp] = [
              element[1],
              element[0],
              element[2],
            ];
            count_wp = count_wp + 1;
          }
        });
        mission_yaml['uav_' + count_uav]['wp_n'] = count_wp;
        count_uav = count_uav + 1;
      });

      console.log(mission_yaml);
      dispatch(missionActions.updateMission({ ...mission_yaml, name: name_mission.slice(0, -4) }));
    } else if (name_mission.endsWith('.plan')) {
      let jsondoc = JSON.parse(text_mission);
      let mission_yaml = { uav_n: 1, uav_1: {} };
      let count_wp = 0;
      jsondoc.mission.items.forEach((element) => {
        mission_yaml.uav_1['wp_' + count_wp] = [
          element.params[4],
          element.params[5],
          element.Altitude,
        ];
        count_wp = count_wp + 1;
      });
      mission_yaml.uav_1['wp_n'] = count_wp;
      dispatch(missionActions.updateMission({ ...mission_yaml, name: name_mission.slice(0, -5) }));
    } else {
      notification('danger', 'Formato de mission no compatible');
    }
  };

  const rosConnect = () => {
    serverConecRos();
  };

  async function connectAddUav(device) {
      serverAddUAV(device);
  }

  function loadMission() {
    serverloadmission();
  }

  function commandMission() {
    servercommandmission();
  }

  return (
    <div style={{ width: '100%', height: '100%' }}>
      <RosContext.Provider
        value={{
          rosConnect,
          rosState,
          openMision,
          connectAddUav,
          commandMission,
          loadMission,
          confirmMission,
          setconfirmMission,
        }}
      >
        {children}
      </RosContext.Provider>
    </div>
  );
};

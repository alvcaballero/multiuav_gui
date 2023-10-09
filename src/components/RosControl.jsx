import React, { useState, useRef, useEffect } from 'react';
import { useDispatch, useSelector, connect } from 'react-redux';
import { missionActions } from '../store'; // here update device action with position of uav for update in map
import YAML from 'yaml';

export const RosContext = React.createContext();

var plan_mission = '';
var mission_layers = [];
var mode_landing = 0;
var mode_yaw = 0;
//let uav_list = [];
let statusLog = [];

let ros = '';

export const RosControl = ({ children, notification }) => {
  const devices = useSelector((state) => state.devices.items);
  const missions = useSelector((state) => state.mission);
  const serverState = useSelector((state) => state.session.server);
  const dispatch = useDispatch();

  const [rosState, setrosState] = useState(false);
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
    //event.preventDefault();
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
    } catch (error) {}
  };

  const serverloadmission = async () => {
    try {
      const response = await fetch('/api/loadmission', {
        method: 'POST',
        body: JSON.stringify({ mission: missions['route'] }),
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
  const servercommandmission = async () => {
    //event.preventDefault();
    console.log('command mission ');
    try {
      const response = await fetch('/api/commandmission', {
        method: 'POST',
        body: JSON.stringify({}),
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

  const openMision = (name_mission, text_mission) => {
    if (name_mission.endsWith('.yaml')) {
      plan_mission = YAML.parse(text_mission);
      mode_landing = plan_mission['mode_landing'];
      mode_yaw = plan_mission['mode_yaw'];
      dispatch(
        missionActions.updateMission({
          name: name_mission.slice(0, -5),
          mission: plan_mission,
        })
      );
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
      dispatch(
        missionActions.updateMission({
          name: name_mission.slice(0, -10),
          mission: mission_yaml,
        })
      );
    } else if (name_mission.endsWith('.kml')) {
      let xmlDocument = new DOMParser().parseFromString(text_mission, 'text/xml');
      let mission_line = xmlDocument
        .querySelector('coordinates')
        .textContent.replace(/(\r\n|\n|\r|\t)/gm, '')
        .split(' ');
      let mission_array = mission_line.map((x) => x.split(','));
      let mission_yaml = { uav_n: 1, uav_1: {} };
      let count_wp = 0;
      mission_array.forEach((element) => {
        if (element.length == 3) {
          mission_yaml.uav_1['wp_' + count_wp] = [element[1], element[0], element[2]];
          count_wp = count_wp + 1;
        }
      });
      mission_yaml.uav_1['wp_n'] = count_wp;
      //console.log(mission_yaml)
      dispatch(
        missionActions.updateMission({
          name: name_mission.slice(0, -4),
          mission: mission_yaml,
        })
      );
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
      //console.log(mission_yaml)
      dispatch(
        missionActions.updateMission({
          name: name_mission.slice(0, -5),
          mission: mission_yaml,
        })
      );
    } else {
      notification('danger', 'Formato de mission no compatible');
    }
  };

  function updateInfoCell(uav_ns, info) {
    var showData = document.getElementById(uav_ns).cells;
    showData[5].innerHTML = info;
  }

  function changeReady(uav_ns) {
    var button = document.getElementById('Ready' + uav_ns);
    if (button.innerHTML === 'Ready') {
      button.innerHTML = 'Not Ready';
    } else {
      button.innerHTML = 'Ready';
      var info = 'Mission requested';
      updateInfoCell(uav_ns, info);
    }
  }

  const rosConnect = () => {
    serverConecRos();
  };

  async function connectAddUav(device) {
    if (rosState) {
      serverAddUAV(device);
    } else {
      alert('\nRos no está conectado.\n\n Por favor conéctelo primero.');
    }
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
        }}
      >
        {children}
      </RosContext.Provider>
    </div>
  );
};

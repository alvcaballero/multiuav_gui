import YAML from 'yaml';
import store, { missionActions } from '../store'; // here update device action with position of uav for update in map
import { useDispatch } from 'react-redux';

var mission_home = [];

export const GetMissionHome = () => {
  return mission_home;
};

export const FiletoMission = (item) => {
  let plan_mission = {};
  if (item.name.endsWith('.yaml') || item.name.endsWith('.yml')) {
    plan_mission = YAML.parse(item.data);
  } else if (item.name.endsWith('.waypoints')) {
    plan_mission = filewaypoint2mission(item.data);
  } else if (item.name.endsWith('.kml')) {
    plan_mission = fileKml2mission(item.data);
  } else if (item.name.endsWith('.plan')) {
    plan_mission = filePlan2mission(item.data);
  } else {
    alert('Formato de archivo no soportado');
    return;
  }
  store.dispatch(missionActions.updateMission({ ...plan_mission, name: item.name }));
  return null;
};
const filePlan2mission = (data) => {
  let jsondoc = JSON.parse(data);
  let mission_yaml = { uav_n: 1, uav_1: {} };
  let count_wp = 0;
  jsondoc.mission.items.forEach((element) => {
    mission_yaml.uav_1['wp_' + count_wp] = [element.params[4], element.params[5], element.Altitude];
    count_wp = count_wp + 1;
  });
  mission_yaml.uav_1['wp_n'] = count_wp;
};
const fileKml2mission = (data) => {
  let xmlDocument = new DOMParser().parseFromString(data, 'text/xml');
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
        mission_yaml['uav_' + count_uav]['wp_' + count_wp] = [element[1], element[0], element[2]];
        count_wp = count_wp + 1;
      }
    });
    mission_yaml['uav_' + count_uav]['wp_n'] = count_wp;
    count_uav = count_uav + 1;
  });
  return mission_yaml;
};
const filewaypoint2mission = (data) => {
  let mission_line = data.split('\n');
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
  return mission_yaml;
};
export const RuteConvert = (route) => {
  const rt = [];
  let latlongError = false;

  // Lista de atributos posibles
  const ATTRS = ['mode_landing', 'mode_yaw', 'mode_gimbal', 'mode_trace', 'idle_vel', 'max_vel'];

  for (let uavN = 0; uavN < route.length; uavN++) {
    const src = route[uavN];
    const dst = {
      id: uavN,
      name: src.name,
      wp: [],
      attributes: {},
    };
    if ('uav' in src) dst.uav = src.uav;

    // Waypoints
    if (Array.isArray(src.wp)) {
      for (let wpN = 0; wpN < src.wp.length; wpN++) {
        const wpSrc = src.wp[wpN];
        const wpDst = {
          pos: wpSrc.pos,
          yaw: wpSrc.yaw,
          gimbal: wpSrc.gimbal,
        };
        // Validación de lat/lon
        if (Math.abs(Number(wpSrc.pos[0])) > 90 || Math.abs(Number(wpSrc.pos[1])) > 90) {
          console.log(`Error en latitud o longitud en UAV ${uavN + 1} WP ${wpN + 1} pos ${wpSrc.pos}`);
          latlongError = true;
        }
        if ('speed' in wpSrc) wpDst.speed = wpSrc.speed;
        if ('action' in wpSrc) wpDst.action = wpSrc.action;
        dst.wp.push(wpDst);
      }
    }

    // Atributos
    const attrSrc = src.attributes || src;
    ATTRS.forEach((key) => {
      if (key in attrSrc) {
        dst.attributes[key] = attrSrc[key];
        if (key === 'mode_landing' && src.attributes) {
          console.log('have modelanding' + uavN);
        }
      }
    });

    rt.push(dst);
  }

  if (latlongError) {
    alert('Error en coordenadas latitud y longitud valores entre -90 y 90');
    return [];
  }
  return rt;
};

export const RuteConvertlegacy = (mission) => {
  const rt = [];
  const ATTRS = ['mode_landing', 'mode_yaw', 'idle_vel', 'max_vel'];

  for (let uavN = 1; uavN <= mission['uav_n']; uavN++) {
    const uavKey = 'uav_' + uavN;
    if (!mission.hasOwnProperty(uavKey)) continue;

    const uavNx = uavN - 1;
    const uavData = mission[uavKey];
    const wpCount = uavData['wp_n'] || 0;

    const uavObj = {
      id: uavNx,
      uav: uavKey,
      name: uavKey,
      wp: [],
      attributes: {},
    };

    // Waypoints
    for (let wpN = 0; wpN < wpCount; wpN++) {
      const wp = uavData['wp_' + wpN];
      if (!wp) continue;
      const pos = wp.length === 3 ? wp : wp.slice(0, -1);
      const yaw = wp.length === 3 ? 0 : wp[3];
      uavObj.wp.push({ pos, yaw });
    }

    // Global attributes
    ATTRS.forEach((attr) => {
      if (mission.hasOwnProperty(attr) && attr !== 'max_vel') {
        uavObj.attributes[attr] = mission[attr];
      }
    });

    // UAV-specific attributes
    const attrSrc = uavData.attributes || uavData;
    ATTRS.forEach((attr) => {
      if (attr in attrSrc) {
        uavObj.attributes[attr] = attrSrc[attr];
        if (attr === 'mode_landing' && uavData.attributes) {
          console.log('have modelanding' + uavN);
        }
      }
    });

    rt[uavNx] = uavObj;
  }

  console.log('-----------   legacy   --------');
  console.log(rt);
  return rt;
};

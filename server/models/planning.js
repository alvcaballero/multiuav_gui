import { readDataFile, writeDataFile, getRandomInt } from '../common/utils.js';
import { missionsConfigData } from '../config/config.js';

const configPlanning = readDataFile('../config/planning/config.yaml');
var initPlanning = readDataFile(missionsConfigData);
import { planningServer, planningHost } from '../config/config.js';
import { MissionController } from '../controllers/mission.js';

const requestPlanning = {};

export class planningModel {
  static getTypes() {
    console.log('Mission types map');
    return configPlanning.missionTypes.map((mission) => ({
      id: mission.id,
      name: mission.name,
      type: mission.type,
      case: mission.case,
      description: mission.description,
    }));
  }
  static getParam(type) {
    console.log('Mission params' + type);
    return configPlanning.missionTypes[type]['data'];
  }
  static getBasesSettings() {
    console.log('Get bases settingd');
    return initPlanning.bases;
  }
  static getMissionTypes() {
    console.log('Mission types all');
    return configPlanning.missionTypes;
  }
  static getDefault() {
    console.log('Get default planning with markers');
    return { ...initPlanning, id: getRandomInt(100000000) };
  }
  static getPlanning() {
    console.log('Get default planing ');
    return {
      name: initPlanning.name,
      objetivo: initPlanning.objetivo,
      loc: initPlanning.loc,
      meteo: initPlanning.meteo,
      bases: initPlanning.bases,
      settings: initPlanning.settings,
    };
  }
  static setDefault(value) {
    console.log('Set default mission');
    console.log(value);
    initPlanning = value;
    let response = writeDataFile(missionsConfigData, value);
    return { result: response };
    // modify the mission init
  }

  static setMarkers(value) {
    // modify de markers
    console.log('Set markers');
    auxinitPlanning = { ...initPlanning, markersbase: value.markersbase, elements: value.elements };
    initPlanning = auxinitPlanning;
    let response = writeDataFile(missionsConfigData, auxinitPlanning);
    return { result: response };
  }
  static getMarkers() {
    console.log('get markers');
    return { markersbase: initPlanning.markersbase, elements: initPlanning.elements };
  }
  static getMarkersTypes() {
    console.log('get markers type');
    return configPlanning.markers;
  }
  static getBases() {
    console.log('get Bases');
    return initPlanning.markersbase;
  }
  static getElements() {
    console.log('get elements');
    return initPlanning.elements;
  }

  static async PlanningRequest({ id, myTask }) {
    let response2;
    const response1 = await fetch(`${planningHost}/mission_request`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(myTask),
    });
    if (response1.ok) {
      response2 = await response1.json();
      console.log(response2);

      requestPlanning[id] = {};
      requestPlanning[id]['count'] = 0;
      requestPlanning[id]['interval'] = setInterval(() => {
        console.log('response Planning' + id);
        this.fetchPlanning(id);
      }, 10000);
    } else {
      throw Error(await response.text());
    }
    return response2;
  }

  static async fetchPlanning(mission_id) {
    console.log('Fetch planning ' + mission_id);
    let planningRoute = null;
    const response = await fetch(`${planningHost}/get_plan?IDs=${mission_id}`);
    if (response.ok) {
      const data = await response.json();
      console.log(data);
      console.log('response to check planning');
      if (data.results && Object.keys(data.results) > 0) {
        if (data.results.hasOwnProperty(mission_id) && data.results[mission_id].hasOwnProperty('route')) {
          console.log('get response planning');
          planningRoute = data.results[mission_id];
          requestPlanning[mission_id]['count'] = 10;
        }
      }
    } else {
      console.log('!! --- error en el fetch planning --- !!');
      throw Error(await response.text());
    }
    requestPlanning[mission_id]['count'] = requestPlanning[mission_id]['count'] + 1;
    if (requestPlanning[mission_id]['count'] > 3) {
      clearInterval(requestPlanning[mission_id]['interval']);
      MissionController.initMission(mission_id, planningRoute); // error
    }
  }

  static localPlanning({ mission_id, objectivo, loc, meteo }) {
    let uav = 'uav_15';

    let home = [37.134092, -6.472401, 50];
    let reqRoute = Object.values(loc);
    let mission = {
      version: '3',
      route: [{ name: 'datetime', uav: uav, wp: [] }],
      status: 'OK',
    };
    let response = { uav: uav, points: [], status: 'OK' };
    response.points.push(home);
    for (let i = 0; i < reqRoute.length; i = i + 1) {
      let wp_len = reqRoute[i].length;
      for (let j = 0; j < wp_len; j = j + 1) {
        if (j == 0) {
          response.points.push([reqRoute[i][j]['lat'], reqRoute[i][j]['lon'], 50]);
        }
        response.points.push([reqRoute[i][j]['lat'], reqRoute[i][j]['lon'], 30]);
        if (j == +wp_len + -1) {
          response.points.push([reqRoute[i][j]['lat'], reqRoute[i][j]['lon'], 50]);
        }
      }
    }

    response.points.push(home);
    mission.route[0]['wp'] = response.points.map((element) => {
      return { pos: element };
    });
    return mission;
  }
}

import { readDataFile, writeDataFile, getRandomInt } from '../common/utils.js';
import { missionsConfigData } from '../config/config.js';

const configPlanning = readDataFile('../config/planning/config.yaml');
var initPlanning = readDataFile(missionsConfigData);
import { planningServer, planningHost } from '../config/config.js';
import { missionController } from '../controllers/mission.js';
import logger from '../common/logger.js';

const requestPlanning = {};

const firstplanning = {
  id: 1234,
  name: 'no mission',
  objetivo: { id: 1 },
  loc: [],
  meteo: [],
  settings: {},
  markersbase: [],
  elements: [],
  assignments: [],
};

if (Object.keys(initPlanning).length === 0) {
  logger.warn('init planning is empty, using default');
  initPlanning = firstplanning;
}

export class planningModel {
  static getTypes() {
    logger.debug('Mission types map');
    return configPlanning.missionTypes.map((mission) => ({
      id: mission.id,
      name: mission.name,
      type: mission.type,
      case: mission.case,
      description: mission.description,
    }));
  }
  static getParam(type) {
    logger.debug(`Mission params type=${type}`);
    return configPlanning.missionTypes[type]['data'];
  }
  static getMissionTypes() {
    logger.debug('Mission types all');
    return configPlanning.missionTypes;
  }
  static getDefault() {
    //console.log('Get default planning with markers');
    return { ...initPlanning, id: getRandomInt(100000000) };
  }
  static getPlanning() {
    //console.log('Get default planing ');
    return {
      name: initPlanning.name,
      objetivo: initPlanning.objetivo,
      loc: initPlanning.loc,
      meteo: initPlanning.meteo,
      bases: initPlanning.bases,
      settings: initPlanning.settings,
    };
  }
  static getBasesSettings() {
    logger.debug('Get bases settings (from assignments)');
    const assignments = initPlanning.assignments || [];
    const markersbase = initPlanning.markersbase || [];
    const basesMap = new Map(markersbase.map((b) => [b.id, b]));
    return assignments.map((a) => ({
      devices: a.device,
      settings: a.settings,
      base: basesMap.get(a.baseId) || null,
    }));
  }

  static getBases() {
    logger.debug('get bases (landing sites)');
    return initPlanning.markersbase;
  }

  static setDefault(value) {
    //console.log('Set default mission');
    //console.log(value);
    initPlanning = value;
    let response = writeDataFile(missionsConfigData, value);
    return { result: response };
    // modify the mission init
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
      logger.debug(`PlanningRequest response: ${JSON.stringify(response2)}`);

      requestPlanning[id] = {};
      requestPlanning[id]['count'] = 0;
      requestPlanning[id]['interval'] = setInterval(() => {
        logger.debug(`polling planning for id ${id}`);
        this.fetchPlanning(id);
      }, 10000);
    } else {
      throw Error(await response.text());
    }
    return response2;
  }

  static async fetchPlanning(mission_id) {
    logger.info(`Fetch planning mission_id=${mission_id}`);
    let planningRoute = null;
    const response = await fetch(`${planningHost}/get_plan?IDs=${mission_id}`);
    if (response.ok) {
      const data = await response.json();
      logger.debug(`fetchPlanning response: ${JSON.stringify(data)}`);
      if (data.results && Object.keys(data.results) > 0) {
        if (data.results.hasOwnProperty(mission_id) && data.results[mission_id].hasOwnProperty('route')) {
          logger.info(`got planning response for mission ${mission_id}`);
          planningRoute = data.results[mission_id];
          requestPlanning[mission_id]['count'] = 10;
        }
      }
    } else {
      logger.error(`error fetching planning for mission ${mission_id}`);
      throw Error(await response.text());
    }
    requestPlanning[mission_id]['count'] = requestPlanning[mission_id]['count'] + 1;
    if (requestPlanning[mission_id]['count'] > 3) {
      clearInterval(requestPlanning[mission_id]['interval']);
      missionController.initMission(mission_id, planningRoute); // error
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

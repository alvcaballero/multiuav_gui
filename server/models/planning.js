import { readDataFile, writeDataFile, getRandomInt } from '../common/utils.js';
import { missionsConfigData, planningHost } from '../config/config.js';
import { markersSnapshotModel } from './markers/snapshot.js';
import { assignmentsModel } from './markers/assignments.js';
import { basesModel } from './markers/bases.js';

const configPlanning = readDataFile('../config/planning/config.yaml');
var initPlanning = readDataFile(missionsConfigData);
import { missionController } from '../controllers/mission.js';
import { missionLogger as logger } from '../common/logger.js';

const requestPlanning = {};

// Planner polling cadence and timeout. We poll /get_plan every PLANNING_POLL_MS and
// give up (mission → ERROR) after PLANNING_MAX_TICKS attempts. A MIP plan with
// several UAVs + collision resolution can take a while, so the budget is 2 minutes.
const PLANNING_POLL_MS = 10000;
const PLANNING_TIMEOUT_S = 120;
const PLANNING_MAX_TICKS = PLANNING_TIMEOUT_S / (PLANNING_POLL_MS / 1000); // 12 ticks

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
  static getParam(id) {
    logger.debug(`Mission params id=${id}`);
    // missionTypes ids are NOT guaranteed to match their array position (e.g.
    // id:3 sits at index 3 but has case:1, same as the id:1 entry) — look up
    // by real id instead of indexing the array by position.
    return configPlanning.missionTypes.find((m) => m.id === Number(id))?.data;
  }
  static getMissionTypes() {
    logger.debug('Mission types all');
    return configPlanning.missionTypes;
  }
  // markersbase/elements/assignments now live in SQL (ElementGroup/ElementItem/
  // Base/Assignment) — this rebuilds the same legacy blob the client/planner
  // expect, sourcing that portion from SQL and the rest (objetivo/loc/meteo/id)
  // from the residual YAML.
  static async getDefault() {
    const { markersbase, elements } = await markersSnapshotModel.getMarkers();
    const assignments = await markersSnapshotModel.getAssignments();
    return {
      ...initPlanning,
      id: getRandomInt(100000000),
      markersbase,
      elements,
      assignments,
    };
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
  static async getBasesSettings() {
    logger.debug('Get bases settings (from assignments)');
    return await assignmentsModel.getBasesSettings();
  }

  static async getBases() {
    logger.debug('get bases (landing sites)');
    return await basesModel.getAll();
  }

  // `value` is the full legacy-shaped payload the client sends (objetivo/loc/
  // meteo/id plus markersbase/elements/assignments). The SQL portion is
  // upserted into the tables; the rest keeps being persisted to the residual
  // YAML, same as before.
  static async setDefault(value) {
    const { markersbase, elements, assignments, ...residual } = value;
    await markersSnapshotModel.setMarkers({ markersbase, elements, assignments });
    initPlanning = { ...initPlanning, ...residual };
    let response = await writeDataFile(missionsConfigData, initPlanning);
    return { result: response };
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
        // fetchPlanning runs detached in the interval: an unhandled rejection here
        // would both crash the tick AND leave the interval running forever. Swallow
        // it so a transient planner error just skips this tick and retries next one.
        this.fetchPlanning(id).catch((err) => {
          logger.error(`fetchPlanning(${id}) failed this tick: ${err.message}`);
        });
      }, PLANNING_POLL_MS);
    } else {
      throw Error(await response1.text());
    }
    return response2;
  }

  // Stops the polling interval for a mission and removes its bookkeeping entry.
  // Idempotent: safe to call even if a concurrent tick already resolved it.
  static _stopPolling(mission_id) {
    const entry = requestPlanning[mission_id];
    if (!entry) return;
    clearInterval(entry['interval']);
    delete requestPlanning[mission_id];
  }

  static async fetchPlanning(mission_id) {
    logger.info(`Fetch planning mission_id=${mission_id}`);
    if (!requestPlanning[mission_id]) {
      logger.debug(`fetchPlanning: mission ${mission_id} already resolved by a previous tick, skipping`);
      return;
    }
    // The planner keys /get_plan results by the STRING form of the id (json[str(id)]
    // in api.py), so look them up with String(mission_id) instead of the raw number.
    const key = String(mission_id);
    let planningRoute = null;
    const response = await fetch(`${planningHost}/get_plan?IDs=${mission_id}`);
    if (response.ok) {
      const data = await response.json();
      logger.debug(`fetchPlanning response: ${JSON.stringify(data)}`);
      if (data.results?.[key]?.hasOwnProperty('route')) {
        logger.info(`got planning response for mission ${mission_id}`);
        planningRoute = data.results[key];
      }
    } else {
      // Do NOT throw: this runs in a setInterval tick. Log and treat it as a failed
      // attempt so a persistently failing planner converges to the timeout→ERROR
      // path below instead of polling forever.
      logger.error(`error fetching planning for mission ${mission_id}: HTTP ${response.status}`);
    }

    // A concurrent tick may have resolved this mission while we awaited the fetch.
    if (!requestPlanning[mission_id]) {
      logger.debug(`fetchPlanning: mission ${mission_id} resolved by a concurrent tick while awaiting, skipping`);
      return;
    }

    // The planner owns NO mission-editing logic: whatever the outcome, it just hands
    // off to initMission, which decides the mission's fate. initMission is designed to
    // never reject (it drives the mission to ERROR internally on any failure), so no
    // .catch is needed here.

    // Plan is ready → stop polling and hand it off.
    if (planningRoute != null) {
      this._stopPolling(mission_id);
      missionController.initMission(mission_id, planningRoute);
      return;
    }

    // No plan yet → count this attempt; once we run out of time, hand off a null plan
    // with timedOut:true so initMission fails the mission with the right reason.
    requestPlanning[mission_id]['count'] += 1;
    if (requestPlanning[mission_id]['count'] >= PLANNING_MAX_TICKS) {
      this._stopPolling(mission_id);
      logger.warn(`fetchPlanning: timeout (${PLANNING_TIMEOUT_S}s) waiting for plan mission=${mission_id}`);
      missionController.initMission(mission_id, null, { timedOut: true });
    }
  }

  static localPlanning({ mission_id: _mission_id, objectivo: _objectivo, loc, meteo: _meteo }) {
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

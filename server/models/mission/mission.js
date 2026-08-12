import { devicesController } from '../../controllers/devices.js';
import { positionsController } from '../../controllers/positions.js';
import { missionSMModel } from './missionSM.js';
import { ExtAppController } from '../../controllers/ExtApp.js';
import { planningController } from '../../controllers/planning.js';
import { filesController } from '../../controllers/files.js';
import { eventsController } from '../../controllers/events.js';
import { commandsController } from '../../controllers/commands.js';
import { readDataFile, sleep, withRetry } from '../../common/utils.js';
import sequelize from '../../common/sequelize.js';
import { Op } from 'sequelize';
import { eventBus, EVENTS } from '../../common/eventBus.js';
import { convertMissionXYZToLatLong, convertMissionBriefingToXYZ } from './coordinateConverter.js';
import { missionLogger as logger } from '../../common/logger.js';
import { MISSION_STATUS, ROUTE_STATUS, MISSION_ALIVE_STATUS } from '../../config/status.js';

/**
 * @typedef Mission
 * @property {integer} id
 * @property {string} status - see MISSION_STATUS in config/status.js
 * @property {string} initTime
 * @property {string} FinishTime
 * @property {Array<number>} uav
 * @property {object} task - task planning
 * @property {object} mission - mission planning
 * @property {Array<object>} results
 */

export class missionModel {
  static async getMissionValue(id, all = false) {
    if (id) {
      return await sequelize.models.Mission.findOne({ where: { id: id } });
    }
    if (all) {
      return await sequelize.models.Mission.findAll();
    }
    const since = new Date(Date.now() - 24 * 60 * 60 * 1000);
    return await sequelize.models.Mission.findAll({
      where: {
        status: { [Op.in]: [MISSION_STATUS.INIT, MISSION_STATUS.PLANNING, MISSION_STATUS.RUNNING] },
        initTime: { [Op.gte]: since },
      },
    });
  }

  // Resolve a mission by the external system's task id (ExtApp dedup / addressing).
  // Coerce to Number so a string-typed id ("118") still matches the INTEGER column.
  static async getMissionByExternalId(externalId) {
    if (externalId == null) return null;
    const numId = Number(externalId);
    if (Number.isNaN(numId)) return null;
    return await sequelize.models.Mission.findOne({ where: { externalId: numId } });
  }

  // Translate an internal mission PK into the external system's task id for ExtApp
  // callbacks. Falls back to the PK itself when the mission has no external origin
  // (e.g. legacy rows or manual missions that somehow reach an ExtApp callback).
  static async _resolveExternalId(missionId) {
    const myMission = await this.getMissionValue(missionId);
    return myMission?.externalId ?? missionId;
  }

  static async getRoutes({ id, deviceId, missionId, status }) {
    if (deviceId && missionId) {
      return await sequelize.models.MissionRoute.findOne({ where: { deviceId: deviceId, missionId: missionId } });
    }
    if (id) {
      return await sequelize.models.MissionRoute.findOne({ where: { id: id } });
    }
    if (deviceId && status) {
      return await sequelize.models.MissionRoute.findOne({ where: { deviceId: deviceId, status: status } });
    }
    if (missionId) {
      return await sequelize.models.MissionRoute.findAll({ where: { missionId: missionId } });
    }
    return await sequelize.models.MissionRoute.findAll();
  }

  // All routes currently in a trackable state (COMMANDED/RUNNING). Used once at
  // server startup to bootstrap missionWpTracking's in-memory registry — routes
  // already in flight when the process restarts need to be picked up without
  // waiting for their next DB write.
  static async getActiveRoutes() {
    return await sequelize.models.MissionRoute.findAll({
      where: { status: { [Op.in]: [ROUTE_STATUS.COMMANDED, ROUTE_STATUS.RUNNING] } },
    });
  }

  // Single emission point for Mission/MissionRoute row changes — called from
  // create*/edit* right after persisting, so every write path (manual, automatic,
  // state-machine-driven) notifies clients uniformly and payload shape can't drift.
  static _emitMissionUpdated(mission) {
    eventBus.emitSafe(EVENTS.MISSION_UPDATED, mission.get({ plain: true }));
  }

  // `signals` (anomalies/wpEstimate/confidence) are transient wpTracking diagnostics —
  // never persisted, just merged into the outbound payload when provided.
  static _emitRouteUpdated(route, signals = {}) {
    eventBus.emitSafe(EVENTS.ROUTE_UPDATED, { ...route.get({ plain: true }), ...signals });
  }

  // Same ROUTE_UPDATED shape as _emitRouteUpdated, but for callers (missionWpTracking's
  // in-memory registry) that already hold a plain cached route object instead of a
  // Sequelize instance — lets the "signals only, nothing persisted" path emit without
  // any DB round-trip.
  static emitRouteSignals(route, signals = {}) {
    eventBus.emitSafe(EVENTS.ROUTE_UPDATED, { ...route, ...signals });
  }

  static async broadcastMission(mission) {
    if (mission == null || !mission?.hasOwnProperty('route') || mission?.route?.length == 0) {
      return { success: false };
    }
    eventBus.emitSafe(EVENTS.MISSION_PLAN_SHOWN, { ...mission, name: mission.name ? mission.name : 'name' });
    return { success: true };
  }

  static async createMission({
    externalId = null,
    name,
    planId = null,
    trigger = 'automatic',
    uav = [],
    status = MISSION_STATUS.INIT,
    initTime = new Date(),
    endTime = null,
    task = {},
    mission = {},
    results = [],
    errorMessage = null,
  }) {
    if (name == null) name = `automatic_${initTime.getTime()}`;

    // externalId is only provided by the automatic (ExtApp) flow. Dedup for
    // idempotency, but ONLY against a mission that is still alive: a re-sent task
    // whose previous attempt is terminal (cancelled/error/finished) must yield a
    // FRESH mission so it re-plans. externalId has no DB UNIQUE constraint precisely
    // to allow these successive attempts. The PK `id` ALWAYS autoincrements — the
    // external id lives in its own column and never touches the PK. Manual flow has
    // no externalId (null), so it skips dedup and always creates a fresh row.
    if (externalId != null) {
      const alive = await sequelize.models.Mission.findOne({
        where: { externalId, status: { [Op.in]: MISSION_ALIVE_STATUS } },
      });
      if (alive) {
        logger.warn(`createMission: externalId=${externalId} already active as mission ${alive.id}, returning it`);
        return alive;
      }
    }

    // Single build of the row: externalId defaults to null, so including it always is
    // identical to omitting it for the manual flow — no need for two separate creates.
    const myMission = await sequelize.models.Mission.create({
      externalId,
      name,
      planId,
      trigger,
      uav,
      status,
      initTime,
      endTime,
      task,
      mission,
      results,
      errorMessage,
    });
    this._emitMissionUpdated(myMission);
    return myMission;
  }

  static async createRoute(payload) {
    const myRoute = await sequelize.models.MissionRoute.create({ ...payload });
    this._emitRouteUpdated(myRoute);
    return myRoute;
  }

  static async editMission({ id, uav, planId, status, initTime, endTime, task, mission, results, errorMessage }) {
    let myMission = await sequelize.models.Mission.findOne({ where: { id: id } });
    if (!myMission) {
      return null;
    }
    if (status) myMission.status = status;
    if (uav) myMission.uav = uav;
    if (planId != null) myMission.planId = planId;
    if (initTime) myMission.initTime = initTime;
    if (endTime) myMission.endTime = endTime;
    if (task) myMission.task = task;
    if (mission) myMission.mission = mission;
    if (results) myMission.results = results;
    if (errorMessage != null) myMission.errorMessage = errorMessage;
    await myMission.save();
    this._emitMissionUpdated(myMission);
    return myMission;
  }

  static async editRoute(
    { id, deviceId, missionId, status, initTime, endTime, task, mission, result, currentWp, totalWp, errorMessage },
    signals = {}
  ) {
    let myRoute = null;
    if (id) myRoute = await sequelize.models.MissionRoute.findOne({ where: { id: id } });
    if (deviceId && missionId)
      myRoute = await sequelize.models.MissionRoute.findOne({ where: { deviceId: deviceId, missionId: missionId } });
    if (!myRoute) {
      return null;
    }
    if (status) myRoute.status = status;
    if (initTime) myRoute.initTime = initTime;
    if (endTime) myRoute.endTime = endTime;
    if (task) myRoute.task = task;
    if (mission) myRoute.mission = mission;
    if (result) myRoute.result = result;
    if (currentWp !== undefined) myRoute.currentWp = currentWp;
    if (totalWp !== undefined) myRoute.totalWp = totalWp;
    if (errorMessage != null) myRoute.errorMessage = errorMessage;
    await myRoute.save();
    this._emitRouteUpdated(myRoute, signals);

    if (status === ROUTE_STATUS.COMPLETED) this._checkMissionComplete(myRoute.missionId);
    return myRoute;
  }

  static async _checkMissionComplete(missionId) {
    const routes = await this.getRoutes({ missionId: missionId });
    // A route in ERROR never flew — it doesn't block completion, but it means the
    // mission finished with failures. Only the "active" (non-error) routes need to
    // be COMPLETED for the mission to be considered done.
    const active = routes.filter((r) => r.status !== ROUTE_STATUS.ERROR);
    const hasErrors = active.length < routes.length;

    // If every route errored there is nothing to complete (mission already ERROR).
    if (active.length === 0) return;
    if (!active.every((r) => r.status === ROUTE_STATUS.COMPLETED)) return;

    const status = hasErrors ? MISSION_STATUS.COMPLETED_WITH_ERRORS : MISSION_STATUS.COMPLETED;
    // editMission() emits MISSION_UPDATED with the final status — no separate event needed.
    await this.editMission({ id: missionId, status, endTime: new Date() });
    logger.info(`WpTracking: Mission ${missionId} finished status=${status} (errors=${hasErrors})`);
  }

  static async decodeTask({ id, name, objetivo, locations, meteo }) {
    let myTask = {};
    myTask.id = id;
    myTask.name = name ? name : 'automatic';
    myTask.locations = locations;
    myTask.case = planningController.getCaseTypes()[objetivo].case;
    myTask.meteo = meteo;

    let devices = await devicesController.getAllDevices();
    // get setting of the task
    let param = planningController.getConfigParam(objetivo);
    let auxconfig = {};
    Object.keys(param['settings']).forEach((key1) => {
      auxconfig[key1] = param['settings'][key1].default;
    });
    logger.debug(`param devices: ${JSON.stringify(param['devices'])}`);

    let baseSettings = await planningController.getBasesSettings();
    if (!baseSettings || baseSettings.length === 0) {
      logger.warn('no base assignments found');
      return null;
    }
    let devicesSettings = [];
    for (const setting of baseSettings) {
      logger.debug(`bases setting: ${JSON.stringify(setting)}`);
      let config = { settings: {} };
      let myDevice = Object.values(devices).find((device) => device.id == setting.devices.id);
      for (const value of Object.keys(auxconfig)) {
        value !== 'base' ? (config.settings[value] = auxconfig[value]) : null;
      }
      // verify if the device is allow to do the mission
      if (param.devices && param.devices['category']) {
        const auvAllow = param.devices['category'].type.some((item) => item == myDevice.category);
        logger.debug(`device ${myDevice.name} auvAllow ${auvAllow}`);
        if (!auvAllow) {
          continue;
        }
      }
      // filter devices are online
      if (myDevice == null || myDevice.status == 'offline') {
        logger.info(`device offline ${myDevice.name}`);
        continue;
      }
      // filter devices are free
      const busyRoute = await this.getRoutes({
        deviceId: myDevice.id,
        status: [ROUTE_STATUS.RUNNING, ROUTE_STATUS.COMMANDED, ROUTE_STATUS.LOADED, ROUTE_STATUS.INIT],
      });
      if (busyRoute) {
        logger.info(`device ${myDevice.name} is busy`);
        continue;
      }

      config.id = myDevice.name;
      config.category = myDevice.category;
      // The external planner expects exactly [latitude, longitude, id] (see
      // server/test/api/json/missionRequest*.json) — build it explicitly
      // instead of Object.values(), whose order/length depends on the shape
      // of `setting.base` (a Sequelize Base instance has more fields now).
      config.settings.base = setting.base
        ? [setting.base.latitude, setting.base.longitude, setting.base.id]
        : [];
      config.settings.landing_mode = 2;
      let uavData = await positionsController.getByDeviceId(myDevice.id);
      logger.debug(`uavData: ${JSON.stringify(uavData)}`);
      if (uavData && uavData?.attributes?.batteryLevel) {
        if (!Number.isNaN(Number.parseFloat(uavData.attributes.batteryLevel))) {
          logger.debug(`device ${myDevice.name} battery ${uavData.attributes.batteryLevel}`);
          config.settings.battery_level = uavData.attributes.batteryLevel / 100;
        }
      }
      devicesSettings.push(config);
    }
    myTask.devices = devicesSettings.filter((item) => item != null);
    logger.info(
      `Task ${myTask.id} ${myTask.name} ${myTask.case} devices: ${myTask.devices.map((item) => item.id).flat()}`
    );
    logger.debug(`task devices: ${JSON.stringify(myTask.devices)}`);
    logger.debug(`task locations: ${JSON.stringify(myTask.locations)}`);
    return myTask;
  }

  // `externalId` is the task id assigned by the requesting external system (ExtApp).
  // It is NOT our primary key: we create the Mission first (its PK autoincrements),
  // then drive the planner and address ExtApp callbacks using the internal PK, while
  // externalId is persisted on the row so we can translate back to ExtApp later.
  static async sendTask({ id: rawExternalId, name, objetivo, locations, meteo }) {
    logger.info('command-sendtask');

    // Normalize the external id to a number at the boundary. The column is INTEGER but
    // SQLite is loosely typed: if the external system posts the id as a string ("118"),
    // an un-coerced value would be stored/queried as text and dedup (getMissionByExternalId)
    // would silently miss a later numeric re-send. Coerce once here so every downstream
    // use (dedup, persistence, ExtApp addressing) sees the same numeric value. Treat
    // null/undefined/'' (Number('') is 0, not NaN) as "no external id".
    const externalId =
      rawExternalId != null && rawExternalId !== '' && !Number.isNaN(Number(rawExternalId))
        ? Number(rawExternalId)
        : null;

    // Idempotency: the external system may re-send the same task (same externalId).
    // Only dedup against a mission that is still ALIVE (init/planning/running): a
    // re-send of an in-flight task must not spawn a duplicate row or a second
    // planner poll. But a re-send AFTER a terminal state (cancelled/error — e.g. the
    // first attempt cancelled because every UAV was busy) is a legitimate new attempt
    // and must be allowed to re-plan, so we do NOT short-circuit on those.
    if (externalId != null) {
      const existing = await this.getMissionByExternalId(externalId);
      if (existing && MISSION_ALIVE_STATUS.includes(existing.status)) {
        logger.warn(`sendTask: externalId=${externalId} already active as mission ${existing.id}, ignoring re-send`);
        return { response: existing.task, status: 'OK' };
      }
    }

    // decodeTask tags myTask with the EXTERNAL id only for logging/planner body
    // parity; it's overwritten with the internal PK below before we drive planning.
    let myTask = await this.decodeTask({ id: externalId, name, objetivo, locations, meteo });
    if (myTask == null) {
      logger.warn('myTask is null');
      await this.createMission({
        externalId,
        name,
        status: MISSION_STATUS.CANCELLED,
        task: myTask,
        errorMessage: 'No se pudo decodificar la tarea: datos de misión inválidos o incompletos',
      });
      return { response: myTask, status: 'ERROR' };
    }

    if (myTask.devices.length == 0) {
      logger.warn('no devices to do the mission');
      await this.createMission({
        externalId,
        name,
        status: MISSION_STATUS.CANCELLED,
        task: myTask,
        errorMessage: 'No hay dispositivos disponibles para ejecutar esta misión',
      });
      return { response: myTask, status: 'ERROR' };
    }

    // Create the mission FIRST so we have the internal PK. The planner is polled by
    // this PK and initMission() addresses the row by it — externalId stays on the row.
    const mission = await this.createMission({ externalId, name, task: myTask });
    const missionId = mission.id;
    myTask.id = missionId;

    const isPlanning = false;
    if (isPlanning) {
      let fileMission = readDataFile(`../config/mission/mission_1.yaml`);
      await this.initMission(missionId, { ...fileMission, id: missionId });
      return { response: myTask, status: 'OK' };
    }

    planningController.PlanningRequest({ id: missionId, myTask });

    eventsController.addEvent({
      type: 'info',
      deviceId: null,
      attributes: { action: 'RcvTask', message: 'Receive a task and sent to planner.' },
    });

    return { response: myTask, status: 'OK' };
  }

  /**
   * Entry point from the planning flow. The planner NEVER edits missions itself: it
   * hands whatever it has to initMission (a plan, or null on timeout) and this method
   * — the owner of the mission lifecycle — decides the outcome. `opts.timedOut` tells
   * apart "planner ran out of time" from "planner replied but the plan was unusable",
   * so the persisted errorMessage reflects the real cause.
   * @param {number} missionId
   * @param {object|null} mission - planner result, or null when the wait timed out
   * @param {{ timedOut?: boolean }} [opts]
   */
  static async initMission(missionId, mission, { timedOut = false } = {}) {
    logger.info('===== initMission =====');
    logger.debug(`initMission data: ${JSON.stringify(mission)}`);
    if (mission == null || !mission?.hasOwnProperty('route') || mission?.route?.length == 0) {
      const errorMessage = timedOut
        ? 'El planificador no respondió en el tiempo esperado'
        : 'El planificador no devolvió rutas válidas para esta misión';
      await this.editMission({
        id: missionId,
        status: MISSION_STATUS.ERROR,
        mission: mission,
        errorMessage,
      });
      logger.warn(`Mission ${missionId} cant planning (timedOut=${timedOut})`);
      return false;
    }
    // From here on any unexpected failure (DB error, etc.) must NOT propagate: the
    // planner calls this detached and has no business handling mission errors. Own the
    // failure here and drive the mission to ERROR ourselves, so it never gets stuck in
    // PLANNING and the caller never sees a rejected promise.
    try {
      // Resolve each planner route to its device, keeping the route↔device pairing so
      // every UAV gets ITS OWN route's waypoint count below (not route[0]'s). Skip
      // routes whose UAV is unknown/deleted instead of crashing on a null device.
      const routeDevices = [];
      for (const route of mission.route) {
        const findDevice = await devicesController.getByName(route.uav);
        if (!findDevice) {
          logger.warn(`initMission: route UAV '${route.uav}' not found in DB, skipping route`);
          continue;
        }
        logger.debug(`findDevice: ${JSON.stringify(findDevice.dataValues)}`);
        routeDevices.push({ route, deviceId: findDevice.id });
      }
      const listUAV = routeDevices.map((rd) => rd.deviceId);

      // Every planner route pointed at an unknown UAV → nothing to fly. Don't leave the
      // mission stuck in PLANNING with zero routes; fail it explicitly.
      if (routeDevices.length === 0) {
        await this.editMission({
          id: missionId,
          status: MISSION_STATUS.ERROR,
          mission,
          errorMessage: 'Ninguna ruta del planificador corresponde a un dispositivo conocido',
        });
        logger.warn(`initMission: mission ${missionId} has no resolvable UAVs, marking as ERROR`);
        return false;
      }

      const plan = await this.createMissionPlan(mission, { source: 'automatic' });
      logger.info(`MissionPlan created id=${plan.id} for automatic mission ${missionId}`);

      await this.editMission({
        id: missionId,
        uav: listUAV,
        planId: plan.id,
        status: MISSION_STATUS.PLANNING,
        mission: mission,
      });
      for (const { route, deviceId: uavId } of routeDevices) {
        const totalWp = route?.wp?.length ?? 0;
        let myroute = await this.createRoute({
          status: ROUTE_STATUS.INIT,
          missionId: missionId,
          deviceId: uavId,
          initTime: new Date(),
          endTime: null,
          result: {},
          currentWp: 0,
          totalWp,
        });
        missionSMModel.createActorMission(uavId, missionId, myroute.id);
      }

      const externalId = await this._resolveExternalId(missionId);
      ExtAppController.missionReqStart(externalId, mission);

      eventsController.addEvent({
        type: 'info',
        deviceId: null,
        attributes: { action: 'initMission', message: `Init mission ${missionId}` },
      });

      // Emitir evento al EventBus para que los subscribers lo manejen
      eventBus.emitSafe(EVENTS.MISSION_PLAN_SHOWN, { ...mission, name: 'name' });

      return { response: mission, status: 'OK' };
    } catch (err) {
      logger.error(`initMission(${missionId}) failed: ${err.message}`, err.stack);
      await this.editMission({
        id: missionId,
        status: MISSION_STATUS.ERROR,
        mission,
        errorMessage: `Fallo al inicializar la misión: ${err.message}`,
      });
      return false;
    }
  }

  static async deviceFinishSyncFiles({ name, id: _id }) {
    let mydevice = await devicesController.getByName(name);
    if (mydevice == null) {
      logger.warn(`mydevice name ${name} not found`);
      return false;
    }

    logger.debug(`deviceFinishSyncFiles: ${JSON.stringify(mydevice)}`);
    logger.info(`mydevice finish download files ${mydevice.id}`);

    eventsController.addEvent({
      type: 'info',
      deviceId: mydevice.id,
      attributes: { action: 'SyncFiles', message: `Finish sync files from device${mydevice.name}` },
    });
    missionSMModel.DownloadFiles(mydevice.id);
    return true;
  }

  static async deviceFinishMission({ name, id: _id }) {
    logger.debug(`deviceFinishMission name: ${name}`);
    let mydevice = await devicesController.getByName(name);
    if (mydevice == null) {
      logger.warn(`mydevice name ${name} not found`);
      return false;
    }
    logger.info(`mydevice in finish mission ${mydevice.id}`);
    eventsController.addEvent({
      type: 'info',
      deviceId: mydevice.id,
      attributes: { action: 'FinishMission', message: `Route complete successfully ${mydevice.name}` },
    });
    missionSMModel.UAVFinishMission(mydevice.id);
    return true;
  }

  static async UAVFinish(missionId, uavId) {
    let resultCode = 0;
    await this.editRoute({
      missionId: missionId,
      deviceId: uavId,
      status: ROUTE_STATUS.COMPLETED,
      endTime: new Date(),
    });

    eventsController.addEvent({
      type: 'info',
      deviceId: uavId,
      attributes: { action: 'MissionComplete', message: `Mission complete for UAV ${uavId}` },
    });

    const externalId = await this._resolveExternalId(missionId);
    await ExtAppController.missionReqResult(externalId, resultCode);
    return true;
  }

  static async updateFiles(missionId, uavId) {
    const myRoute = await this.getRoutes({ deviceId: uavId, missionId: missionId });
    const myMission = await this.getMissionValue(missionId);
    if (!myRoute || !myMission) {
      logger.warn(
        `updateFiles: mission=${missionId} or route for uav=${uavId} not found in DB, skipping file download`
      );
      return false;
    }
    await filesController.updateFiles(uavId, missionId, myRoute.id, myMission.initTime);
    await sleep(5000);
    return true;
  }

  static async UAVEnd(missionId, uavId) {
    logger.info('===== UAVEnd whole mission =====');
    const myRoute = await this.getRoutes({ missionId, deviceId: uavId });
    if (!myRoute) {
      logger.warn(`UAVEnd: no route found for mission=${missionId} uav=${uavId}`);
      return false;
    }
    const routeId = myRoute.id;
    const listfiles = await filesController.getFilesInfo({ routeId });
    logger.debug(`UAVEnd listfiles: ${JSON.stringify(listfiles)}`);
    const maxByMeasureName = {};
    let attributes = {};
    for (const file of listfiles) {
      if (file.attributes && file.attributes.hasOwnProperty('measures') && file.attributes.measures.length > 0) {
        for (const measure of file.attributes.measures) {
          if (measure.name && measure.value) {
            const isNewMax =
              !maxByMeasureName.hasOwnProperty(measure.name) ||
              Number(maxByMeasureName[measure.name]) < Number(measure.value);
            if (isNewMax) {
              maxByMeasureName[measure.name] = measure.value;
              attributes = file.attributes;
            }
          }
        }
      }
    }
    logger.debug(`UAVEnd attributes: ${JSON.stringify(attributes)}`);
    eventsController.addEvent({
      type: 'info',
      deviceId: uavId,
      attributes: { action: 'MissionEnd', message: `Mission ended for UAV ${uavId}` },
    });
    await this.editRoute({ id: routeId, status: ROUTE_STATUS.END, result: attributes, endTime: new Date() });

    if (attributes.hasOwnProperty('measures') && attributes.measures.length > 0) {
      const myMission = await this.getMissionValue(missionId);
      const existingResults = Array.isArray(myMission?.results) ? myMission.results : [];
      await this.editMission({ id: missionId, results: [...existingResults, attributes] });
    }

    const allRoutes = await this.getRoutes({ missionId });
    const active = allRoutes.filter((r) => r.status !== ROUTE_STATUS.ERROR);
    if (active.length > 0 && active.every((r) => r.status === ROUTE_STATUS.END)) {
      await this.notifyFinishProcessfiles(missionId);
    }
    return true;
  }

  static async notifyFinishProcessfiles(missionId) {
    logger.info('===== notifyFinishProcessfiles whole mission =====');
    let code = 0;
    let result = { files: [], data: {} };
    let myfiles = await filesController.getFilesInfo({ missionId });
    result.files = myfiles.map((file) => `${file.path}${file.name}`);
    const routes = await this.getRoutes({ missionId });
    result.data = routes.filter((r) => r.result).map((r) => ({ deviceId: r.deviceId, result: r.result }));
    eventsController.addEvent({
      type: 'info',
      deviceId: null,
      attributes: { action: 'Mission', message: `Finish process files for mission ${missionId}` },
    });
    const externalId = await this._resolveExternalId(missionId);
    ExtAppController.missionReqMedia(externalId, { code, files: result.files, data: result.data });
    return true;
  }

  static async updateMission({ device, mission, state }) {
    logger.debug(`updateMission device: ${device} mission: ${mission} state: ${state}`);
    // await sequelize.models.Route.update(
    //   { status: state },
    //   {
    //     where: {
    //       missionId: mission,
    //       deviceId: device,
    //     },
    //   }
    // );
    return true;
  }
  static async CheckLastMissionRoute() {
    let listMission = await this.getMissionValue();
    let listRoute = await this.getRoutes({});
    for (const mission of listMission) {
      if (
        mission.status == MISSION_STATUS.RUNNING ||
        mission.status == MISSION_STATUS.PLANNING ||
        mission.status == MISSION_STATUS.INIT
      ) {
        let listRoutes = listRoute.filter(
          (route) =>
            route.missionId == mission.id &&
            (route.status == ROUTE_STATUS.RUNNING ||
              route.status == ROUTE_STATUS.COMMANDED ||
              route.status == ROUTE_STATUS.LOADED ||
              route.status == ROUTE_STATUS.INIT)
        );
        for (const route of listRoutes) {
          await this.editRoute({
            id: route.id,
            status: ROUTE_STATUS.ERROR,
            errorMessage: 'Ruta interrumpida: el servidor se reinició mientras la misión estaba activa',
          });
        }
        await this.editMission({
          id: mission.id,
          status: MISSION_STATUS.ERROR,
          errorMessage: 'Misión interrumpida: el servidor se reinició mientras estaba activa',
        });
      }
    }
  }

  static async createMissionPlan(missionData, { name = null, source = 'manual' } = {}) {
    return await sequelize.models.MissionPlan.create({ missionData, name, source });
  }

  static async createMissionFromPlan(planId, { trigger = 'manual', uav = [] } = {}) {
    const plan = await sequelize.models.MissionPlan.findOne({ where: { id: planId } });
    if (!plan) return null;
    const initTime = new Date();
    const name = plan.name ?? `mission_plan_${planId}_${initTime.getTime()}`;
    return await this.createMission({
      name,
      planId,
      trigger,
      uav,
      status: MISSION_STATUS.RUNNING,
      initTime,
      mission: plan.missionData,
    });
  }

  /**
   * MANUAL flow — load. Mirrors initMission (automatic): creates MissionPlan +
   * Mission + one MissionRoute per device, and loads each drone's route via the
   * per-device command primitive. Persists the plan explicitly (no side-effect).
   * Routes that fail after one retry are left in ROUTE_STATUS.ERROR so `command`
   * can skip them. Returns { missionId, planId, results }.
   * @param {object} missionData - { route: [...], version }
   */
  static async loadMissionManual(missionData) {
    logger.info('===== loadMissionManual =====');
    if (missionData == null || !Array.isArray(missionData.route) || missionData.route.length === 0) {
      return { missionId: null, planId: null, results: [], state: 'info', msg: 'no mission' };
    }

    // Persist with version:'3' so consumers (client RuteConvert) parse it as the
    // current route format instead of falling back to the legacy parser.
    const normalizedMission = { ...missionData, version: '3' };
    const plan = await this.createMissionPlan(normalizedMission, { source: 'manual' });
    logger.info(`loadMissionManual: MissionPlan created id=${plan.id}`);

    // Resolve devices for the mission (skip routes whose UAV is unknown).
    const routeDevices = [];
    for (const route of normalizedMission.route) {
      const device = await devicesController.getByName(route.uav);
      routeDevices.push({ route, device });
    }
    const listUAV = routeDevices.filter((rd) => rd.device).map((rd) => rd.device.id);

    // Idempotency: a drone can't have two not-yet-commanded missions at once.
    // Cancel any INIT mission that overlaps with this load's devices before
    // creating the new one, so double-clicks / repeated loads don't pile up
    // orphaned Mission/Route rows or re-send configureMission redundantly.
    await this._cancelStaleInitMissions(listUAV);

    const mission = await this.createMission({
      name: `manual_${plan.id}`,
      planId: plan.id,
      trigger: 'manual',
      uav: listUAV,
      // Loaded, not yet commanded: INIT until commandMissionManual promotes it.
      status: MISSION_STATUS.INIT,
      mission: normalizedMission,
    });
    logger.info(`loadMissionManual: Mission created id=${mission.id} devices=[${listUAV.join(',')}]`);

    const results = [];
    for (const { route, device } of routeDevices) {
      if (!device) {
        results.push({ deviceId: null, name: route.uav, state: 'warning', msg: `device ${route.uav} not found` });
        continue;
      }
      const totalWp = route.wp?.length ?? 0;
      let response;
      try {
        // Pass the FULL mission; loadMissionToDevice extracts this drone's route.
        response = await withRetry(() =>
          commandsController.sendCommandDevice({
            deviceId: device.id,
            type: 'loadMission',
            attributes: normalizedMission,
          })
        );
      } catch (err) {
        response = { state: 'error', msg: err instanceof Error ? err.message : String(err) };
      }
      const ok = response.state !== 'error';
      await this.createRoute({
        missionId: mission.id,
        deviceId: device.id,
        status: ok ? ROUTE_STATUS.LOADED : ROUTE_STATUS.ERROR,
        initTime: new Date(),
        currentWp: 0,
        totalWp,
        errorMessage: ok ? null : `Fallo al cargar la ruta en el dispositivo: ${response.msg ?? 'error desconocido'}`,
      });
      results.push({ deviceId: device.id, name: device.name, state: response.state, msg: response.msg });
    }

    // If no route loaded (e.g. the only drone failed), the mission is unusable → ERROR.
    // Otherwise it stays INIT and command will handle the routes that did load.
    const anyLoaded = results.some((r) => r.state !== 'error' && r.deviceId != null);
    const finalStatus = anyLoaded ? MISSION_STATUS.INIT : MISSION_STATUS.ERROR;
    if (!anyLoaded) {
      await this.editMission({ id: mission.id, status: finalStatus, errorMessage: 'Ninguna ruta se pudo cargar' });
    }

    logger.info(`loadMissionManual finished mission=${mission.id} anyLoaded=${anyLoaded}`);
    return { missionId: mission.id, planId: plan.id, results };
  }

  /**
   * MANUAL flow — command. Mission + routes already exist (created in load).
   * Commands each LOADED route (skips ERROR ones) and promotes LOADED → COMMANDED
   * so missionWpTracking.checkProgress picks it up. Returns { missionId, results }.
   * @param {number} missionId
   */
  static async commandMissionManual(missionId) {
    logger.info(`===== commandMissionManual mission=${missionId} =====`);
    const routes = await this.getRoutes({ missionId });
    const routeList = Array.isArray(routes) ? routes : Object.values(routes ?? {});
    if (routeList.length === 0) {
      return { missionId, results: [], state: 'warning', msg: `mission ${missionId} has no routes` };
    }

    const results = [];
    for (const route of routeList) {
      if (route.status !== ROUTE_STATUS.LOADED) {
        results.push({ deviceId: route.deviceId, state: 'warning', msg: `route not loaded (status=${route.status})` });
        continue;
      }
      let response;
      try {
        response = await withRetry(() =>
          commandsController.sendCommandDevice({ deviceId: route.deviceId, type: 'commandMission' })
        );
      } catch (err) {
        response = { state: 'error', msg: err instanceof Error ? err.message : String(err) };
      }
      if (response.state !== 'error') {
        await this.editRoute({ id: route.id, status: ROUTE_STATUS.COMMANDED });
      }
      results.push({ deviceId: route.deviceId, state: response.state, msg: response.msg });
    }

    // RUNNING if at least one route was commanded OK; ERROR if none could be
    // (all failed or none were in LOADED to begin with).
    const anyCommanded = results.some((r) => r.state !== 'error' && r.state !== 'warning');
    const finalStatus = anyCommanded ? MISSION_STATUS.RUNNING : MISSION_STATUS.ERROR;
    if (anyCommanded) {
      await this.editMission({ id: missionId, status: finalStatus });
    } else {
      await this.editMission({ id: missionId, status: finalStatus, errorMessage: 'Ninguna ruta se pudo comandar' });
    }

    logger.info(`commandMissionManual finished mission=${missionId} anyCommanded=${anyCommanded}`);
    return { missionId, results };
  }

  /**
   * Cancels any Mission still in INIT (loaded but not yet commanded) that shares
   * at least one device with `deviceIds`. Called before creating a new manual
   * Mission so a repeated/duplicate load doesn't leave the old one orphaned in
   * INIT forever — the new load supersedes it.
   * @param {number[]} deviceIds
   */
  static async _cancelStaleInitMissions(deviceIds) {
    if (deviceIds.length === 0) return;

    const staleMissions = await sequelize.models.Mission.findAll({
      where: { status: MISSION_STATUS.INIT },
    });
    for (const stale of staleMissions) {
      const overlaps = (stale.uav ?? []).some((id) => deviceIds.includes(id));
      if (!overlaps) continue;

      await this.editMission({ id: stale.id, status: MISSION_STATUS.CANCELLED });
      const routes = await this.getRoutes({ missionId: stale.id });
      const routeList = Array.isArray(routes) ? routes : Object.values(routes ?? {});
      // Per-route edit (not a bulk update) so each row goes through editRoute()
      // and emits ROUTE_UPDATED — stale-route volume is low, consistency wins.
      for (const route of routeList) {
        await this.editRoute({ id: route.id, status: ROUTE_STATUS.CANCELLED });
      }
      logger.info(`_cancelStaleInitMissions: cancelled stale mission=${stale.id} (device overlap with new load)`);
    }
  }

  static async getMissionPlan(id) {
    return await sequelize.models.MissionPlan.findOne({ where: { id } });
  }

  static async getAllMissionPlans() {
    return await sequelize.models.MissionPlan.findAll({ order: [['createdAt', 'DESC']] });
  }

  static async convertBriefingToXYZ(missionBriefing) {
    return await convertMissionBriefingToXYZ(missionBriefing);
  }

  static convertXYZToGeodetic(missionDataXYZ) {
    return convertMissionXYZToLatLong(missionDataXYZ);
  }

  static async showMissionXYZ(missionDataXYZ) {
    logger.info(`[MissionShowXYZ] Starting mission show in XYZ coordinates`);
    let missionxyz = { ...missionDataXYZ, version: '3' };
    try {
      logger.debug(`[MissionShowXYZ] Input data:`, JSON.stringify(missionDataXYZ, null, 2));
      const mission = convertMissionXYZToLatLong(missionxyz);
      logger.debug(`[MissionShowXYZ] Converted mission:`, JSON.stringify(mission, null, 2));
      const response = await this.broadcastMission(mission);
      logger.info(`[MissionShowXYZ] Mission show completed`);
      return response;
    } catch (error) {
      logger.error(`[MissionShowXYZ] Error processing mission:`, error.message);
      logger.error(`[MissionShowXYZ] Stack:`, error.stack);
      throw error;
    }
  }
}

missionModel.CheckLastMissionRoute();

import { devicesController } from '../../controllers/devices.js';
import { positionsController } from '../../controllers/positions.js';
import { missionSMModel } from './missionSM.js';
import { ExtAppController } from '../../controllers/ExtApp.js';
import { planningController } from '../../controllers/planning.js';
import { filesController } from '../../controllers/files.js';
import { eventsController } from '../../controllers/events.js';
import { commandsController } from '../../controllers/commands.js';
import { readDataFile, writeJSON, sleep, withRetry } from '../../common/utils.js';
import sequelize from '../../common/sequelize.js';
import { Op } from 'sequelize';
import { eventBus, EVENTS } from '../../common/eventBus.js';
import { convertMissionXYZToLatLong, convertMissionBriefingToXYZ } from './coordinateConverter.js';
import { missionLogger as logger } from '../../common/logger.js';
import { MISSION_STATUS, ROUTE_STATUS } from '../../config/status.js';

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

  static async getRoutes({ id, deviceId, missionId }) {
    if (deviceId && missionId) {
      return await sequelize.models.MissionRoute.findOne({ where: { deviceId: deviceId, missionId: missionId } });
    }
    if (id) {
      return await sequelize.models.MissionRoute.findOne({ where: { id: id } });
    }
    if (missionId) {
      return await sequelize.models.MissionRoute.findAll({ where: { missionId: missionId } });
    }
    return await sequelize.models.MissionRoute.findAll();
  }

  static async broadcastMission(mission) {
    if (mission == null || !mission?.hasOwnProperty('route') || mission?.route?.length == 0) {
      return { success: false };
    }
    eventBus.emitSafe(EVENTS.MISSION_CREATED, { ...mission, name: mission.name ? mission.name : 'name' });
    return { success: true };
  }

  static async createMission({
    id,
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
    // id is only provided by the automatic (ExtApp) flow — use findOrCreate for idempotency.
    // Manual flow has no external id: use plain create and let autoIncrement assign one.
    if (id != null) {
      const [instance, created] = await sequelize.models.Mission.findOrCreate({
        where: { id },
        defaults: { name, planId, trigger, uav, status, initTime, endTime, task, mission, results, errorMessage },
      });
      if (!created) logger.warn(`createMission: id=${id} already exists, returning existing record`);
      return instance;
    }
    return await sequelize.models.Mission.create({
      name, planId, trigger, uav, status, initTime, endTime, task, mission, results, errorMessage,
    });
  }

  static async createRoute(payload) {
    return await sequelize.models.MissionRoute.create({ ...payload });
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
    return myMission;
  }

  static async editRoute({
    id,
    deviceId,
    missionId,
    status,
    initTime,
    endTime,
    task,
    mission,
    results,
    currentWp,
    totalWp,
  }) {
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
    if (results) myRoute.results = results;
    if (currentWp !== undefined) myRoute.currentWp = currentWp;
    if (totalWp !== undefined) myRoute.totalWp = totalWp;
    await myRoute.save();

    if (status === ROUTE_STATUS.COMPLETED) this.validateMission(missionId);
    return myRoute;
  }

  static async validateMission(missionId) {
    const myRoute = await this.getRoutes({ missionId: missionId });
    const allFinish = myRoute.every((route) => route.status == ROUTE_STATUS.COMPLETED);

    if (allFinish) {
      let myMission = await sequelize.models.Mission.findOne({ where: { id: missionId } });
      myMission.status = MISSION_STATUS.COMPLETED;
      await myMission.save();
      this.FinishProcessFiles(missionId);
    }
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

    let baseSettings = planningController.getBasesSettings();
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
      let getRoutes = await this.getRoutes({ deviceId: myDevice.id });
      if (
        getRoutes.some(
          (route) =>
            route.status == ROUTE_STATUS.RUNNING ||
            route.status == ROUTE_STATUS.COMMANDED ||
            route.status == ROUTE_STATUS.LOADED ||
            route.status == ROUTE_STATUS.INIT
        )
      ) {
        logger.info(`device ${myDevice.name} is busy`);
        continue;
      }

      config.id = myDevice.name;
      config.category = myDevice.category;
      config.settings.base = setting.base ? Object.values(setting.base) : [];
      config.settings.landing_mode = 2;
      let uavData = await positionsController.getLastPositions(myDevice.id);
      logger.debug(`uavData: ${JSON.stringify(uavData)}`);
      if (uavData && uavData[0]?.attributes?.batteryLevel) {
        if (!Number.isNaN(Number.parseFloat(uavData[0].attributes.batteryLevel))) {
          logger.debug(`device ${myDevice.name} battery ${uavData[0].attributes.batteryLevel}`);
          config.settings.battery_level = uavData[0].attributes.batteryLevel / 100;
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

  static async sendTask({ id, name, objetivo, locations, meteo }) {
    logger.info('command-sendtask');

    const isPlanning = false;
    if (isPlanning) {
      let mission = readDataFile(`../config/mission/mission_1.yaml`);
      await this.initMission(id, { ...mission, id: id });
      return { response: myTask, status: 'OK' };
    }

    let myTask = await this.decodeTask({ id, name, objetivo, locations, meteo });
    if (myTask == null) {
      logger.warn('myTask is null');
      await this.createMission({ id, status: MISSION_STATUS.CANCELLED, task: myTask, errorMessage: 'No se pudo decodificar la tarea: datos de misión inválidos o incompletos' });
      return { response: myTask, status: 'ERROR' };
    }

    if (myTask.devices.length == 0) {
      logger.warn('no devices to do the mission');
      await this.createMission({ id, status: MISSION_STATUS.CANCELLED, task: myTask, errorMessage: 'No hay dispositivos disponibles para ejecutar esta misión' });
      return { response: myTask, status: 'ERROR' };
    }

    planningController.PlanningRequest({ id, myTask });

    await this.createMission({ id, name, task: myTask });

    eventsController.addEvent({
      type: 'info',
      deviceId: null,
      attributes: { message: 'Ext APP send task' },
    });

    return { response: myTask, status: 'OK' };
  }

  static async initMission(missionId, mission) {
    logger.info('===== initMission =====');
    logger.debug(`initMission data: ${JSON.stringify(mission)}`);
    if (mission == null || !mission?.hasOwnProperty('route') || mission?.route?.length == 0) {
      await this.editMission({ id: missionId, status: MISSION_STATUS.ERROR, mission: mission, errorMessage: 'El planificador no devolvió rutas válidas para esta misión' });
      logger.warn(`Mission ${missionId} cant planning`);
      return false;
    }
    const listUAV = [];
    for (const route of mission.route) {
      let findDevice = await devicesController.getByName(route.uav);
      logger.debug(`findDevice: ${JSON.stringify(findDevice.dataValues)}`);
      listUAV.push(findDevice.id);
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
    for (const uavId of listUAV) {
      const routeData = mission.route.find((r) => {
        return true;
      });
      const totalWp = routeData?.wp?.length ?? 0;
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

    ExtAppController.missionReqStart(missionId, mission);

    eventsController.addEvent({
      type: 'info',
      deviceId: null,
      attributes: { message: `Init mission ${missionId}` },
    });

    // Emitir evento al EventBus para que los subscribers lo manejen
    eventBus.emitSafe(EVENTS.MISSION_INIT, { ...mission, name: 'name' });

    return { response: mission, status: 'OK' };
  }

  static async deviceFinishSyncFiles({ name, id }) {
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
      attributes: { message: `Finish mission ${mydevice.name}` },
    });
    missionSMModel.DownloadFiles(mydevice.id);
    return true;
  }

  static async deviceFinishMission({ name, id }) {
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
      attributes: { message: `Finish mission ${mydevice.name}` },
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
      attributes: { message: `Device end` },
    });

    await ExtAppController.missionReqResult(missionId, resultCode);
    return true;
  }

  static async updateFiles(missionId, uavId) {
    const myRoute = await this.getRoutes({ deviceId: uavId, missionId: missionId });
    const myMission = await this.getMissionValue(missionId);
    if (!myRoute || !myMission) {
      logger.warn(`updateFiles: mission=${missionId} or route for uav=${uavId} not found in DB, skipping file download`);
      return false;
    }
    const results = await filesController.updateFiles(uavId, missionId, myRoute.id, myMission.initTime);
    await sleep(5000);
    return true;
  }

  static async UAVEnd(missionId, uavId) {
    logger.info('===== UAVEnd whole mission =====');
    const myRoute = await this.getRoutes({ missionId, uavId }).id;
    const routeId = myRoute.id;
    const listfiles = await filesController.getFilesInfo({ routeId });
    logger.debug(`UAVEnd listfiles: ${JSON.stringify(listfiles)}`);
    const results = {};
    let attributes = {};
    for (const file of listfiles) {
      if (file.attributes && file.attributes.hasOwnProperty('measures') && file.attributes.measures.length > 0) {
        for (const measure of file.attributes.measures) {
          if (measure.name && measure.value) {
            if (!results.hasOwnProperty(measure.name)) {
              results[measure.name] = measure.value;
              attributes = file.attributes;
            } else if (Number(results[measure.name]) < Number(measure.value)) {
              results[measure.name] = measure.value;
              attributes = file.attributes;
            }
          }
        }
      }
    }
    logger.debug(`UAVEnd results: ${JSON.stringify(results)} attributes: ${JSON.stringify(attributes)}`);
    eventsController.addEvent({
      type: 'info',
      deviceId: uavId,
      attributes: { message: `device end ` },
    });
    await this.editRoute({ id: routeId, status: ROUTE_STATUS.END, result: attributes, endTime: new Date() });
    return true;
  }

  static async FinishProcessFiles(missionId) {
    logger.info('===== FinishProcessFiles whole mission =====');
    let code = 0;
    let result = { files: [], data: {} };
    let myfiles = await filesController.getFilesInfo({ missionId });
    result.files = myfiles.map((file) => `${file.route}${file.name}`);
    const myMission = await this.getMissionValue(missionId);
    result.data = myMission.results;
    eventsController.addEvent({
      type: 'info',
      deviceId: null,
      attributes: { message: `Finish mission ${missionId}` },
    });
    ExtAppController.missionReqMedia(missionId, { code, files: result.files, data: result.data });
    return true;
  }

  static async updateMission({ device, mission, state }) {
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
          await this.editRoute({ id: route.id, status: ROUTE_STATUS.ERROR });
        }
        await this.editMission({ id: mission.id, status: MISSION_STATUS.ERROR, errorMessage: 'Misión interrumpida: el servidor se reinició mientras estaba activa' });
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
          commandsController.sendCommandDevice({ deviceId: device.id, type: 'loadMission', attributes: normalizedMission })
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

    // The client only discovers new missions via the initial REST fetch or by
    // seeing a missionProgress for an unknown missionId (SocketController auto-
    // fetches it then). Emit one now so ActiveMissionsPopover picks up the mission
    // right after load, without waiting for a page refresh.
    for (const { deviceId, state } of results) {
      if (deviceId == null) continue;
      eventBus.emitSafe(EVENTS.MISSION_PROGRESS, {
        missionId: mission.id,
        deviceId,
        currentWp: 0,
        totalWp: routeDevices.find((rd) => rd.device?.id === deviceId)?.route.wp?.length ?? 0,
        completed: false,
        anomalies: [],
        wpEstimate: null,
        confidence: null,
        missionStatus: finalStatus,
        routeStatus: state === 'error' ? ROUTE_STATUS.ERROR : ROUTE_STATUS.LOADED,
      });
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

    // Nudge the client so ActiveMissionsPopover reflects the new mission/route
    // status right away (see loadMissionManual for why this is needed).
    for (const route of routeList) {
      const result = results.find((r) => r.deviceId === route.deviceId);
      if (!result || result.state === 'warning') continue;
      eventBus.emitSafe(EVENTS.MISSION_PROGRESS, {
        missionId,
        deviceId: route.deviceId,
        currentWp: route.currentWp ?? 0,
        totalWp: route.totalWp ?? 0,
        completed: false,
        anomalies: [],
        wpEstimate: null,
        confidence: null,
        missionStatus: finalStatus,
        routeStatus: result.state === 'error' ? route.status : ROUTE_STATUS.COMMANDED,
      });
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
      await sequelize.models.MissionRoute.update(
        { status: ROUTE_STATUS.CANCELLED },
        { where: { missionId: stale.id } }
      );
      logger.info(`_cancelStaleInitMissions: cancelled stale mission=${stale.id} (device overlap with new load)`);

      // Nudge the client so ActiveMissionsPopover drops/updates the stale mission
      // instead of showing it stuck in 'init' (same reasoning as loadMissionManual).
      for (const route of routeList) {
        eventBus.emitSafe(EVENTS.MISSION_PROGRESS, {
          missionId: stale.id,
          deviceId: route.deviceId,
          currentWp: route.currentWp ?? 0,
          totalWp: route.totalWp ?? 0,
          completed: false,
          anomalies: [],
          wpEstimate: null,
          confidence: null,
          missionStatus: MISSION_STATUS.CANCELLED,
          routeStatus: ROUTE_STATUS.CANCELLED,
        });
      }
    }
  }

  static async getMissionPlan(id) {
    return await sequelize.models.MissionPlan.findOne({ where: { id } });
  }

  static async getAllMissionPlans() {
    return await sequelize.models.MissionPlan.findAll({ order: [['createdAt', 'DESC']] });
  }

  static convertBriefingToXYZ(missionBriefing) {
    return convertMissionBriefingToXYZ(missionBriefing);
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

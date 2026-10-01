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
import {
  MISSION_STATUS,
  TASK_STATUS,
  MISSION_ALIVE_STATUS,
  TASK_ACTIVE_STATUS,
  TASK_TERMINAL_STATUS,
  TASK_FAILED_STATUS,
} from '../../config/status.js';
import { normalizeMission, missionForTask, TASK_GRAPH_VERSION } from './taskGraph.js';
import { taskScheduler } from './taskScheduler.js';

// Plans are stored as received, tagged with their format: consumers (client RuteConvert)
// parse route[] as '3' and tasks[] as '4'.
const withFormatVersion = (missionData) => ({
  ...missionData,
  version: missionData.tasks ? TASK_GRAPH_VERSION : '3',
});

/**
 * @typedef Mission
 * @property {integer} id
 * @property {string} status - see MISSION_STATUS in config/status.js
 * @property {string} initTime
 * @property {string} FinishTime
 * @property {Array<number>} uav
 * @property {object} request - mission request (ExtApp or planning UI) that originated it
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

  // Resolve a mission by the external system's request id (ExtApp dedup / addressing).
  // Coerce to Number so a string-typed id ("118") still matches the INTEGER column.
  static async getMissionByExternalId(externalId) {
    if (externalId == null) return null;
    const numId = Number(externalId);
    if (Number.isNaN(numId)) return null;
    return await sequelize.models.Mission.findOne({ where: { externalId: numId } });
  }

  // Translate an internal mission PK into the external system's request id for ExtApp
  // callbacks. Falls back to the PK itself when the mission has no external origin
  // (e.g. legacy rows or manual missions that somehow reach an ExtApp callback).
  static async _resolveExternalId(missionId) {
    const myMission = await this.getMissionValue(missionId);
    return myMission?.externalId ?? missionId;
  }

  static async getTask(id) {
    return await sequelize.models.MissionTask.findByPk(id);
  }

  // Every filter is optional and they combine (AND). A device can hold several tasks
  // in one mission, so (missionId, deviceId) is a list, never a single task.
  static async getTasks({ id, missionId, deviceId, status } = {}) {
    const where = {};
    if (id != null) where.id = id;
    if (missionId != null) where.missionId = missionId;
    if (deviceId != null) where.deviceId = deviceId;
    if (status != null) where.status = Array.isArray(status) ? { [Op.in]: status } : status;
    return await sequelize.models.MissionTask.findAll({ where, order: [['id', 'ASC']] });
  }

  static async getActiveTaskForDevice(deviceId) {
    return await sequelize.models.MissionTask.findOne({
      where: { deviceId, status: { [Op.in]: TASK_ACTIVE_STATUS } },
      order: [['id', 'DESC']],
    });
  }

  // Tasks in flight (COMMANDED/RUNNING). Used once at server startup to bootstrap
  // missionWpTracking's in-memory registry, so tasks already flying when the process
  // restarts are picked up without waiting for their next DB write.
  static async getTrackableTasks() {
    return await this.getTasks({ status: [TASK_STATUS.COMMANDED, TASK_STATUS.RUNNING] });
  }

  // Single emission point for Mission/MissionTask row changes — called from
  // create*/edit* right after persisting, so every write path (manual, automatic,
  // state-machine-driven) notifies clients uniformly and payload shape can't drift.
  static _emitMissionUpdated(mission) {
    eventBus.emitSafe(EVENTS.MISSION_UPDATED, mission.get({ plain: true }));
  }

  // `signals` (anomalies/wpEstimate/confidence) are transient wpTracking diagnostics —
  // never persisted, just merged into the outbound payload when provided.
  static _emitTaskUpdated(task, signals = {}) {
    eventBus.emitSafe(EVENTS.TASK_UPDATED, { ...task.get({ plain: true }), ...signals });
  }

  // Same shape as _emitTaskUpdated, but for callers (missionWpTracking's in-memory
  // registry) that already hold a plain cached task object instead of a Sequelize
  // instance — lets the "signals only, nothing persisted" path emit without any DB
  // round-trip.
  static emitTaskSignals(task, signals = {}) {
    eventBus.emitSafe(EVENTS.TASK_UPDATED, { ...task, ...signals });
  }

  static async broadcastMission(mission) {
    const hasPlan = (mission?.route?.length ?? 0) > 0 || (mission?.tasks?.length ?? 0) > 0;
    if (!hasPlan) {
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
    request = {},
    mission = {},
    results = [],
    errorMessage = null,
  }) {
    if (name == null) name = `automatic_${initTime.getTime()}`;

    // externalId is only provided by the automatic (ExtApp) flow. Dedup for
    // idempotency, but ONLY against a mission that is still alive: a re-sent request
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
      request,
      mission,
      results,
      errorMessage,
    });
    this._emitMissionUpdated(myMission);
    return myMission;
  }

  static async createTask(payload) {
    const task = await sequelize.models.MissionTask.create({ ...payload });
    this._emitTaskUpdated(task);
    return task;
  }

  static async editMission({ id, uav, planId, status, initTime, endTime, request, mission, results, errorMessage }) {
    let myMission = await sequelize.models.Mission.findOne({ where: { id: id } });
    if (!myMission) {
      return null;
    }
    if (status) myMission.status = status;
    if (uav) myMission.uav = uav;
    if (planId != null) myMission.planId = planId;
    if (initTime) myMission.initTime = initTime;
    if (endTime) myMission.endTime = endTime;
    if (request) myMission.request = request;
    if (mission) myMission.mission = mission;
    if (results) myMission.results = results;
    if (errorMessage != null) myMission.errorMessage = errorMessage;
    await myMission.save();
    this._emitMissionUpdated(myMission);
    return myMission;
  }

  static async editTask({ id, status, initTime, endTime, result, currentWp, totalWp, errorMessage }, signals = {}) {
    const task = await this.getTask(id);
    if (!task) {
      return null;
    }
    if (status) task.status = status;
    if (initTime) task.initTime = initTime;
    if (endTime) task.endTime = endTime;
    if (result) task.result = result;
    if (currentWp !== undefined) task.currentWp = currentWp;
    if (totalWp !== undefined) task.totalWp = totalWp;
    if (errorMessage != null) task.errorMessage = errorMessage;
    await task.save();
    this._emitTaskUpdated(task, signals);

    if (TASK_TERMINAL_STATUS.includes(status)) await this._checkMissionComplete(task.missionId);
    return task;
  }

  // The mission is over once every task is terminal. Only an alive mission is closed
  // here: one already CANCELLED/ERROR by its own flow keeps that status and message.
  static async _checkMissionComplete(missionId) {
    const mission = await this.getMissionValue(missionId);
    if (!mission || !MISSION_ALIVE_STATUS.includes(mission.status)) return;

    const tasks = await this.getTasks({ missionId });
    if (tasks.length === 0 || !tasks.every((t) => TASK_TERMINAL_STATUS.includes(t.status))) return;

    const failed = tasks.filter((t) => TASK_FAILED_STATUS.includes(t.status));
    if (failed.length === tasks.length) {
      await this.editMission({
        id: missionId,
        status: MISSION_STATUS.ERROR,
        endTime: new Date(),
        errorMessage: 'Ninguna tarea de la misión se completó',
      });
      logger.info(`Mission ${missionId} finished status=${MISSION_STATUS.ERROR} (no task completed)`);
      return;
    }

    const status = failed.length > 0 ? MISSION_STATUS.COMPLETED_WITH_ERRORS : MISSION_STATUS.COMPLETED;
    // editMission() emits MISSION_UPDATED with the final status — no separate event needed.
    await this.editMission({ id: missionId, status, endTime: new Date() });
    logger.info(`Mission ${missionId} finished status=${status} (failedTasks=${failed.length})`);
  }

  static async decodeMissionRequest({ id, name, objetivo, locations, meteo }) {
    let missionRequest = {};
    missionRequest.id = id;
    missionRequest.name = name ? name : 'automatic';
    missionRequest.locations = locations;
    missionRequest.case = planningController.getCaseTypes()[objetivo].case;
    missionRequest.meteo = meteo;

    let devices = await devicesController.getAllDevices();
    // get settings of the requested objective
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
      // INIT counts as busy: the device is already committed to a task of a live mission.
      const busyTasks = await this.getTasks({
        deviceId: myDevice.id,
        status: [TASK_STATUS.INIT, ...TASK_ACTIVE_STATUS],
      });
      if (busyTasks.length > 0) {
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
    missionRequest.devices = devicesSettings.filter((item) => item != null);
    logger.info(
      `MissionRequest ${missionRequest.id} ${missionRequest.name} ${missionRequest.case} devices: ${missionRequest.devices.map((item) => item.id).flat()}`
    );
    logger.debug(`missionRequest devices: ${JSON.stringify(missionRequest.devices)}`);
    logger.debug(`missionRequest locations: ${JSON.stringify(missionRequest.locations)}`);
    return missionRequest;
  }

  // `externalId` is the request id assigned by the requesting external system (ExtApp).
  // It is NOT our primary key: we create the Mission first (its PK autoincrements),
  // then drive the planner and address ExtApp callbacks using the internal PK, while
  // externalId is persisted on the row so we can translate back to ExtApp later.
  static async requestMission({ id: rawExternalId, name, objetivo, locations, meteo }) {
    logger.info('requestMission');

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

    // Idempotency: the external system may re-send the same request (same externalId).
    // Only dedup against a mission that is still ALIVE (init/planning/running): a
    // re-send of an in-flight request must not spawn a duplicate row or a second
    // planner poll. But a re-send AFTER a terminal state (cancelled/error — e.g. the
    // first attempt cancelled because every UAV was busy) is a legitimate new attempt
    // and must be allowed to re-plan, so we do NOT short-circuit on those.
    if (externalId != null) {
      const existing = await this.getMissionByExternalId(externalId);
      if (existing && MISSION_ALIVE_STATUS.includes(existing.status)) {
        logger.warn(`requestMission: externalId=${externalId} already active as mission ${existing.id}, ignoring re-send`);
        return { response: existing.request, status: 'OK' };
      }
    }

    // decodeMissionRequest tags missionRequest with the EXTERNAL id only for logging/planner body
    // parity; it's overwritten with the internal PK below before we drive planning.
    let missionRequest = await this.decodeMissionRequest({ id: externalId, name, objetivo, locations, meteo });
    if (missionRequest == null) {
      logger.warn('missionRequest is null');
      await this.createMission({
        externalId,
        name,
        status: MISSION_STATUS.CANCELLED,
        request: missionRequest,
        errorMessage: 'No se pudo decodificar la solicitud de misión: datos inválidos o incompletos',
      });
      return { response: missionRequest, status: 'ERROR' };
    }

    if (missionRequest.devices.length == 0) {
      logger.warn('no devices to do the mission');
      await this.createMission({
        externalId,
        name,
        status: MISSION_STATUS.CANCELLED,
        request: missionRequest,
        errorMessage: 'No hay dispositivos disponibles para ejecutar esta misión',
      });
      return { response: missionRequest, status: 'ERROR' };
    }

    // Create the mission FIRST so we have the internal PK. The planner is polled by
    // this PK and initMission() addresses the row by it — externalId stays on the row.
    const mission = await this.createMission({ externalId, name, request: missionRequest });
    const missionId = mission.id;
    missionRequest.id = missionId;

    const isPlanning = false;
    if (isPlanning) {
      let fileMission = readDataFile(`../config/mission/mission_1.yaml`);
      await this.initMission(missionId, { ...fileMission, id: missionId });
      return { response: missionRequest, status: 'OK' };
    }

    planningController.PlanningRequest({ id: missionId, missionRequest });

    eventsController.addEvent({
      type: 'info',
      deviceId: null,
      attributes: { action: 'RcvTask', message: 'Received a mission request and sent it to the planner.' },
    });

    return { response: missionRequest, status: 'OK' };
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
    const hasPlan = (mission?.route?.length ?? 0) > 0 || (mission?.tasks?.length ?? 0) > 0;
    if (!hasPlan) {
      const errorMessage = timedOut
        ? 'El planificador no respondió en el tiempo esperado'
        : 'El planificador no devolvió tareas válidas para esta misión';
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
      // Validate the graph before persisting anything: an invalid plan fails the
      // mission (catch below) without leaving a MissionPlan behind.
      const { tasks } = normalizeMission(mission);
      const devices = await this._resolveDevices(tasks);

      // Every task pointed at an unknown device → nothing to fly. Fail it explicitly
      // instead of leaving it alive with nothing that can ever run.
      if (devices.size === 0) {
        await this.editMission({
          id: missionId,
          status: MISSION_STATUS.ERROR,
          mission,
          errorMessage: 'Ninguna tarea del planificador corresponde a un dispositivo conocido',
        });
        logger.warn(`initMission: mission ${missionId} has no resolvable devices, marking as ERROR`);
        return false;
      }

      const plan = await this.createMissionPlan(mission, { source: 'automatic' });
      logger.info(`MissionPlan created id=${plan.id} for automatic mission ${missionId}`);

      await this.editMission({
        id: missionId,
        uav: [...devices.values()].map((d) => d.id),
        planId: plan.id,
        status: MISSION_STATUS.RUNNING,
        mission: mission,
      });
      await this._createTasks(missionId, tasks, devices);
      await taskScheduler.dispatchReady(missionId);

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

  // Device rows by name, for the devices of `tasks` that exist.
  static async _resolveDevices(tasks) {
    const devices = new Map();
    for (const name of new Set(tasks.map((t) => t.device))) {
      const device = await devicesController.getByName(name);
      if (device) devices.set(name, device);
      else logger.warn(`Task device '${name}' not found in DB`);
    }
    return devices;
  }

  // Every task is created (INIT) before any is marked failed: marking one ERROR makes
  // the scheduler skip its dependents, which therefore must already exist. A task whose
  // device doesn't exist is still created, then failed, so its dependents get skipped
  // instead of waiting forever. Returns task rows by task_id.
  static async _createTasks(missionId, tasks, devices) {
    const created = new Map();
    for (const t of tasks) {
      const row = await this.createTask({
        missionId,
        deviceId: devices.get(t.device)?.id ?? null,
        taskKey: t.task_id,
        dependsOn: t.depends_on,
        action: t.action,
        status: TASK_STATUS.INIT,
        initTime: new Date(),
        result: {},
        currentWp: 0,
        totalWp: t.wp.length,
      });
      created.set(t.task_id, row);
    }
    for (const t of tasks) {
      if (devices.has(t.device)) continue;
      await this.editTask({
        id: created.get(t.task_id).id,
        status: TASK_STATUS.ERROR,
        errorMessage: `El dispositivo '${t.device}' no existe`,
      });
    }
    return created;
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
    missionSMModel.deviceSyncedFiles(mydevice.id);
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
    missionSMModel.deviceFinishedMission(mydevice.id);
    return true;
  }

  // Files are downloaded once per device, by its last task in the mission. Same-device
  // tasks are ordered by the graph, so any other open task of this device runs later.
  static async shouldDownloadFiles(taskId) {
    const task = await this.getTask(taskId);
    const deviceTasks = await this.getTasks({ missionId: task.missionId, deviceId: task.deviceId });
    return !deviceTasks.some((t) => t.id !== task.id && !TASK_TERMINAL_STATUS.includes(t.status));
  }

  static async finishTask(taskId) {
    const task = await this.editTask({ id: taskId, status: TASK_STATUS.COMPLETED, endTime: new Date() });
    if (!task) {
      logger.warn(`finishTask: task ${taskId} not found`);
      return false;
    }

    eventsController.addEvent({
      type: 'info',
      deviceId: task.deviceId,
      attributes: { action: 'MissionComplete', message: `Task ${task.taskKey} complete for UAV ${task.deviceId}` },
    });

    const externalId = await this._resolveExternalId(task.missionId);
    await ExtAppController.missionReqResult(externalId, 0);
    return true;
  }

  static async updateTaskFiles(taskId) {
    const task = await this.getTask(taskId);
    const mission = task && (await this.getMissionValue(task.missionId));
    if (!task || !mission) {
      logger.warn(`updateTaskFiles: task ${taskId} or its mission not found in DB, skipping file download`);
      return false;
    }
    await filesController.updateFiles(task.deviceId, task.missionId, task.id, mission.initTime);
    await sleep(5000);
    return true;
  }

  static async endTask(taskId) {
    logger.info(`===== endTask ${taskId} =====`);
    const task = await this.getTask(taskId);
    if (!task) {
      logger.warn(`endTask: task ${taskId} not found`);
      return false;
    }
    const { missionId, deviceId: uavId } = task;
    const listfiles = await filesController.getFilesInfo({ taskId });
    logger.debug(`endTask listfiles: ${JSON.stringify(listfiles)}`);
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
    logger.debug(`endTask attributes: ${JSON.stringify(attributes)}`);
    eventsController.addEvent({
      type: 'info',
      deviceId: uavId,
      attributes: { action: 'MissionEnd', message: `Task ${task.taskKey} ended for UAV ${uavId}` },
    });
    await this.editTask({ id: taskId, status: TASK_STATUS.END, result: attributes, endTime: new Date() });

    if (attributes.hasOwnProperty('measures') && attributes.measures.length > 0) {
      const myMission = await this.getMissionValue(missionId);
      const existingResults = Array.isArray(myMission?.results) ? myMission.results : [];
      await this.editMission({ id: missionId, results: [...existingResults, attributes] });
    }

    const tasks = await this.getTasks({ missionId });
    const flown = tasks.filter((t) => !TASK_FAILED_STATUS.includes(t.status));
    if (flown.length > 0 && flown.every((t) => t.status === TASK_STATUS.END)) {
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
    const tasks = await this.getTasks({ missionId });
    result.data = tasks.filter((t) => t.result).map((t) => ({ deviceId: t.deviceId, result: t.result }));
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
  static async failInterruptedMissions() {
    const aliveMissions = await this.getMissionValue();
    for (const mission of aliveMissions) {
      // Mission first: once it is no longer alive, closing its tasks below can't
      // trigger _checkMissionComplete into overwriting this status/message.
      await this.editMission({
        id: mission.id,
        status: MISSION_STATUS.ERROR,
        errorMessage: 'Misión interrumpida: el servidor se reinició mientras estaba activa',
      });
      const openTasks = await this.getTasks({ missionId: mission.id, status: [TASK_STATUS.INIT, ...TASK_ACTIVE_STATUS] });
      for (const task of openTasks) {
        const neverStarted = task.status === TASK_STATUS.INIT;
        await this.editTask({
          id: task.id,
          status: neverStarted ? TASK_STATUS.SKIPPED : TASK_STATUS.ERROR,
          errorMessage: neverStarted
            ? 'Tarea no iniciada: el servidor se reinició antes de que se ejecutara'
            : 'Tarea interrumpida: el servidor se reinició mientras estaba activa',
        });
      }
    }
  }

  /**
   * Every stored plan is a valid task graph (tasks[] or legacy route[]): validated here
   * so a plan saved straight over HTTP can't fail only later, when it is executed.
   * Throws TaskGraphError (status 400) on an invalid graph.
   */
  static async createMissionPlan(missionData, { name = null, source = 'manual' } = {}) {
    normalizeMission(missionData);
    return await sequelize.models.MissionPlan.create({ missionData: withFormatVersion(missionData), name, source });
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
   * MANUAL flow — load. Validates the task graph, creates MissionPlan + Mission +
   * every task, and loads only the ROOT tasks (no depends_on), each with its one-task
   * mission. Dependents stay INIT: the scheduler loads them once the mission runs.
   * A root that fails to load (after one retry) goes to ERROR, which skips its
   * dependents. Returns { missionId, planId, results } (one result per root).
   * Throws TaskGraphError (status 400) on an invalid graph.
   * @param {object} missionData - { tasks: [...] } or legacy { route: [...] }
   */
  static async loadMissionManual(missionData) {
    logger.info('===== loadMissionManual =====');
    const graph = normalizeMission(missionData);

    const storedMission = withFormatVersion(missionData);
    const plan = await this.createMissionPlan(storedMission, { source: 'manual' });
    logger.info(`loadMissionManual: MissionPlan created id=${plan.id}`);

    const devices = await this._resolveDevices(graph.tasks);
    const listUAV = [...devices.values()].map((d) => d.id);

    // Idempotency: a drone can't have two not-yet-commanded missions at once.
    // Cancel any INIT mission that overlaps with this load's devices before
    // creating the new one, so double-clicks / repeated loads don't pile up
    // orphaned Mission/Task rows or re-send configureMission redundantly.
    await this._cancelStaleInitMissions(listUAV);

    const mission = await this.createMission({
      name: `manual_${plan.id}`,
      planId: plan.id,
      trigger: 'manual',
      uav: listUAV,
      // Loaded, not yet commanded: INIT until commandMissionManual promotes it.
      status: MISSION_STATUS.INIT,
      mission: storedMission,
    });
    logger.info(`loadMissionManual: Mission created id=${mission.id} devices=[${listUAV.join(',')}]`);

    const created = await this._createTasks(mission.id, graph.tasks, devices);

    const results = [];
    for (const task of graph.tasks.filter((t) => t.depends_on.length === 0)) {
      const device = devices.get(task.device);
      if (!device) {
        results.push({ deviceId: null, name: task.device, taskKey: task.task_id, state: 'warning', msg: `device ${task.device} not found` });
        continue;
      }
      let response;
      try {
        response = await withRetry(() =>
          commandsController.sendCommandDevice({
            deviceId: device.id,
            type: 'loadMission',
            attributes: missionForTask(graph, task.task_id),
          })
        );
      } catch (err) {
        response = { state: 'error', msg: err instanceof Error ? err.message : String(err) };
      }
      const ok = response.state !== 'error';
      await this.editTask({
        id: created.get(task.task_id).id,
        status: ok ? TASK_STATUS.LOADED : TASK_STATUS.ERROR,
        errorMessage: ok ? null : `Fallo al cargar la tarea en el dispositivo: ${response.msg ?? 'error desconocido'}`,
      });
      results.push({ deviceId: device.id, name: device.name, taskKey: task.task_id, state: response.state, msg: response.msg });
    }

    // If no root loaded (e.g. the only drone failed), the mission is unusable → ERROR.
    // Otherwise it stays INIT and command will start the roots that did load.
    const anyLoaded = results.some((r) => r.state !== 'error' && r.deviceId != null);
    if (!anyLoaded) {
      await this.editMission({ id: mission.id, status: MISSION_STATUS.ERROR, errorMessage: 'Ninguna tarea se pudo cargar' });
    }

    logger.info(`loadMissionManual finished mission=${mission.id} anyLoaded=${anyLoaded}`);
    return { missionId: mission.id, planId: plan.id, results };
  }

  /**
   * MANUAL flow — command. Tasks already exist (created in load). Commands each
   * LOADED root and promotes it to COMMANDED (so missionWpTracking picks it up), with
   * a state machine attached in RunningMission so it downloads files and frees its
   * device like any dispatched task. A root that can't be commanded goes to ERROR
   * (skipping its dependents). The mission then runs and the scheduler takes over
   * the dependents. Returns { missionId, results } (one result per loaded root).
   * @param {number} missionId
   */
  static async commandMissionManual(missionId) {
    logger.info(`===== commandMissionManual mission=${missionId} =====`);
    // A repeated command (double click) must not touch a mission already running.
    const mission = await this.getMissionValue(missionId);
    if (mission?.status !== MISSION_STATUS.INIT) {
      return { missionId, results: [], state: 'warning', msg: `mission ${missionId} is not awaiting command (status=${mission?.status})` };
    }
    const loaded = await this.getTasks({ missionId, status: TASK_STATUS.LOADED });
    if (loaded.length === 0) {
      await this.editMission({ id: missionId, status: MISSION_STATUS.ERROR, errorMessage: 'Ninguna tarea cargada para comandar' });
      return { missionId, results: [], state: 'warning', msg: `mission ${missionId} has no loaded tasks` };
    }

    const results = [];
    for (const task of loaded) {
      let response;
      try {
        response = await withRetry(() =>
          commandsController.sendCommandDevice({ deviceId: task.deviceId, type: 'commandMission' })
        );
      } catch (err) {
        response = { state: 'error', msg: err instanceof Error ? err.message : String(err) };
      }
      if (response.state !== 'error') {
        await this.editTask({ id: task.id, status: TASK_STATUS.COMMANDED });
        missionSMModel.createActorMission(task.deviceId, missionId, task.id, { alreadyRunning: true });
      } else {
        await this.editTask({
          id: task.id,
          status: TASK_STATUS.ERROR,
          errorMessage: `Fallo al comandar la tarea en el dispositivo: ${response.msg ?? 'error desconocido'}`,
        });
      }
      results.push({ deviceId: task.deviceId, taskKey: task.taskKey, state: response.state, msg: response.msg });
    }

    const anyCommanded = results.some((r) => r.state !== 'error');
    if (anyCommanded) {
      await this.editMission({ id: missionId, status: MISSION_STATUS.RUNNING });
      await taskScheduler.dispatchReady(missionId);
    } else {
      await this.editMission({ id: missionId, status: MISSION_STATUS.ERROR, errorMessage: 'Ninguna tarea se pudo comandar' });
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
      // Per-task edit (not a bulk update) so each row goes through editTask() and
      // emits its update — stale-task volume is low, consistency wins. Terminal tasks
      // (e.g. ERROR on load) keep their status: overwriting it would erase why they failed.
      const openTasks = await this.getTasks({ missionId: stale.id, status: [TASK_STATUS.INIT, ...TASK_ACTIVE_STATUS] });
      for (const task of openTasks) {
        await this.editTask({ id: task.id, status: TASK_STATUS.CANCELLED });
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

missionModel.failInterruptedMissions();

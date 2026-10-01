import { missionModel } from '../models/mission/mission.js';
import {
  validateMissionCollission,
  resolveCollisions as resolveCollisionsAlgo,
  formatMissionReport,
  validateInspectionCoverage,
  formatInspectionReport,
} from '../models/collision/index.js';
import { resolveInspectionTargets } from '../models/markers/inspectionTargets.js';
import { devicesController } from './devices.js';
import { missionLogger as logger } from '../common/logger.js';

/**
 * Build a single unambiguous reason string for the OVERALL verdict, combining
 * collision and inspection-coverage results so the caller doesn't have to
 * infer it from separate per-section status lines.
 */
function buildOverallReason(result, inspection, unknownTargetIds) {
  const reasons = [];
  if (result.totalCollisions > 0) {
    reasons.push(`${result.totalCollisions} collision(s) with obstacles`);
  }
  if (result.interRouteCollisions?.length > 0) {
    reasons.push(`${result.interRouteCollisions.length} inter-UAV conflict(s)`);
  }
  if (inspection?.missing.length > 0) {
    reasons.push(`${inspection.missing.length} target(s) not covered`);
  }
  if (unknownTargetIds.length > 0) {
    reasons.push(`${unknownTargetIds.length} unknown target id(s)`);
  }
  if (reasons.length > 0) {
    return reasons.join('; ');
  }
  return inspection ? 'no collisions, all targets inspected' : 'no collisions';
}

class missionController {
  static getMission = async (req, res) => {
    const response = await missionModel.getMissionValue(req.query.id, req.query.all === 'true');
    res.json(response);
  };

  static createMission = async (req, res) => {
    const response = await missionModel.broadcastMission(req.body);
    res.json(response);
  };

  static getTasks = async (req, res) => {
    const { id, missionId, deviceId, status } = req.query;
    res.json(await missionModel.getTasks({ id, missionId, deviceId, status }));
  };
  static requestMission = async (req, res) => {
    logger.info(`requestMission: ${JSON.stringify(req.body)}`);
    let id = req.body.id || req.body.mission_id;
    let name = req.body.name;
    let objetivo = req.body.objetivo;
    let locations = req.body.locations || req.body.loc;
    let meteo = []; // req.body.meteo;
    for (let i = 0; i < locations.length; i++) {
      locations[i].hasOwnProperty('items') ? null : (locations[i].items = []);
      locations[i].hasOwnProperty('geo_points') ? (locations[i].items = locations[i].geo_points) : null;
      locations[i].hasOwnProperty('geopoints') ? (locations[i].items = locations[i].geopoints) : null;

      for (let j = 0; j < locations[i].items.length; j++) {
        locations[i].items[j].hasOwnProperty('lat')
          ? (locations[i].items[j].latitude = locations[i].items[j].lat)
          : null;
        locations[i].items[j].hasOwnProperty('lon')
          ? (locations[i].items[j].longitude = locations[i].items[j].lon)
          : null;
      }
    }
    logger.debug(`requestMission id=${id}`);
    await missionModel.requestMission({ id, name, objetivo, locations, meteo });
    res.status(200).json('all ok');
  };

  static showMission = async (mission_data) => {
    let response = await missionModel.broadcastMission(mission_data);
    return response;
  };

  static showMissionXYZ = async (req, res) => {
    try {
      const response = await missionModel.showMissionXYZ(req.body);
      res.json(response);
    } catch (error) {
      logger.error(`Error in showMissionXYZ: ${error.message}`);
      res.status(500).json({ error: error.message || 'Failed to show mission XYZ.' });
    }
  };

  // Manual flow — load. Body: the mission ({ tasks: [...] } or legacy { route: [...] }),
  // or wrapped as { mission: {...} }. Returns { missionId, planId, results }.
  static loadMissionManual = async (req, res) => {
    const missionData = req.body?.tasks || req.body?.route ? req.body : (req.body?.mission ?? {});
    try {
      const response = await missionModel.loadMissionManual(missionData);
      res.json(response);
    } catch (error) {
      logger.error(`Error in loadMissionManual: ${error.message}`);
      // TaskGraphError carries status 400 and every validation error.
      res.status(error.status || 500).json({ error: error.message || 'Failed to load mission.', errors: error.errors });
    }
  };

  // Manual flow — command. Body: { missionId }. Returns { missionId, results }.
  static commandMissionManual = async (req, res) => {
    const missionId = req.body?.missionId;
    if (missionId == null) {
      return res.status(400).json({ error: 'missionId is required.' });
    }
    try {
      const mission = await missionModel.getMissionValue(missionId);
      if (!mission) return res.status(404).json({ error: `Mission ${missionId} not found.` });
      const response = await missionModel.commandMissionManual(missionId);
      res.json(response);
    } catch (error) {
      logger.error(`Error in commandMissionManual: ${error.message}`);
      res.status(500).json({ error: error.message || 'Failed to command mission.' });
    }
  };

  static initMission = (mission_id, data, opts) => {
    return missionModel.initMission(mission_id, data, opts);
  };
  static editMission = (payload) => {
    return missionModel.editMission(payload);
  };
  static shouldDownloadFiles = (taskId) => {
    return missionModel.shouldDownloadFiles(taskId);
  };
  static getTask = (taskId) => {
    return missionModel.getTask(taskId);
  };
  static editTask = (payload) => {
    return missionModel.editTask(payload);
  };
  static finishTask = (taskId) => {
    return missionModel.finishTask(taskId);
  };
  static deviceFinishMission = ({ name, id }) => {
    return missionModel.deviceFinishMission({ name, id });
  };
  static deviceFinishSyncFiles = ({ name, id }) => {
    return missionModel.deviceFinishSyncFiles({ name, id });
  };
  static endTask = (taskId) => {
    return missionModel.endTask(taskId);
  };
  static getMissionById = async (missionId) => {
    return await missionModel.getMissionValue(missionId);
  };
  static updateTaskFiles = (taskId) => {
    return missionModel.updateTaskFiles(taskId);
  };
  static updateMission = ({ device, mission, state }) => {
    missionModel.updateMission({ device, mission, state });
    return true;
  };

  static convertGeodeticToXYZ = async (req, res) => {
    const missionBriefing = req.body;
    if (!missionBriefing.selected_devices || !missionBriefing.targets) {
      return res.status(400).json({ error: 'selected_devices and targets are required.' });
    }
    try {
      const missionDataXYZ = await missionModel.convertBriefingToXYZ(missionBriefing);
      res.json(missionDataXYZ);
    } catch (error) {
      logger.error(`Error in convertGeodeticToXYZ: ${error.message}`);
      res.status(error.status || 500).json({ error: error.message });
    }
  };

  static convertXYZToGeodetic = async (req, res) => {
    const missionDataXYZ = req.body;
    if (!Array.isArray(missionDataXYZ.tasks) && !Array.isArray(missionDataXYZ.route)) {
      return res.status(400).json({ error: 'tasks (or legacy route) is required.' });
    }
    try {
      const missionGeodetic = missionModel.convertXYZToGeodetic(missionDataXYZ);
      res.json(missionGeodetic);
    } catch (error) {
      logger.error(`Error in convertXYZToGeodetic: ${error.message}`);
      res.status(500).json({ error: error.message });
    }
  };

  static createMissionPlan = async (req, res) => {
    const { missionData, name, source } = req.body;
    if (!missionData) {
      return res.status(400).json({ error: 'missionData is required.' });
    }
    try {
      const saved = await missionModel.createMissionPlan(missionData, { name, source });
      res.status(201).json({ id: saved.id, name: saved.name, createdAt: saved.createdAt });
    } catch (error) {
      logger.error(`Error in createMissionPlan: ${error.message}`);
      // TaskGraphError carries status 400 and every validation error.
      res.status(error.status || 500).json({ error: error.message, errors: error.errors });
    }
  };

  static getMissionPlans = async (req, res) => {
    const response = await missionModel.getAllMissionPlans();
    res.json(response);
  };

  static getMissionPlanById = async (req, res) => {
    const response = await missionModel.getMissionPlan(req.params.id);
    if (!response) return res.status(404).json({ error: 'MissionPlan not found' });
    res.json(response);
  };

  static showMissionPlan = async (req, res) => {
    const plan = await missionModel.getMissionPlan(req.params.id);
    if (!plan) return res.status(404).json({ error: 'MissionPlan not found' });
    await missionModel.broadcastMission(plan.missionData);
    res.json({ ok: true });
  };

  /**
   * Validate mission for collisions without modifying it. Optionally also
   * validates inspection coverage: pass target_ids (catalog ElementItem ids,
   * NOT positions) and every waypoint gets checked against the target's real
   * position/type resolved server-side from the SQL catalog - the LLM never
   * gets to supply the position that's being checked against.
   * POST /missions/validate
   * Body: { mission: MissionObject, collision_objects: ObstacleArray, target_ids?: (number|string)[] }
   */
  static validateCollisions = async (req, res) => {
    try {
      const { mission, collision_objects, target_ids } = req.body;

      if (!mission) {
        return res.status(400).json({ error: 'Mission data is required' });
      }

      if (!collision_objects || !Array.isArray(collision_objects)) {
        return res.status(400).json({ error: 'collision_objects array is required' });
      }

      for (const route of mission.route ?? []) {
        const device = await devicesController.getByName(route.uav);
        if (!device) {
          return res.status(400).json({ error: `El UAV '${route.uav}' no existe. Por favor, intenta de nuevo.` });
        }
      }

      const result = validateMissionCollission(mission, collision_objects);

      let inspection = null;
      let unknownTargetIds = [];
      if (target_ids && Array.isArray(target_ids) && target_ids.length > 0) {
        if (!mission.global_origin) {
          return res.status(400).json({ error: 'mission.global_origin is required to validate target_ids' });
        }

        const { targets, notFound } = await resolveInspectionTargets(target_ids, mission.global_origin);
        inspection = validateInspectionCoverage(mission, targets, collision_objects);
        unknownTargetIds = notFound;
        if (notFound.length > 0) {
          inspection.valid = false;
        }
      }

      const overallValid = result.valid && (inspection?.valid ?? true);
      const overallReason = buildOverallReason(result, inspection, unknownTargetIds);

      let report =
        `OVERALL: ${overallValid ? '✅ VALID' : '❌ INVALID'} — ${overallReason}\n\n` + formatMissionReport(result);
      if (inspection) {
        report += '\n\n' + formatInspectionReport(inspection);
      }
      if (unknownTargetIds.length > 0) {
        report += `\n\n#### [UNKNOWN TARGET IDS]\n  * These ids don't exist in the catalog: ${unknownTargetIds.join(', ')}`;
      }

      res.json({
        valid: overallValid,
        totalCollisions: result.totalCollisions,
        totalWarnings: result.totalWarnings,
        routes: result.routes,
        inspection,
        report,
      });
    } catch (error) {
      logger.error(`Error validating collisions: ${error.message}`);
      res.status(500).json({ error: error.message || 'Failed to validate collisions' });
    }
  };

  /**
   * Validate and automatically resolve collisions with detours
   * POST /missions/resolve
   * Body: { mission: MissionObject, collision_objects: ObstacleArray }
   */
  static resolveCollisions = async (req, res) => {
    try {
      const { mission, collision_objects } = req.body;

      if (!mission) {
        return res.status(400).json({ error: 'Mission data is required' });
      }

      if (!collision_objects || !Array.isArray(collision_objects)) {
        return res.status(400).json({ error: 'collision_objects array is required' });
      }

      // First validate
      const validation = validateMissionCollission(mission, collision_objects);

      if (validation.valid) {
        return res.json({
          modified: false,
          message: 'Mission is already collision-free',
          mission,
          validation,
        });
      }

      // Resolve collisions
      const result = resolveCollisionsAlgo(mission, collision_objects);

      // Validate the corrected mission
      const finalValidation = validateMissionCollission(result.mission, collision_objects);

      res.json({
        modified: true,
        detoursApplied: result.totalDetoursApplied,
        mission: result.mission,
        report: result.report,
        validation: finalValidation,
      });
    } catch (error) {
      logger.error(`Error resolving collisions: ${error.message}`);
      res.status(500).json({ error: error.message || 'Failed to resolve collisions' });
    }
  };
}

export { missionController };

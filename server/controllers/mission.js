import { missionModel } from '../models/mission/mission.js';
import {
  validateMissionCollission,
  resolveCollisions as resolveCollisionsAlgo,
  formatMissionReport,
} from '../models/collision/index.js';
import { missionLogger as logger } from '../common/logger.js';

class missionController {
  static getMission = async (req, res) => {
    const response = await missionModel.getMissionValue(req.query.id, req.query.all === 'true');
    res.json(response);
  };

  static createMission = async (req, res) => {
    const response = await missionModel.broadcastMission(req.body);
    res.json(response);
  };

  static getRoutes = async (req, res) => {
    logger.debug('getRoutes');
    const response = await missionModel.getRoutes(req.query);
    res.json(Object.values(response));
  };
  static sendTask = async (req, res) => {
    logger.info(`sendTask: ${JSON.stringify(req.body)}`);
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
    logger.debug(`sendTask id=${id}`);
    let response = await missionModel.sendTask({ id, name, objetivo, locations, meteo });
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

  // Manual flow — load. Body: { route: [...] } (a mission's routes).
  // Returns { missionId, planId, results }.
  static loadMissionManual = async (req, res) => {
    const missionData = req.body?.route ? req.body : { route: req.body?.mission?.route ?? [] };
    if (!Array.isArray(missionData.route) || missionData.route.length === 0) {
      return res.status(400).json({ error: 'route is required and must be a non-empty array.' });
    }
    try {
      const response = await missionModel.loadMissionManual(missionData);
      res.json(response);
    } catch (error) {
      logger.error(`Error in loadMissionManual: ${error.message}`);
      res.status(500).json({ error: error.message || 'Failed to load mission.' });
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
  static editRoute = (payload) => {
    return missionModel.editRoute(payload);
  };
  static finishMission = (missionId, deviceId) => {
    return missionModel.UAVFinish(missionId, deviceId);
  };
  static deviceFinishMission = ({ name, id }) => {
    return missionModel.deviceFinishMission({ name, id });
  };
  static deviceFinishSyncFiles = ({ name, id }) => {
    return missionModel.deviceFinishSyncFiles({ name, id });
  };
  static endRouteUAV = (missionId, uavId) => {
    return missionModel.UAVEnd(missionId, uavId);
  };
  static getMissionRoute = async (missionId) => {
    return await missionModel.getMissionValue(missionId);
  };
  static updateFiles = (missionId, deviceId, routeId) => {
    return missionModel.updateFiles(missionId, deviceId, routeId);
  };
  static updateMission = ({ device, mission, state }) => {
    missionModel.updateMission({ device, mission, state });
    return true;
  };

  static convertGeodeticToXYZ = async (req, res) => {
    const missionBriefing = req.body;
    if (!missionBriefing.target_elements || !missionBriefing.drone_information) {
      return res.status(400).json({ error: 'target_elements and drone_information are required.' });
    }
    try {
      const missionDataXYZ = missionModel.convertBriefingToXYZ(missionBriefing);
      res.json(missionDataXYZ);
    } catch (error) {
      logger.error(`Error in convertGeodeticToXYZ: ${error.message}`);
      res.status(500).json({ error: error.message });
    }
  };

  static convertXYZToGeodetic = async (req, res) => {
    const missionDataXYZ = req.body;
    if (!missionDataXYZ.route) {
      return res.status(400).json({ error: 'route is required.' });
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
      res.status(500).json({ error: error.message });
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
   * Validate mission for collisions without modifying it
   * POST /missions/validate
   * Body: { mission: MissionObject, collision_objects: ObstacleArray }
   */
  static validateCollisions = async (req, res) => {
    try {
      const { mission, collision_objects } = req.body;

      if (!mission) {
        return res.status(400).json({ error: 'Mission data is required' });
      }

      if (!collision_objects || !Array.isArray(collision_objects)) {
        return res.status(400).json({ error: 'collision_objects array is required' });
      }

      const result = validateMissionCollission(mission, collision_objects);
      const report = formatMissionReport(result);

      res.json({
        valid: result.valid,
        totalCollisions: result.totalCollisions,
        totalWarnings: result.totalWarnings,
        routes: result.routes,
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

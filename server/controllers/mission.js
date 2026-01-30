import { missionModel } from '../models/mission.js';
import {
  validateMission,
  resolveCollisions as resolveCollisionsAlgo,
  formatCollisionReport,
} from '../models/collision/index.js';

class missionController {
  static getMission = async (req, res) => {
    const response = await missionModel.getMissionValue(req.query.id);
    res.json(response);
  };

  static createMission = async (req, res) => {
    const response = await missionModel.setMission(req.body);
    res.json(response);
  };

  static getRoutes = async (req, res) => {
    console.log('get routes');
    const response = await missionModel.getRoutes(req.query);
    res.json(Object.values(response));
  };
  static sendTask = async (req, res) => {
    console.log('======== send task ========');
    console.log(req.body);
    let id = req.body.id || req.body.mission_id;
    let name = req.body.name;
    let objetivo = req.body.objetivo;
    let locations = req.body.locations || req.body.loc;
    console.log(locations);
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
        console.log(locations[i].items[j]);
      }
    }
    console.log('id: ', id);
    let response = await missionModel.sendTask({ id, name, objetivo, locations, meteo });
    res.status(200).json('all ok');
  };

  static showMission = async (mission_data) => {
    let response = await missionModel.setMission(mission_data);
    return response;
  };

  static showMissionXYZ = async (req, res) => {
    try {
      const response = await missionModel.showMissionXYZ(req.body);
      res.json(response);
    } catch (error) {
      console.error('Error in showMissionXYZ:', error);
      res.status(500).json({ error: error.message || 'Failed to show mission XYZ.' });
    }
  };

  static initMission = (mission_id, data) => {
    missionModel.initMission(mission_id, data);
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
  static finishMissionProcessFiles = (missionId, deviceId, results) => {
    return missionModel.FinishProcessFiles((missionId, deviceId, results));
  };
  static getMissionRoute = async (missionId) => {
    return await missionModel.getMissionValue(missionId);
  };
  static updateFiles = (missionId, deviceId) => {
    return missionModel.updateFiles(missionId, deviceId);
  };
  static updateMission = ({ device, mission, state }) => {
    missionModel.updateMission({ device, mission, state });
    return true;
  };

  /**
   * Validate mission for collisions without modifying it
   * POST /missions/validate
   * Body: { mission: MissionObject, obstacles: ObstacleArray }
   */
  static validateCollisions = async (req, res) => {
    try {
      const { mission, obstacles } = req.body;

      if (!mission) {
        return res.status(400).json({ error: 'Mission data is required' });
      }

      if (!obstacles || !Array.isArray(obstacles)) {
        return res.status(400).json({ error: 'Obstacles array is required' });
      }

      const result = validateMission(mission, obstacles);
      const report = result.routes.map((r) => formatCollisionReport(r)).join('\n\n');

      res.json({
        valid: result.valid,
        totalCollisions: result.totalCollisions,
        totalWarnings: result.totalWarnings,
        routes: result.routes,
        report,
      });
    } catch (error) {
      console.error('Error validating collisions:', error);
      res.status(500).json({ error: error.message || 'Failed to validate collisions' });
    }
  };

  /**
   * Validate and automatically resolve collisions with detours
   * POST /missions/resolve
   * Body: { mission: MissionObject, obstacles: ObstacleArray }
   */
  static resolveCollisions = async (req, res) => {
    try {
      const { mission, obstacles } = req.body;

      if (!mission) {
        return res.status(400).json({ error: 'Mission data is required' });
      }

      if (!obstacles || !Array.isArray(obstacles)) {
        return res.status(400).json({ error: 'Obstacles array is required' });
      }

      // First validate
      const validation = validateMission(mission, obstacles);

      if (validation.valid) {
        return res.json({
          modified: false,
          message: 'Mission is already collision-free',
          mission,
          validation,
        });
      }

      // Resolve collisions
      const result = resolveCollisionsAlgo(mission, obstacles);

      // Validate the corrected mission
      const finalValidation = validateMission(result.mission, obstacles);

      res.json({
        modified: true,
        detoursApplied: result.totalDetoursApplied,
        mission: result.mission,
        report: result.report,
        validation: finalValidation,
      });
    } catch (error) {
      console.error('Error resolving collisions:', error);
      res.status(500).json({ error: error.message || 'Failed to resolve collisions' });
    }
  };
}

export { missionController };

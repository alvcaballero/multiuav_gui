import { missionModel } from '../models/mission.js';
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

  static setMission = async (req, res) => {
    let response = await missionModel.setMission(req.body);
    res.json(response);
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
}

export { missionController };

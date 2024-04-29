var MissionController;
class missionController {
  constructor({ model }) {
    this.missionModel = model;
  }
  getMission = async (req, res) => {
    console.log('get missions');
    const response = await this.missionModel.getMissionValue(req.query.id);
    console.log(response);
    res.json(response);
  };
  getRoutes = async (req, res) => {
    console.log('get routes');
    console.log(req.query);
    const response = await this.missionModel.getRoutes(req.query);
    res.json(Object.values(response));
  };
  sendTask = async (req, res) => {
    console.log('======== send task ========');
    console.log(req.body);
    let id = req.body.id || req.body.mission_id;
    let name = req.body.name;
    let objetivo = req.body.objetivo;
    let locations = req.body.locations || req.body.loc;
    console.log(locations);
    let meteo = req.body.meteo;
    for (let i = 0; i < locations.length; i++) {
      locations[i].hasOwnProperty('geo_points') ? (locations[i]['items'] = locations[i].geo_points) : null;
      locations[i].hasOwnProperty('geopoints') ? (locations[i]['items'] = locations[i].geopoints) : null;

      for (let j = 0; j < locations[i].items.length; j++) {
        locations[i].items[j].hasOwnProperty('lat')
          ? (locations[i].items[j].latitude = locations[i].items[j].lat)
          : null;
        locations[i].items[j].hasOwnProperty('lon')
          ? (locations[i].items[j].longitude = locations[i].items[j].lon)
          : null;
      }
    }
    let response = await this.missionModel.sendTask({ id, name, objetivo, locations, meteo });

    res.json(response);
  };
  setMission = async (req, res) => {
    let response = await this.missionModel.setMission(req.body);
    res.json(response);
  };

  updateFiles = async (req, res) => {
    console.log('updates Files');
    let response = await this.missionModel.updateFiles(req.params);
    res.json(response);
  };

  showFiles = async (req, res) => {
    console.log('show files');
    let response = await this.missionModel.showFiles(req.params);
    res.json(response);
  };

  listFiles = async (req, res) => {
    console.log('list files');
    let response = await this.missionModel.listFiles(req.params);
    res.json(response);
  };

  initMission = (mission_id, data) => {
    this.missionModel.initMission(mission_id, data);
  };
  finishMission = (missionId, deviceId) => {
    return this.missionModel.UAVFinish(missionId, deviceId);
  };
  finishMissionProcessFiles = (missionId, deviceId, results) => {
    return this.missionModel.FinishProcessFiles((missionId, deviceId, results));
  };
  getMissionRoute = async (missionId) => {
    return await this.missionModel.getMissionValue(missionId);
  };
  updateFiles = (missionId, deviceId) => {
    return this.missionModel.updateFiles(missionId, deviceId);
  };
  updateMission = ({ device, mission, state }) => {
    this.missionModel.updateMission({ device, mission, state });
    return true;
  };
}

function CreateController({ model }) {
  MissionController = new missionController({ model });
  return MissionController;
}
export { MissionController, missionController, CreateController };

var MissionController;
class missionController {
  constructor({ model }) {
    this.missionModel = model;
  }
  getmission = async (req, res) => {
    console.log('get missions');
    const response = await this.missionModel.getmissionValue(req.query.id);
    res.json(Object.values(response));
  };
  getRoutes = async (req, res) => {
    console.log('get routes');
    console.log(req.query);
    const response = await this.missionModel.getRoutes(req.query);
    res.json(Object.values(response));
  };
  sendTask = async (req, res) => {
    let response = await this.missionModel.sendTask(req.body);
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
  getMissionRoute = (missionId) => {
    return this.missionModel.getmissionValue(missionId);
  };
  getCurrentMission = (mission_id) => {
    return this.missionModel.getmission(mission_id);
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

import { planningModel } from '../models/planning.js';

export class planningController {
  static async getTypes(req, res) {
    const response = await planningModel.getTypes();
    res.json(response);
  }
  static async getParam(req, res) {
    const response = await planningModel.getParam(req.params.type);
    res.json(response);
  }
  static async getMissionTypes(req, res) {
    const response = await planningModel.getMissionTypes();
    res.json(response);
  }
  static async getDefault(req, res) {
    let response = await planningModel.getDefault();
    res.json(response);
  }
  static async getDefaultPlanning() {
    return await planningModel.getDefault();
  }

  static async setDefault(req, res) {
    let response = await planningModel.setDefault(req.body);
    res.json(response);
  }

  static getConfigParam(obj) {
    return planningModel.getParam(obj);
  }
  static async getConfigBases() {
    return await planningModel.getBases();
  }
  static async getBasesSettings() {
    return await planningModel.getBasesSettings();
  }
  static getCaseTypes() {
    return planningModel.getTypes();
  }
  static async PlanningRequest(req) {
    return await planningModel.PlanningRequest(req);
  }
  static async fetchPlanning(req) {
    return await planningModel.fetchPlanning(req);
  }
  static async localPlanning(req) {
    return await planningModel.localPlanning(req);
  }
}

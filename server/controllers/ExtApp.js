import { ExtApp } from '../models/ExtApp.js';

export class ExtAppController {
  static async UpdateToken(req, res) {
    let response = await ExtApp.UpdateToken();
    res.json(response);
  }
  static async missionStart(req, res) {
    let response = await ExtApp.missionStart(req.body.mission_id, req.body);
    res.json(response);
  }
  static async missionResult(req, res) {
    let response = await ExtApp.missionResult(req.body.mission_id, req.body.resolution_code);
    res.json(response);
  }
  static async missionMedia(req, res) {
    let response = await ExtApp.missionMedia(req.body.mission_id, req.body.results);
    res.json(response);
  }
  // These *Req* methods are the internal (server → external) callbacks. They receive
  // the EXTERNAL task id (already translated from the internal PK by missionModel),
  // because ExtApp must address the external system by the id it assigned.
  static missionReqStart(externalId, mission) {
    ExtApp.missionStart(externalId, mission);
  }
  static async missionReqResult(externalId, resultCode) {
    return await ExtApp.missionResult(externalId, resultCode);
  }
  static missionReqMedia(externalId, attributes) {
    ExtApp.missionMedia(externalId, attributes);
  }
}

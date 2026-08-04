import { markersModel } from '../../models/markers/markers.js';

export class markersController {
  // ─── Instances ────────────────────────────────────────────────────────────

  static async getMarkers(req, res) {
    const response = await markersModel.getMarkers();
    res.json(response);
  }

  static async setMarkers(req, res) {
    try {
      const response = await markersModel.setMarkers(req.body);
      res.json(response);
    } catch (error) {
      res.status(500).json({ error: error.message });
    }
  }

  static async getBases(req, res) {
    const { markersbase } = await markersModel.getMarkers();
    res.json(markersbase);
  }

  static async getElements(req, res) {
    const { elements } = await markersModel.getMarkers();
    res.json(elements);
  }

  static async getBasesWithAssignments(req, res) {
    const response = await markersModel.getBaseswithAssignments();
    res.json(response);
  }
}

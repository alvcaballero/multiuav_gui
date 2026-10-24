import { deviceModelsModel } from '../models/deviceModels/deviceModels.js';

export class deviceModelsController {
  // ?category=dji_M300 → only the models a device of that category can be, its default first.
  static getAll(req, res) {
    res.json(deviceModelsModel.list({ category: req.query.category }));
  }

  // The capability catalog (config/devices/capabilities.yaml): key → { label, description, commands, slots }.
  static getCapabilities(req, res) {
    res.json(deviceModelsModel.getCapabilities());
  }

  static get(req, res) {
    const model = deviceModelsModel.get(req.params.model);
    if (!model) return res.status(404).json({ error: `unknown device model '${req.params.model}'` });
    res.json({ name: req.params.model, ...model });
  }
}

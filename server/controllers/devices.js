import { DevicesModel } from '../models/devices.js';
import { validateDevice, validatePartialDevice } from '../schemas/devices.js';
export class devicesController {
  static async getAll(req, res) {
    console.log('controller get all');
    const devices = await DevicesModel.getAll();
    res.json(devices);
  }

  static async getById(req, res) {
    const { id } = req.params;
    const device = await DevicesModel.getById({ id });
    if (device) return res.json(device);
    res.status(404).json({ message: 'device not found' });
  }

  static async create(req, res) {
    const result = validateDevice(req.body);

    if (!result.success) {
      // 422 Unprocessable Entity
      return res.status(400).json({ error: JSON.parse(result.error.message) });
    }

    const newDevice = await DevicesModel.create({ input: result.data });

    res.status(201).json(newDevice);
  }

  static async delete(req, res) {
    const { id } = req.params;

    const result = await DevicesModel.delete({ id });

    if (result === false) {
      return res.status(404).json({ message: 'device not found' });
    }

    return res.json({ message: 'device deleted' });
  }

  static async update(req, res) {
    const result = validatePartialDevice(req.body);

    if (!result.success) {
      return res.status(400).json({ error: JSON.parse(result.error.message) });
    }

    const { id } = req.params;

    const updatedDevice = await DevicesModel.update({ id, input: result.data });

    return res.json(updatedDevice);
  }

  static async GetDeviceCategory(req, res) {
    return await DevicesModel.GetdevicesCategory();
  }
}

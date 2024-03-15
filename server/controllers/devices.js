import { DevicesModel } from '../models/devices.js';
import { validateDevice, validatePartialDevice } from '../schemas/devices.js';
export class devicesController {
  static async getAll(req, res) {
    console.log('controller get all');
    const devices = await DevicesModel.getAll(req.query.id);
    res.json(Object.values(devices));
  }
  static async getAllDevices() {
    const devices = await DevicesModel.getAll();
    return devices;
  }
  static getbyName(name) {
    const device = DevicesModel.getByName(name);
    return device;
  }

  static async getById(req, res) {
    const { id } = req.params;
    const device = await DevicesModel.getById({ id });
    if (device) return res.json(device);
    res.status(404).json({ message: 'device not found' });
  }

  static async create(req, res) {
    console.log('create new device');
    console.log(req.body);
    const result = validateDevice(req.body);

    if (!result.success) {
      // 422 Unprocessable Entity
      res.status(400).json({ error: JSON.parse(result.error.message) });
    }

    const newDevice = await DevicesModel.create(result.data);

    res.status(201).json(newDevice);
  }

  static async delete(req, res) {
    const { id } = req.params;
    console.log('delete device ' + id);
    //console.log(req.params);

    const result = await DevicesModel.delete({ id });

    if (result === false) {
      res.status(404).json({ message: 'device not found' });
    }

    res.json({ message: 'device deleted' });
  }

  static async update(req, res) {
    const result = validatePartialDevice(req.body);

    if (!result.success) {
      res.status(400).json({ error: JSON.parse(result.error.message) });
    }

    const { id } = req.params;

    const updatedDevice = await DevicesModel.update({ id, input: result.data });

    res.json(updatedDevice);
  }
}

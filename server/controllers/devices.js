import { validateDevice, validatePartialDevice } from '../schemas/zod/devices.js';
import { DevicesModel } from '../models/devices.js';

class devicesController {
  static getAll = async (req, res) => {
    const devices = await DevicesModel.getAll();
    res.json(Object.values(devices));
  };
  static getAllDevices = async () => {
    //console.log('get all devices controller');
    const devices = await DevicesModel.getAll();
    return devices;
  };
  static getDevice = async (id) => {
    const devices = await DevicesModel.getAll(id);
    return Array.isArray(devices) ? devices.at() : devices;
  };
  static getByName = async (name) => {
    const device = await DevicesModel.getByName(name);
    return device;
  };

  static getAccess = async (id) => {
    return await DevicesModel.getAccess(id);
  };

  static getById = async (req, res) => {
    const { id } = req.params;
    const device = await DevicesModel.getById({ id });
    if (device) return res.json(device);
    res.status(404).json({ message: 'device not found' });
  };

  static create = async (req, res) => {
    console.log('create new device');
    console.log(req.body);
    const result = validateDevice(req.body);

    if (!result.success) {
      // 422 Unprocessable Entity
      res.status(400).json({ error: JSON.parse(result.error.message) });
    }

    const newDevice = await DevicesModel.create(result.data);

    res.status(201).json(newDevice);
  };

  static delete = async (req, res) => {
    const { id } = req.params;
    console.log('delete device ' + id);
    //console.log(req.params);

    const result = await DevicesModel.delete({ id });

    if (result === false) {
      res.status(404).json({ message: 'device not found' });
    }

    res.json({ message: 'device deleted' });
  };

  static update = async (req, res) => {
    const result = validatePartialDevice(req.body);

    if (!result.success) {
      res.status(400).json({ error: JSON.parse(result.error.message) });
    }

    const { id } = req.params;
    result.data.id = id;
    console.log(result.data);

    const updatedDevice = await DevicesModel.editDevice(result.data);

    res.json(updatedDevice);
  };
}

export { devicesController };

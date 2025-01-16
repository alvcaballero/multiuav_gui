import { validateDevice, validatePartialDevice } from '../schemas/devices.js';

var DevicesController;
class devicesController {
  constructor({ model }) {
    this.DevicesModel = model;
  }
  getAll = async (req, res) => {
    const devices = await this.DevicesModel.getAll(req.query.id);
    res.json(Object.values(devices));
  };
  getAllDevices = async () => {
    //console.log('get all devices controller');
    const devices = await this.DevicesModel.getAll();
    return devices;
  };
  getDevice = async (id) => {
    const devices = await this.DevicesModel.getAll(id);
    return devices['id'];
  };
  getByName = async (name) => {
    const device = await this.DevicesModel.getByName(name);
    return device;
  };
  updateDeviceTime = (id) => {
    this.DevicesModel.updateDeviceTime(id);
  };

  getAccess = async (id) => {
    return await this.DevicesModel.getAccess(id);
  };

  getById = async (req, res) => {
    const { id } = req.params;
    const device = await this.DevicesModel.getById({ id });
    if (device) return res.json(device);
    res.status(404).json({ message: 'device not found' });
  };

  create = async (req, res) => {
    console.log('create new device');
    console.log(req.body);
    const result = validateDevice(req.body);

    if (!result.success) {
      // 422 Unprocessable Entity
      res.status(400).json({ error: JSON.parse(result.error.message) });
    }

    const newDevice = await this.DevicesModel.create(result.data);

    res.status(201).json(newDevice);
  };

  delete = async (req, res) => {
    const { id } = req.params;
    console.log('delete device ' + id);
    //console.log(req.params);

    const result = await this.DevicesModel.delete({ id });

    if (result === false) {
      res.status(404).json({ message: 'device not found' });
    }

    res.json({ message: 'device deleted' });
  };

  update = async (req, res) => {
    const result = validatePartialDevice(req.body);

    if (!result.success) {
      res.status(400).json({ error: JSON.parse(result.error.message) });
    }

    const { id } = req.params;
    result.data.id = id;
    console.log(result.data);

    const updatedDevice = await this.DevicesModel.editDevice(result.data);

    res.json(updatedDevice);
  };
}

function CreateController({ model }) {
  DevicesController = new devicesController({ model });
  return DevicesController;
}

export { DevicesController, devicesController, CreateController };
//export const DevicesController = new devicesController()

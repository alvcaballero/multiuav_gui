import { validateDevice, validatePartialDevice } from '../schemas/zod/devices.js';
import { DevicesModel } from '../models/devices.js';
import { categoryModel } from '../models/category.js';
import { logger } from '../common/logger.js';
import { telemetryWatchdog } from '../models/inspect/telemetryWatchdog.js';

class devicesController {
  static getAll = async (req, res) => {
    const { id } = req.query;
    // Soporta tanto `id=1&id=2` (Express arma un array) como `id=1,2,3` (CSV).
    const parsedIds = id
      ? (Array.isArray(id) ? id : [id])
          .flatMap((value) => String(value).split(','))
          .map((value) => Number(value.trim()))
          .filter((value) => !Number.isNaN(value))
      : undefined;
    const devices = await DevicesModel.getAll(parsedIds);
    res.json(Object.values(devices));
  };
  static getAllDevices = async () => {
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
  static getBySystemId = async (systemId) => {
    return await DevicesModel.getBySystemId(systemId);
  };

  static getDevicesWithPositions = async (req, res) => {
    const summary = await DevicesModel.getDevicesWithPositions();
    res.json(summary);
  };

  static getAccess = async (id) => {
    return await DevicesModel.getAccess(id);
  };

  static getFilesConfig = async (id) => {
    return await DevicesModel.getFilesConfig(id);
  };

  static getById = async (req, res) => {
    const { id } = req.params;
    const device = await DevicesModel.getById({ id });
    if (device) return res.json(device);
    res.status(404).json({ message: 'device not found' });
  };

  static create = async (req, res) => {
    const result = validateDevice(req.body);

    if (!result.success) {
      return res.status(400).json({ error: JSON.parse(result.error.message) });
    }
    // zod checks the shape; whether the category can work with the protocol needs the category
    // config, and schemas do not import models.
    const problem = categoryModel.deviceProblem(result.data);
    if (problem) return res.status(400).json({ error: [problem] });

    const newDevice = await DevicesModel.create(result.data);
    res.status(201).json(newDevice);
  };

  static delete = async (req, res) => {
    const { id } = req.params;
    logger.info(`Deleting device id=${id}`);

    const result = await DevicesModel.delete({ id });

    if (result === false) {
      return res.status(404).json({ message: 'device not found' });
    }

    res.json({ message: 'device deleted' });
  };

  static update = async (req, res) => {
    logger.info('Updating device');

    const result = validatePartialDevice(req.body);

    if (!result.success) {
      return res.status(400).json({ error: JSON.parse(result.error.message) });
    }
    const { id } = req.params;
    result.data.id = id;

    // A partial edit is checked on the device it LEAVES: switching only the protocol to mavlink,
    // on a device with no systemId, must fail too.
    const current = await DevicesModel.getById({ id });
    if (!current) return res.status(404).json({ message: 'device not found' });
    const problem = categoryModel.deviceProblem({
      protocol: result.data.protocol ?? current.protocol ?? 'ros',
      category: result.data.category ?? current.category,
      systemId: result.data.systemId !== undefined ? result.data.systemId : current.systemId,
    });
    if (problem) return res.status(400).json({ error: [problem] });

    const updatedDevice = await DevicesModel.editDevice(result.data);

    res.json(updatedDevice);
  };

  static getSnapshot = async (req, res) => {
    const { id } = req.params;

    if (!id) {
      return res.status(400).json({ message: 'Device id is required.' });
    }

    let device;
    try {
      device = await DevicesModel.getById({ id });
    } catch (err) {
      logger.error(`getSnapshot: DB error for id=${id}: ${err.message}`);
      return res.status(500).json({ message: 'Error retrieving device.' });
    }

    if (!device) {
      return res.status(404).json({ message: `Device with id=${id} not found.` });
    }

    const hasCamera = device.camera && device.camera.length > 0;
    if (!hasCamera) {
      return res.status(422).json({ message: `Device '${device.name}' has no camera configured.` });
    }

    try {
      const { cameraModel } = await import('../models/camera.js');
      const snapshot = await cameraModel.getSnapshot(device);

      if (snapshot) {
        res.writeHead(200, {
          'Content-Type': snapshot.mimeType,
          'Content-Length': snapshot.buffer.length,
        });
        return res.end(snapshot.buffer);
      }
    } catch (err) {
      logger.error(`getSnapshot: capture error for device '${device.name}': ${err.message}`);
      return res.status(500).json({ message: 'Snapshot capture failed.' });
    }

    res.status(404).json({ message: 'No snapshot available. Ensure stream is active.' });
  };

  // Per-slot rate, age and health from the telemetry watchdog (in memory, no DB): `{}` for a
  // device that sends nothing, which is OFFLINE rather than an unknown id.
  static getTelemetry = async (req, res) => {
    res.json(telemetryWatchdog.snapshotOf(Number(req.params.id)));
  };
}

export { devicesController };

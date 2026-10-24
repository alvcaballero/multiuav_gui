import { validateDevice, validatePartialDevice, validateDeviceAttributes, pruneAttributes } from '../schemas/zod/devices.js';
import { DevicesModel } from '../models/devices.js';
import { categoryModel } from '../models/category.js';
import { deviceModelsModel } from '../models/deviceModels/deviceModels.js';
import { logger } from '../common/logger.js';
import { telemetryWatchdog } from '../models/inspect/telemetryWatchdog.js';

/**
 * A MAVLink vehicle is found by its systemId, so two devices sharing one would both be ignored by
 * the server. Needs the DB, which is why it is not in `categoryModel.deviceProblem`.
 * @param {number|string|null} [selfId] the device being edited, which may keep its own systemId
 * @returns {Promise<{path: string[], message: string}|null>}
 */
const systemIdTaken = async ({ protocol, attributes }, selfId = null) => {
  const systemId = attributes?.systemId;
  if (protocol !== 'mavlink' || systemId == null) return null;
  const others = ((await DevicesModel.getBySystemId(systemId)) ?? []).filter((d) => String(d.id) !== String(selfId));
  if (!others.length) return null;
  return { path: ['attributes', 'systemId'], message: `systemId ${systemId} is already used by '${others.map((d) => d.name).join("', '")}'` };
};

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
  /** @returns {Promise<object|null>} the device, or null when `id` matches none */
  static getDevice = async (id) => {
    if (id == null) return null;
    // getAll(query) answers EVERY device when `id` is not a number (or is 0): an exact match here
    // keeps a bad id from resolving to whichever device comes first.
    const devices = await DevicesModel.getAll(id);
    return devices.find((device) => String(device.id) === String(id)) ?? null;
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
    // config, and schemas do not import models. A device created without a model (MCP, ExtApp, an
    // old client) gets its category's default.
    const device = { ...result.data, deviceModel: result.data.deviceModel ?? deviceModelsModel.defaultFor(result.data.category) };
    const problem =
      categoryModel.deviceProblem(device) ??
      deviceModelsModel.deviceProblem(device) ??
      deviceModelsModel.capabilityOverrideProblem(device) ??
      (await systemIdTaken(device));
    if (problem) return res.status(400).json({ error: [problem] });
    result.data = device;

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
    const leaving = {
      protocol: result.data.protocol ?? current.protocol ?? 'ros',
      category: result.data.category ?? current.category,
      deviceModel: result.data.deviceModel ?? current.deviceModel,
    };
    // Stored attributes that no longer apply to the device it becomes (e.g. mavlink's after a protocol
    // change) are dropped; the ones sent in this edit are merged on top and must all apply.
    const kept = pruneAttributes(leaving, current.attributes);
    // A new model has its own capabilities: the old unit's switches do not carry over unless sent again.
    if (leaving.deviceModel !== current.deviceModel) delete kept.capabilities;
    const attributes = validateDeviceAttributes(leaving, { ...kept, ...result.data.attributes });
    if (!attributes.success) {
      const error = attributes.error.issues.map((issue) => ({ ...issue, path: ['attributes', ...issue.path] }));
      return res.status(400).json({ error });
    }
    leaving.attributes = attributes.data;
    // A category change keeps the model unless one is sent: if it does not fit the new category, the
    // edit says so instead of leaving a device that is something its interface cannot drive.
    const problem =
      categoryModel.deviceProblem(leaving) ??
      (leaving.deviceModel ? deviceModelsModel.deviceProblem(leaving) : null) ??
      deviceModelsModel.capabilityOverrideProblem(leaving) ??
      (await systemIdTaken(leaving, id));
    if (problem) return res.status(400).json({ error: [problem] });
    result.data.attributes = attributes.data;

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

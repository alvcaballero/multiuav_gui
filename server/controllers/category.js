import { categoryModel, CategoryError } from '../models/category.js';
import { deviceModelsModel } from '../models/deviceModels/deviceModels.js';
import { devicesController } from './devices.js';
import { logger } from '../common/logger.js';

// Writes go through the same validator as the boot (models/interfaces/interfaceCatalog.js), BEFORE
// the file is written: an invalid body is a 400 { error, errors: [{path, message}] } and never
// reaches disk.
async function respondWrite(res, write) {
  try {
    res.json((await write()) ?? null);
  } catch (err) {
    if (!(err instanceof CategoryError)) throw err;
    res.status(err.status).json({ error: err.message, errors: err.errors });
  }
}

// The mission getters (actions, params, waypoint params…) answer for an interface (`:type`) or, with
// `?deviceId=`, for ONE device: its interface narrowed to what it can do (capabilities, vehicle type;
// docs/decisions/vehicle-platforms.md › F5 · Contenido de la misión). Then the device's own interface
// is used, not `:type`. An unknown device is a 404, never the unfiltered answer.
async function withMissionScope(req, res, answer) {
  const { deviceId } = req.query;
  if (deviceId == null || deviceId === '') return res.json(await answer({ type: req.params.type, restrict: null }));
  const device = await devicesController.getDevice(deviceId);
  if (!device) return res.status(404).json({ error: `device ${deviceId} not found` });
  return res.json(await answer({ type: device.category, restrict: deviceModelsModel.missionRestrictionOf(device) }));
}

export class categoryController {
  static async getAll(req, res) {
    logger.debug('Getting all categories');
    // ?protocol=mavlink → only the categories a device of that protocol can use.
    // ?deviceModel=x500 → only the ones that model lists (its `interfaces`): the device form picks
    //   the model first, then one of its interfaces, then a protocol that carries it.
    // ?details=true → [{ name, root, messageFormat, firmwares, vehicleTypes, control }] instead of names.
    const filter = { protocol: req.query.protocol };
    let response = req.query.details === 'true' ? categoryModel.getAllDetailed(filter) : categoryModel.getAll(filter);
    if (req.query.deviceModel) {
      const listed = deviceModelsModel.interfacesOf(req.query.deviceModel);
      response = response.filter((it) => listed.includes(it.name ?? it));
    }
    res.json(response);
  }
  static async getCategory(req, res) {
    logger.debug(`Getting category: ${req.params.category}`);
    const response = await categoryModel.getCategory(req.params.category);
    res.json(response);
  }
  // What the file says (own entries + `extends`): what the category editor loads and saves back.
  static async getCategoryRaw(req, res) {
    const response = categoryModel.getCategoryRaw(req.params.category);
    if (response === undefined) return res.status(404).json({ error: `unknown category '${req.params.category}'` });
    res.json(response);
  }
  static async updateCategory(req, res) {
    logger.info(`Updating category: ${req.params.category}`);
    await respondWrite(res, () => categoryModel.updateCategory(req.params.category, req.body));
  }
  static async createCategory(req, res) {
    logger.info(`Creating category: ${req.params.category}`);
    await respondWrite(res, () => categoryModel.createCategory(req.params.category, req.body));
  }
  static async deleteCategory(req, res) {
    logger.info(`Deleting category: ${req.params.category}`);
    await respondWrite(res, () => categoryModel.deleteCategory(req.params.category));
  }
  static async messagesTypes(req, res) {
    logger.debug('Getting message types');
    const response = await categoryModel.getMessagesType();
    res.json(response);
  }

  static async getAtributes(req, res) {
    logger.debug(`Getting attributes for device type: ${req.params.type}`);
    await withMissionScope(req, res, ({ type, restrict }) => categoryModel.getAtributes(type, restrict));
  }

  static async getAtributesParam(req, res) {
    logger.debug(`Getting attribute parameters for: ${JSON.stringify(req.params)}`);
    await withMissionScope(req, res, ({ type, restrict }) =>
      categoryModel.getAtributesParam({ type, param: req.params.param }, restrict)
    );
  }

  static async getActions(req, res) {
    logger.debug(`Getting actions for: ${JSON.stringify(req.params)}`);
    await withMissionScope(req, res, ({ type, restrict }) => categoryModel.getActions({ type }, restrict));
  }
  static async getActionsParam(params) {
    logger.debug(`Getting action parameters for: ${JSON.stringify(params)}`);
    return await categoryModel.getActions(params);
  }

  static async getAttributesList(req, res) {
    logger.debug(`Getting attributes list for: ${req.params.type}`);
    await withMissionScope(req, res, ({ type, restrict }) => categoryModel.getAttributesList(type, restrict));
  }

  static async getWaypointParams(req, res) {
    logger.debug(`Getting waypoint params for: ${req.params.type}`);
    await withMissionScope(req, res, ({ type, restrict }) => categoryModel.getWaypointParams(type, restrict));
  }

  static async getAttributesDefaults(req, res) {
    logger.debug(`Getting attributes defaults for: ${req.params.type}`);
    await withMissionScope(req, res, ({ type, restrict }) => categoryModel.getAttributesDefaults(type, restrict));
  }
}

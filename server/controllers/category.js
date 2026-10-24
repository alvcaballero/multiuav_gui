import { categoryModel, CategoryError } from '../models/category.js';
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

export class categoryController {
  static async getAll(req, res) {
    logger.debug('Getting all categories');
    // ?protocol=mavlink → only the categories a device of that protocol can use.
    // ?details=true → [{ name, root, firmwares, vehicleTypes, control }] instead of names.
    const filter = { protocol: req.query.protocol };
    const response = req.query.details === 'true' ? categoryModel.getAllDetailed(filter) : categoryModel.getAll(filter);
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
    let response = await categoryModel.getAtributes(req.params.type);
    res.json(response);
  }

  static async getAtributesParam(req, res) {
    logger.debug(`Getting attribute parameters for: ${JSON.stringify(req.params)}`);
    let response = await categoryModel.getAtributesParam(req.params);
    res.json(response);
  }

  static async getActions(req, res) {
    logger.debug(`Getting actions for: ${JSON.stringify(req.params)}`);
    let response = await categoryModel.getActions(req.params);
    res.json(response);
  }
  static async getActionsParam(params) {
    logger.debug(`Getting action parameters for: ${JSON.stringify(params)}`);
    return await categoryModel.getActions(params);
  }

  static async getAttributesList(req, res) {
    logger.debug(`Getting attributes list for: ${req.params.type}`);
    const response = await categoryModel.getAttributesList(req.params.type);
    res.json(response);
  }

  static async getWaypointParams(req, res) {
    logger.debug(`Getting waypoint params for: ${req.params.type}`);
    const response = await categoryModel.getWaypointParams(req.params.type);
    res.json(response);
  }

  static async getAttributesDefaults(req, res) {
    logger.debug(`Getting attributes defaults for: ${req.params.type}`);
    const response = await categoryModel.getAttributesDefaults(req.params.type);
    res.json(response);
  }
}

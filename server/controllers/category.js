import { categoryModel } from '../models/category.js';
import { logger } from '../common/logger.js';

export class categoryController {
  static async getAll(req, res) {
    logger.debug('Getting all categories');
    const response = await categoryModel.getAll();
    res.json(response);
  }
  static async getCategory(req, res) {
    logger.debug(`Getting category: ${req.params.category}`);
    const response = await categoryModel.getCategory(req.params.category);
    res.json(response);
  }
  static async updateCategory(req, res) {
    logger.info(`Updating category: ${req.params.category}`);
    const response = await categoryModel.updateCategory(req.params.category, req.body);
    res.json(response);
  }
  static async createCategory(req, res) {
    logger.info(`Creating category: ${req.params.category}`);
    const response = await categoryModel.createCategory(req.params.category, req.body);
    res.json(response);
  }
  static async deleteCategory(req, res) {
    logger.info(`Deleting category: ${req.params.category}`);
    const response = await categoryModel.deleteCategory(req.params.category);
    res.json(response);
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

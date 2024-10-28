import { categoryModel } from '../models/category.js';

export class categoryController {
  static async getAll(req, res) {
    console.log('category get all');
    const response = await categoryModel.getAll();
    res.json(response);
  }
  static async getCategory(req, res) {
    console.log('category get');
    const response = await categoryModel.getCategory(req.params.category);
    res.json(response);
  }
  static async updateCategory(req, res) {
    console.log('category update');
    const response = await categoryModel.updateCategory(req.params.category, req.body);
    res.json(response);
  }
  static async createCategory(req, res) {
    console.log('category create');
    const response = await categoryModel.createCategory(req.params.category, req.body);
    res.json(response);
  }
  static async deleteCategory(req, res) {
    console.log('category delete');
    const response = await categoryModel.deleteCategory(req.params.category);
    res.json(response);
  }
  static async messagesTypes(req, res) {
    console.log('messages types');
    const response = await categoryModel.getMessagesType();
    res.json(response);
  }

  static async getAtributes(req, res) {
    console.log('device Category');
    let response = await categoryModel.getAtributes(req.params.type);
    res.json(response);
  }

  static async getAtributesParam(req, res) {
    console.log('device Category');
    let response = await categoryModel.getAtributesParam(req.params);
    res.json(response);
  }

  static async getActions(req, res) {
    console.log('device Category');
    let response = await categoryModel.getActions(req.params);
    res.json(response);
  }
  static async getActionsParam(params) {
    console.log('device Category');
    return await categoryModel.getActions(params);
  }
}

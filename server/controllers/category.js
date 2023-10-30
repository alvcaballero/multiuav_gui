import { categoryModel } from '../models/category.js';

export class categoryController {
  static async getAll(req, res) {
    console.log('category get all');
    const response = await categoryModel.getAll();
    res.json(response);
  }
  static async getAtributes(req, res) {
    console.log('device Category');
    let response = await categoryModel.getAtributes(req.params.type);
    res.json(response)  
}

  static async getAtributesParam(req, res) {
    console.log('device Category');
    let response = await categoryModel.getAtributesParam(req.params);
    res.json(response) 
  }

  static async getActions(req, res) {
    console.log('device Category');
    let response = await categoryModel.getActions(req.params);
    res.json(response) 
  }
}


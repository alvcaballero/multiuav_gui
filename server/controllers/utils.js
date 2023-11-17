import { utilsModel } from '../models/utils.js';

export class utilsController {
  static async getDateTime(req, res) {
    console.log('controller get all');
    let response = await utilsModel.DateTime();
    res.json(response);
  }
}

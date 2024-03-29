//import { filesModel } from '../models/files.js';
import { filesModel } from '../models/files-sql.js';
export class filesController {
  static async getfiles(req, res) {
    console.log('controller get foles');
    let response = await filesModel.getfiles();
    res.json(response);
  }
  static async testfile(req, res) {
    let response = await filesModel.testMetadata(req.body.src);
    res.json(response);
  }
  static async MetadataTempImage(req, res) {
    let response = await filesModel.MetadataTempImage(req.body.src);
    res.json(response);
  }
  static async ProcessThermalImages(req, res) {
    let response = await filesModel.ProcessThermalImages(req.body.src);
    res.json(response);
  }

  static async donwload(req, res) {
    //https://www.geeksforgeeks.org/how-to-download-a-file-using-express-js/
    //https://medium.com/@imajeet5/how-to-serve-files-using-node-js-d99de4653a3
    console.log('controller donwload');
    let response = await filesModel.donwload(req.params.filename);
    console.log(response);
    if (response) {
      res.download(response, function (err) {
        if (err) {
          console.log(err);
          res.send({
            error: err,
            msg: 'Problem downloading the file',
          });
        }
      });
    } else {
      res.statusMessage = 'Path no match';
      res.status(400).end();
    }
  }
}

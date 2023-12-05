import { filesModel } from '../models/files.js';

export class filesController {
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

import { serverModel } from '../models/server.js';

export class serverController {
  static async server(req, res) {
    let response = await serverModel.Serverconfig();
    res.json(response);
  }
  static async getDateTime(req, res) {
    let response = await serverModel.DateTime();
    res.json(response);
  }
  static async getServerProtocol(req, res) {
    let response = await serverModel.Protocol();
    res.json(response);
  }
  static async ServerProtocol() {
    let response = await serverModel.Protocol();
    return response;
  }
  static async donwload(req, res) {
    //https://www.geeksforgeeks.org/how-to-download-a-file-using-express-js/
    //https://medium.com/@imajeet5/how-to-serve-files-using-node-js-d99de4653a3
    console.log('resources  donwload');
    try {
      let response = await serverModel.checkFileRoute(req.params.filename);
      console.log(response);
      if (response) {
        res.download(response, function (err) {
          if (err) {
            console.error('Error during file download:', err);
            if (!res.headersSent) {
              res.send({
                error: err,
                msg: 'Problem downloading the file',
              });
            }
          }
        });
      } else {
        if (!res.headersSent) {
          res.statusMessage = 'Path no match';
          res.status(400).end();
        }
      }
    } catch (error) {
      console.error('Unexpected error:', error);
      if (!res.headersSent) {
        res.status(500).send({
          error: error.message,
          msg: 'Unexpected error occurred',
        });
      }
    }
  }
}

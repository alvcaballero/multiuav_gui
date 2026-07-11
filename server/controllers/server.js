import { serverModel } from '../models/server.js';
import { logger } from '../common/logger.js';

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
    logger.info(`Downloading resource: ${req.params.filename}`);
    try {
      let response = await serverModel.checkFileRoute(req.params.filename);
      logger.debug(`File route resolved to: ${response}`);
      if (response) {
        res.download(response, function (err) {
          if (err) {
            logger.error(`Error during file download: ${err}`);
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
      logger.error(`Unexpected error during resource download: ${error.message}`);
      if (!res.headersSent) {
        res.status(500).send({
          error: error.message,
          msg: 'Unexpected error occurred',
        });
      }
    }
  }
}

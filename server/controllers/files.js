import { filesModel } from '../models/files.js';
import logger from '../common/logger.js';

class filesController {
  static getFiles = async (req, res) => {
    logger.debug('Getting files');
    let response = await filesModel.getFiles(req.query);
    res.json(response);
  };
  static getFilesInfo = async (request) => {
    logger.debug('Getting files info');
    let response = await filesModel.getFiles(request);
    return response;
  };
  static listFiles = async (req, res) => {
    logger.debug('Listing GCS files');
    let response = await filesModel.readGCSFiles();
    res.json(response);
  };

  static MetadataTempImage = async (req, res) => {
    let response = await filesModel.MetadataTempImage(req.body.src);
    res.json(response);
  };
  static ProcessThermalImages = async (req, res) => {
    let response = await filesModel.ProcessThermalImages(req.body.src);
    res.json(response);
  };
  static updateFiles = async (uavId, missionId, routeId, initTime) => {
    return await filesModel.updateFiles(uavId, missionId, routeId, initTime);
  };
  static updateFilesAPI = async (req, res) => {
    logger.info('Updating files');
    const { uavId, missionId, routeId, initTime } = req.params;
    let response = await filesModel.updateFiles(uavId, missionId, routeId, initTime);
    res.json(response);
  };
  static showFiles = async (req, res) => {
    logger.debug('Showing files');
    let response = await filesModel.showFiles(req.params);
    res.json(response);
  };

  static donwload = async (req, res) => {
    logger.info(`Downloading file: ${req.params.filename}`);
    try {
      let filePath = await filesModel.checkFileRoute(req.params.filename);
      if (filePath) {
        res.download(filePath, function (err) {
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
      logger.error(`Unexpected error during file download: ${error.message}`);
      if (!res.headersSent) {
        res.status(500).send({
          error: error.message,
          msg: 'Unexpected error occurred',
        });
      }
    }
  };
}

export { filesController };

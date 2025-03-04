import { filesModel } from '../models/files.js';
class filesController {
  static getFiles = async (req, res) => {
    console.log('controller get files');
    let response = await filesModel.getFiles(req.query);
    res.json(response);
  };
  static getFilesInfo = async (request) => {
    console.log('controller get files');
    let response = await filesModel.getFiles(request);
    return response;
  };
  static listFiles = async (req, res) => {
    console.log('controller get list files');
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
    console.log(' update files');
    const { uavId, missionId, routeId, initTime } = req.params;
    let response = await filesModel.updateFiles(uavId, missionId, routeId, initTime);
    res.json(response);
  };
  static showFiles = async (req, res) => {
    console.log('show files');
    let response = await filesModel.showFiles(req.params);
    res.json(response);
  };

  static donwload = async (req, res) => {
    //https://www.geeksforgeeks.org/how-to-download-a-file-using-express-js/
    //https://medium.com/@imajeet5/how-to-serve-files-using-node-js-d99de4653a3
    console.log('controller donwload file' + req.params.filename);
    try {
      let filePath = await filesModel.checkFileRoute(req.params.filename);
      if (filePath) {
        res.download(filePath, function (err) {
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
  };
}

export { filesController };

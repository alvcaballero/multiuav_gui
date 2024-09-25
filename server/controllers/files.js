var FilesController;
class filesController {
  constructor({ model }) {
    this.filesModel = model;
  }
  getFiles = async (req, res) => {
    console.log('controller get files');
    let response = await this.filesModel.getFiles(req.query);
    res.json(response);
  };
  getFilesInfo = async (request) => {
    console.log('controller get files');
    let response = await this.filesModel.getFiles(request);
    res.json(response);
  };
  listFiles = async (req, res) => {
    console.log('controller get list files');
    let response = await this.filesModel.readGCSFiles();
    res.json(response);
  };

  MetadataTempImage = async (req, res) => {
    let response = await this.filesModel.MetadataTempImage(req.body.src);
    res.json(response);
  };
  ProcessThermalImages = async (req, res) => {
    let response = await this.filesModel.ProcessThermalImages(req.body.src);
    res.json(response);
  };
  updateFiles = async (uavId, missionId, routeId, initTime) => {
    return await this.filesModel.updateFiles(uavId, missionId, routeId, initTime);
  };
  updateFilesAPI = async (req, res) => {
    console.log(' update files');
    const { uavId, missionId, routeId, initTime } = req.params;
    let response = await this.filesModel.updateFiles(uavId, missionId, routeId, initTime);
    res.json(response);
  };
  showFiles = async (req, res) => {
    console.log('show files');
    let response = await this.filesModel.showFiles(req.params);
    res.json(response);
  };

  donwload = async (req, res) => {
    //https://www.geeksforgeeks.org/how-to-download-a-file-using-express-js/
    //https://medium.com/@imajeet5/how-to-serve-files-using-node-js-d99de4653a3
    console.log('controller donwload');
    let response = await this.filesModel.donwload(req.params.filename);
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
  };
}

function CreateController({ model }) {
  FilesController = new filesController({ model });
  return FilesController;
}
export { FilesController, filesController, CreateController };

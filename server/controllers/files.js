var FilesController;
class filesController {
  constructor({ model }) {
    this.filesModel = model;
  }
  getfiles = async (req, res) => {
    console.log('controller get foles');
    let response = await this.filesModel.getfiles();
    res.json(response);
  };
  testfile = async (req, res) => {
    let response = await this.filesModel.testMetadata(req.body.src);
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
  updateFiles = async (uavid, missionid, initTime) => {
    return await this.filesModel.updateFiles(uavid, missionid, initTime);
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

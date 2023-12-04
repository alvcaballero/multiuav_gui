import { filesModel } from '../models/files.js';

export class filesController {
  static async donwload(req, res) {
    //https://www.geeksforgeeks.org/how-to-download-a-file-using-express-js/
    //https://medium.com/@imajeet5/how-to-serve-files-using-node-js-d99de4653a3
    console.log('controller donwload');
    let response = await filesModel.donwload();
        res.download(folderPath+'/single_gfg.txt', function(err) {
        if(err) {
            console.log(err);
        }
    })
  }
}

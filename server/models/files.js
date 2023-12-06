//example that a men manage databases
import * as fs from 'fs';
const filespath = '/home/arpa/GCS_media/';
export class filesModel {
  static getfiles() {
    let response = [];
    let firstFiles = fs.readdirSync(filespath, { withFileTypes: true });
    let missionFolder = firstFiles.filter((myroute) => myroute.isDirectory());
    for (let mymission of missionFolder) {
      var stats = fs.statSync(filespath + mymission.name + '/');
      console.log('last time create in secons' + stats.mtime);
      let secondFiles = fs.readdirSync(filespath + mymission.name + '/', { withFileTypes: true });
      let uavfolder = secondFiles.filter((myroute) => myroute.isDirectory());
      for (let myuav of uavfolder) {
        let thirdFiles = fs.readdirSync(filespath + mymission.name + '/' + myuav.name + '/', {
          withFileTypes: true,
        });
        let uavfiles = thirdFiles.filter((myroute) => myroute.isFile());
        for (let myfile of uavfiles) {
          response.push(mymission.name + '/' + myuav.name + '/' + myfile.name);
        }
      }
    }
    return response;
  }
  static donwload(path) {
    let dir = filespath + path.replaceAll('-', '/');
    console.log(dir);
    if (!fs.existsSync(dir)) {
      console.log('no exist ' + dir);
      return null;
    }

    // check if file exist
    return dir;
  }
}

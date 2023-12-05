import * as fs from 'fs';
const filespath = '/home/arpa/GCS_media/';
export class filesModel {
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

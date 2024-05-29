//example that a men manage databases
import * as fs from 'fs';
//import { exec } from 'child_process';
import child_process from 'child_process';
import util from 'util';
import sharp from 'sharp';
import exif from 'exif-reader';

const exec = util.promisify(child_process.exec);

import { dateString, addTime, GetLocalTime } from '../common/utils.js';
import { SFTPClient } from '../common/SFTPClient.js';
import { DevicesController } from '../controllers/devices.js';
import { filesPath, processThermalImg, processThermalsSrc } from '../config/config.js';

// const  files// id , route ,name, uav, date, attributes
const files = {
  1: {
    id: 1,
    routeId: 1,
    missionId: 72510181,
    deviceId: '14',
    name: 'DJI_20240305122904_0010_WIDE.jpg',
    route: '/mission_72510181/uav_14/',
    date: '2024-03-05T12:29:04',
    attributes: {
      latitude: 37.09241,
      longitude: -5.232,
      measures: [],
    },
  },
  2: {
    id: 2,
    routeId: 1,
    missionId: 72510181,
    deviceId: '14',
    name: 'DJI_20240305122904_0010_THRM_process.jpg',
    route: 'mission_72510181/uav_14/',
    date: '2024-03-05T12:29:04',
    attributes: {
      latitude: 37.09241,
      longitude: -5.232,
      measures: [
        {
          name: 'MaxTemp',
          value: 83.7,
        },
      ],
    },
  },
  3: {
    id: 3,
    routeId: 1,
    missionId: 72510181,
    deviceId: '14',
    name: 'distances.pcd',
    route: 'mission_72510181/uav_14/',
    date: '2024-03-05T12:29:04',
    attributes: {
      latitude: 37.09241,
      longitude: -5.232,
      measures: [
        {
          name: 'minDist',
          value: 10,
        },
      ],
    },
  },
};
const sftconections = {}; // manage connection to drone
const filestodownload = []; //manage files that fail download// list of objects,with name, and fileroute, number of try.

export class filesModel {
  static getFiles({ deviceId, missionId, routeId }) {
    if (deviceId) {
      return Object.values(files).filter((word) => word.deviceId == deviceId);
    }
    if (missionId) {
      return Object.values(files).filter((word) => word.missionId == missionId);
    }
    if (routeId) {
      return Object.values(files).filter((word) => word.routeId == routeId);
    }
    return Object.values(files);
  }

  /*
  / read all files in the gcs in folder GCS_MEDIA , and return a list of files
  */

  static readGCSFiles() {
    let response = [];
    let firstFiles = fs.readdirSync(filesPath, { withFileTypes: true });
    let missionFolder = firstFiles.filter((myroute) => myroute.isDirectory());
    for (let mymission of missionFolder) {
      var stats = fs.statSync(filesPath + mymission.name + '/');
      console.log('last time create in secons' + stats.mtime);
      let secondFiles = fs.readdirSync(filesPath + mymission.name + '/', { withFileTypes: true });
      let uavfolder = secondFiles.filter((myroute) => myroute.isDirectory());
      for (let myuav of uavfolder) {
        let thirdFiles = fs.readdirSync(filesPath + mymission.name + '/' + myuav.name + '/', {
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

  /*
  / donwload a file from gcs, if the file exist return the path, if not return null
  */

  static donwload(path) {
    let dir = filesPath + path.replaceAll('-', '/');
    console.log(dir);
    if (!fs.existsSync(dir)) {
      console.log('no exist ' + dir);
      return null;
    }
    return dir;
  }

  /* 
  / Show list of files in the drone, from folder uav_media,
  */
  static async paramsConnection({ mydevice }) {
    console.log(mydevice);
    let myurl = 'sftp://' + mydevice.user + ':' + mydevice.pwd + '@' + mydevice.ip; //sftp://user:password@host
    const parsedURL = new URL(myurl);
    let port = parsedURL.port || 22;
    if (!mydevice.ip.includes('10.42.')) {
      console.log('port 21');
      port = 21;
    }
    const { host, username, password } = parsedURL;
    return { host, port, username, password };
  }

  static async showFiles({ uavId, missionId, initTime }) {
    let mydevice = await DevicesController.getAccess(uavId);
    const { host, port, username, password } = await this.paramsConnection({ mydevice });
    console.log('host ' + host + ' port ' + port + ' user ' + username + ' pass ' + password);
    const client = new SFTPClient();
    await client.connect({ host, port, username, password });
    let listFiles = await client.listFiles('./uav_media/', '^mission_', 'd', true); // que devuelva un  lista de objetos con la fecha de creacion
    let nameFolder = null;
    for (const myfiles of listFiles) {
      var matches = myfiles.match(/^mission_(\d{4})-(\d{2})-(\d{2})_(\d{2}):(\d{2})$/);
      if (matches) {
        console.log(matches[0]);
        nameFolder = myfiles;
        let folderdate = myfiles.slice(8).replace('_', 'T') + ':00';
        let currentdate = new Date(folderdate);
        console.log(folderdate + ' date  ' + currentdate.toJSON() + ' gcs=' + initTime.toJSON());
        if (initTime && currentdate > initTime) {
          console.log('folder mayor');
        }
        break;
      }
    }
    if (nameFolder) {
      listFiles = await client.listFiles('./uav_media/' + nameFolder + '/', '.jpg$', '-', true);
    }
    await client.disconnect();
    return listFiles;
  }

  /*
   / Update files in the drone, from folder uav_media, and download to the server (GCS)
   / if the file is a thermal image, process the image and return the metadata
   / return a list of files, and a list of metadata 
   */
  static async updateFiles(uavId, missionId, routeId, initTime) {
    console.log('update files ' + uavId + '-' + missionId);
    let listImages = [];
    let metadataResponse = { value: {}, imageMetaData: [] };

    let mydevice = await DevicesController.getAccess(uavId);
    console.log(mydevice);
    if (!mydevice.hasOwnProperty('user')) {
      console.log('no have ftp ');
      return false;
    }
    const { host, port, username, password } = await this.paramsConnection({ mydevice });
    console.log('host' + host + ' port ' + port + ' user ' + username + ' pass ' + password);
    const client = new SFTPClient();
    let isConnected = await client.connect({ host, port, username, password });
    if (!isConnected) {
      console.log('cant connect to device ');
      return listImages;
    }

    let listFolders = await client.listFiles('./uav_media/', '^mission_', 'd', true); // que devuelva un  lista de objetos con la fecha de creacion
    let nameFolder = null;
    console.log('select folder');
    let myInitTime = dateString(GetLocalTime(initTime)).replace(/-|:|\s/g, '_');

    for (const myfiles of listFolders) {
      //console.log([myfiles] + '' + 'mission_' + myInitTime);
      if (myfiles === 'mission_' + myInitTime) {
        nameFolder = myfiles;
        console.log('find folder 1');
      }
    }
    console.log('find folder ' + nameFolder);
    if (nameFolder) {
      let dir = `${filesPath}mission_${missionId}/${mydevice.name}`;
      let dir2 = `mission_${missionId}/${mydevice.name}`;

      if (!fs.existsSync(dir)) {
        console.log('no exist ' + dir);
        fs.mkdirSync(dir, { recursive: true });
      }

      let listFiles = await client.listFiles('./uav_media/' + nameFolder + '/', '.jpg$', '-', true);
      console.log('list files in uav');
      console.log(listFiles);
      let downloadOk = true;
      for (let myfile of listFiles) {
        console.log('download file' + myfile);
        let response = await client.downloadFile(`./uav_media/${nameFolder}/${myfile}`, `${dir}/${myfile}`);
        console.log(response);
        let fileId = Math.max(...Object.keys(files).map((key) => Number(key))) + 1;
        files[fileId] = {
          id: fileId,
          routeId: routeId,
          missionId: missionId,
          deviceId: uavId,
          name: myfile,
          route: `${dir2}/`,
          date: new Date(),
          attributes: {},
        };
        filestodownload.push({
          name: myfile,
          source: `./uav_media/${nameFolder}/${myfile}`,
          dist: `${dir}/${myfile}`,
          ref: `${dir2}/${myfile}`,
          status: response.status,
        });
        if (!response.status) {
          downloadOk = false;
        }
      }

      //* Close the connection
      await client.disconnect();
      console.log(filestodownload);

      if (downloadOk) {
        console.log('download OK');
        let listImagestoprocess = filestodownload.filter((image) => image.dist.includes('THRM'));
        if (processThermalImg && listImagestoprocess.length > 0) {
          let listThermal = await this.ProcessThermalImages(listImagestoprocess);
          console.log('listThermal');
          console.log(listThermal);
          for (const themalFile of listThermal) {
            let fileId = Math.max(...Object.keys(files).map((key) => Number(key))) + 1;
            files[fileId] = {
              id: fileId,
              routeId: routeId,
              missionId: missionId,
              deviceId: uavId,
              name: themalFile.name,
              route: `${dir2}/`,
              date: new Date(),
              attributes: {},
            };
          }
          console.log(files);
          metadataResponse = await this.MetadataTempImage(listThermal);
          console.log('metadata response');
          console.log(metadataResponse);
          console.log('add attributes to files');
          metadataResponse.imageMetaData.map((image) => {
            let myfile = Object.values(files).find((item) => item.name == image.name);
            if (myfile) {
              files[myfile.id].attributes = image.attributes;
            }
            console.log(myfile);
          });

          listImages = filestodownload.map((image) => image.ref);
          for (const srcImage of listThermal) {
            listImages.push(srcImage.ref);
          }
        }
      }
    }
    console.log(listImages);
    return { files: listImages, data: [metadataResponse.value] };
  }

  /*
  / return the metadata from a image
  */
  static async testMetadata(srcImage) {
    console.log(' test image');
    let metadata = await sharp(srcImage).metadata();
    let response1 = exif(metadata.exif);
    console.log(response1.GPSInfo.GPSLatitude[2] + '+' + response1.GPSInfo.GPSLongitude[2]);
    console.log(response1.GPSInfo);
    console.log(response1.Image.DateTime);

    let mystring = response1.Photo.UserComment.toString('utf8').replace(/\u0000/g, '');
    let response = JSON.parse(mystring.slice(7).trim());
    console.log(response);
    console.log(mystring);

    console.log(response);

    return response;
  }

  /*
  / return metadata from a list of images thermal images
  */
  static async MetadataTempImage(listThermal) {
    console.log(listThermal);
    console.log('metadata imagen');
    let MissionResponse = { measures: [{ name: 'TempMax', value: 0 }] };
    let latitude;
    let longitude;
    let imageMetaData = [];
    for (const srcImage of listThermal) {
      let metadata = await sharp(srcImage.dist).metadata();
      let dataexitf = exif(metadata.exif);
      let mystring = dataexitf.Photo.UserComment.toString('utf8').replace(/\u0000/g, '');
      let userdata = JSON.parse(mystring.slice(7).trim());
      let GPSPosition = dataexitf.GPSInfo;

      latitude = this.convertDMSToDD(
        GPSPosition.GPSLatitude[0],
        GPSPosition.GPSLatitude[1],
        GPSPosition.GPSLatitude[2],
        GPSPosition.GPSLatitudeRef
      );
      longitude = this.convertDMSToDD(
        GPSPosition.GPSLongitude[0],
        GPSPosition.GPSLongitude[1],
        GPSPosition.GPSLongitude[2],
        GPSPosition.GPSLongitudeRef
      );
      imageMetaData.push({
        name: srcImage.name,
        attributes: {
          latitude,
          longitude,
          measures: [{ name: 'TempMax', value: userdata.MaxTemp }],
        },
      });
      if (Object.keys(MissionResponse).length == 0) {
        MissionResponse.latitude = latitude;
        MissionResponse.longitude = longitude;
        MissionResponse.measures[0].value = userdata.MaxTemp;
      } else {
        if (Number(userdata.MaxTemp) > Number(MissionResponse.measures[0].value)) {
          MissionResponse.latitude = latitude;
          MissionResponse.longitude = longitude;
          MissionResponse.measures[0].value = userdata.MaxTemp;
        }
      }
    }
    return { value: MissionResponse, imageMetaData: imageMetaData };
  }

  static getNormalSize({ width, height, orientation }) {
    return (orientation || 0) >= 5 ? { width: height, height: width } : { width, height };
  }

  /*
  / convert from DMS to DD
  */

  static convertDMSToDD(degrees, minutes, seconds, direction) {
    var dd = degrees + minutes / 60 + seconds / (60 * 60);
    if (direction == 'S' || direction == 'W') {
      dd = dd * -1; // Convert to negative if south or west
    }
    return dd;
  }

  static async ProcessThermalImages(Images2process) {
    console.log('last images process');
    console.log(Images2process);
    const listImages = [];
    //const listImagesdist = [];
    for (const image of Images2process) {
      console.log('process img' + image.ref);
      listImages.push({
        name: image.name.slice(0, -4) + '_process.jpg',
        ref: image.ref.slice(0, -4) + '_process.jpg',
        dist: image.dist.slice(0, -4) + '_process.jpg',
      });
      //listImagesdist.push(image.dist.slice(0, -4) + '_process.jpg');
      try {
        const { stdout, stderr } = await exec(
          `conda run -n DJIThermal ${processThermalsSrc} -i "${image.dist}" -o "${image.dist.slice(
            0,
            -4
          )}_process.jpg" `
        );
        console.log('stdout:', stdout);
        console.log('stderr:', stderr);
      } catch (e) {
        console.error(e); // should contain code (exit code) and signal (that caused the termination).
      }
      //exec(
      //  `conda run -n DJIThermal ${ProcessSRC} -i "${image.dist}" -o "${image.dist.slice(
      //    0,
      //    -4
      //  )}_process.jpg" `,
      //  (error, stdout, stderr) => {
      //    if (error) {
      //      console.log(`error: ${error.message}`);
      //      return;
      //    }
      //    if (stderr) {
      //      console.log(`stderr: ${stderr}`);
      //      return;
      //    }
      //    console.log(`stdout: ${stdout}`);
      //  }
      //);
    }
    console.log('finish process');
    return listImages; //{ ref: listImages, dist: listImagesdist };
  }

  static async listFiles({ uavId, missionId }) {
    console.log('devices acction ' + uavId);
    return [];
  }
}

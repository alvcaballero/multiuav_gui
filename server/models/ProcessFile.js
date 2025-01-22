import child_process from 'child_process';
import sharp from 'sharp';
import exif from 'exif-reader';
import util from 'util';
import { processThermalImg, processThermalsSrc } from '../config/config.js';

const exec = util.promisify(child_process.exec);

function getNormalSize({ width, height, orientation }) {
  return (orientation || 0) >= 5 ? { width: height, height: width } : { width, height };
}

/*
/ convert from DMS to DD
*/

function convertDMSToDD(degrees, minutes, seconds, direction) {
  var dd = degrees + minutes / 60 + seconds / (60 * 60);
  if (direction == 'S' || direction == 'W') {
    dd = dd * -1; // Convert to negative if south or west
  }
  return dd;
}

/*
  / return metadata from a list of images thermal images
  */

export async function getMetadata(path) {
  console.log('metadata imagen');
  let latitude;
  let longitude;
  let measures = [];

  let metadata = await sharp(path).metadata();

  let dataexitf = exif(metadata.exif);

  if (!dataexitf.hasOwnProperty('GPSInfo')) return { latitude, longitude, measures };

  let GPSPosition = dataexitf.GPSInfo;

  latitude = convertDMSToDD(
    GPSPosition.GPSLatitude[0],
    GPSPosition.GPSLatitude[1],
    GPSPosition.GPSLatitude[2],
    GPSPosition.GPSLatitudeRef
  );
  longitude = convertDMSToDD(
    GPSPosition.GPSLongitude[0],
    GPSPosition.GPSLongitude[1],
    GPSPosition.GPSLongitude[2],
    GPSPosition.GPSLongitudeRef
  );
  if (!dataexitf.Photo.hasOwnProperty('UserComment')) return { latitude, longitude, measures };

  let mystring = dataexitf.Photo.UserComment.toString('utf8').replace(/\u0000/g, '');
  let userdata = JSON.parse(mystring.slice(7).trim());

  if (userdata.hasOwnProperty('MinTemp')) measures.push({ name: 'TempMin', value: userdata.MinTemp });
  if (userdata.hasOwnProperty('MaxTemp')) measures.push({ name: 'TempMax', value: userdata.MaxTemp });
  if (userdata.hasOwnProperty('DistVeg')) measures.push({ name: 'DistVeg', value: userdata.DistVeg });
  if (userdata.hasOwnProperty('DistFle')) measures.push({ name: 'DistFle', value: userdata.DistFle });

  return { latitude, longitude, measures };
}

export async function ProcessThermalImage(input, output) {
  console.log('process thermal image' + input);
  if (!processThermalImg) return false;
  //conda run -n DJIThermal
  console.log('last images process');
  console.log('process img' + input + ' ' + output);
  try {
    const { stdout, stderr } = await exec(
      ` ${processThermalsSrc} -i "${input}" -o "${output}" `, { shell: '/bin/bash' }
    );
    console.log('stdout:', stdout);
    console.log('stderr:', stderr);
  } catch (e) {
    console.error(e);
    return false;
  }
  console.log('finish process');
  return true;
}

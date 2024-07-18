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
  let mystring = dataexitf.Photo.UserComment.toString('utf8').replace(/\u0000/g, '');
  let userdata = JSON.parse(mystring.slice(7).trim());
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
  if (userdata.MinTemp) measures.push({ name: 'TempMin', value: userdata.MinTemp });
  if (userdata.MaxTemp) measures.push({ name: 'TempMax', value: userdata.MaxTemp });
  if (userdata.DistVeg) measures.push({ name: 'DistVeg', value: userdata.DistVeg });
  if (userdata.DistFle) measures.push({ name: 'DistFle', value: userdata.DistFle });

  return { latitude, longitude, measures };
}

export async function ProcessThermalImage(input, output) {
  console.log('last images process');
  console.log('process img' + input + ' ' + output);
  try {
    const { stdout, stderr } = await exec(
      `conda run -n DJIThermal ${processThermalsSrc} -i "${input}" -o "${output}" `
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

import child_process from 'child_process';
import sharp from 'sharp';
import exif from 'exif-reader';
import util from 'util';
import { processThermalImg, processThermalScript } from '../config/config.js';
import { logger } from '../common/logger.js';

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
  logger.debug(`getMetadata: ${path}`);
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

  let mystring = dataexitf.Photo.UserComment.toString('utf8').replaceAll('\u0000', '');
  let userdata = JSON.parse(mystring.slice(7).trim());

  if (userdata.hasOwnProperty('MinTemp')) measures.push({ name: 'TempMin', value: userdata.MinTemp });
  if (userdata.hasOwnProperty('MaxTemp')) measures.push({ name: 'TempMax', value: userdata.MaxTemp });
  if (userdata.hasOwnProperty('DistVeg')) measures.push({ name: 'DistVeg', value: userdata.DistVeg });
  if (userdata.hasOwnProperty('DistFle')) measures.push({ name: 'DistFle', value: userdata.DistFle });

  return { latitude, longitude, measures };
}

export async function ProcessThermalImage(input, output) {
  logger.info(`Processing ThermalImage: ${input} -> ${output}`);
  if (!processThermalImg) {
    logger.info('Thermal image processing is disabled');
    return false;
  }
  try {
    const { stdout, stderr } = await exec(` ${processThermalScript} -i "${input}" -o "${output}" `, {
      shell: '/bin/bash',
    });
    logger.debug(`stdout: ${stdout}`);
    if (stderr) logger.debug(`stderr: ${stderr}`);
  } catch (e) {
    logger.error(e);
    return false;
  }
  logger.info('ProcessThermalImage finished');
  return true;
}

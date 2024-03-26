import { createRequire } from 'node:module';
const require = createRequire(import.meta.url);
import { writeFile, readFileSync } from 'fs';
import { parse, stringify } from 'yaml';
import { URL } from 'url';
import { resolve } from 'path';

const __filename = new URL('', import.meta.url).pathname;
const __dirname = new URL('.', import.meta.url).pathname;

export const readJSON = (path) => require(path);

export const readYAML = (path) => {
  let devices_msg = {};
  try {
    let fileContents = readFileSync(resolve(__dirname, path), 'utf8');
    devices_msg = parse(fileContents);
    console.log('load ' + path);
  } catch (e) {
    console.log(e);
  }
  return devices_msg;
};

export const getDatetime = () => {
  var dateTime = new Date();
  return dateTime.toISOString();
};
export const getRandomInt = (max) => {
  return Math.floor(Math.random() * max);
};

export const writeYAML = (path, content) => {
  const saveContent = stringify(content);
  writeFile(__dirname + path, saveContent, (err) => {
    if (err) {
      console.error(err);
      return false;
    } else {
      // file written successfully
      return true;
    }
  });
};

export const GetLocalTime = (date) => {
  return new Date(date.getTime() - date.getTimezoneOffset() * 60 * 1000);
};
export const addTime = (date, minute) => {
  return new Date(date.getTime() + minute * 60000);
};
export const dateString = (date) => {
  return date.toISOString().slice(0, -8).replace('T', ' ');
};
export const sleep = (ms) => {
  return new Promise((resolve) => setTimeout(resolve, ms));
};

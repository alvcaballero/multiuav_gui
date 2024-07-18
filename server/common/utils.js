import { createRequire } from 'node:module';
const require = createRequire(import.meta.url);
import { writeFile, readFileSync, existsSync } from 'fs';
import { parse, stringify } from 'yaml';
import { URL } from 'url';
import { resolve } from 'path';

const __filename = new URL('', import.meta.url).pathname;
const __dirname = new URL('.', import.meta.url).pathname;

export const readJSON = (filepath) => {
  if (existsSync(resolve(__dirname, filepath))) {
    //console.log(`The file or directory at '${filepath}' exists.`);
  } else {
    console.log(`File '${filepath}' does not exist.`);
    return {};
  }
  console.log('load ' + filepath);
  let fileContents = readFileSync(resolve(__dirname, filepath), 'utf8');
  return JSON.parse(fileContents);
};

export const readYAML = (filepath) => {
  let path = filepath;
  if (existsSync(resolve(__dirname, filepath))) {
    //console.log(`The file or directory at '${filepath}' exists.`);
  } else {
    console.log(`File '${filepath}' does not exist.`);
    if (filepath == '../config/devices/devices_init.yaml') {
      path = '../config/devices/.devices_init.yaml';
    }
  }
  let content = {};
  try {
    let fileContents = readFileSync(resolve(__dirname, path), 'utf8');
    content = parse(fileContents);
    console.log('load ' + path);
  } catch (e) {
    console.log(e);
  }
  return content;
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
export const writeJSON = (path, content) => {
  const saveContent = JSON.stringify(content, null, 2);
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

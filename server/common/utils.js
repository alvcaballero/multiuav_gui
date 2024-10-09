import { writeFile, readFileSync, existsSync } from 'fs';
import { parse, stringify } from 'yaml';
import { fileURLToPath } from 'url';
import { dirname, resolve, normalize } from 'path';

const __dirname = dirname(fileURLToPath(import.meta.url));

export const checkFile = (filepath) => {
  const dir = resolve(__dirname, filepath);
  if (!existsSync(dir)) {
    console.log(`File '${filepath}' does not exist.`);
    return null;
  }
  return dir;
};

export const readDataFile = (filepath) => {
  if (filepath.includes('.json')) {
    return readJSON(filepath);
  }
  if (filepath.includes('.yaml')) {
    return readYAML(filepath);
  }
};
export const writeDataFile = (filepath, content) => {
  if (filepath.includes('.json')) {
    return writeJSON(filepath, content);
  }
  if (filepath.includes('.yaml')) {
    return writeYAML(filepath, content);
  }
};

export const readJSON = (filepath) => {
  console.log('load ' + filepath);
  let jsonfile = {};
  if (!existsSync(resolve(__dirname, filepath))) {
    console.log(`File '${filepath}' does not exist.`);
    return {};
  }
  try {
    let fileContents = readFileSync(resolve(__dirname, filepath), 'utf8');
    jsonfile = JSON.parse(fileContents);
  } catch (e) {
    console.log(`file is not a json file ${filepath}`);
    return {};
  }
  return jsonfile;
};

export const readYAML = (filepath) => {
  let path = filepath;
  let content = {};
  if (!existsSync(resolve(__dirname, filepath))) {
    console.log(`File '${filepath}' does not exist.`);
    return {};
  }
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
  writeFile(resolve(__dirname, path), saveContent, (err) => {
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
  console.log('write Json' + path);
  const saveContent = JSON.stringify(content, null, 2);

  writeFile(resolve(__dirname, path), saveContent, (err) => {
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

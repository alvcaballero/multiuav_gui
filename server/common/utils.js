import { writeFileSync, readFileSync, existsSync } from 'fs';
import { parse, stringify } from 'yaml';
import { fileURLToPath } from 'url';
import { dirname, resolve, normalize } from 'path';

const __dirname = dirname(fileURLToPath(import.meta.url));

export const round = (number, decimals = 0) => {
  const factor = Math.pow(10, decimals);
  return Math.round(number * factor) / factor;
};

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
export const writeDataFile = async (filepath, content) => {
  if (filepath.includes('.json')) {
    return await writeJSON(filepath, content);
  }
  if (filepath.includes('.yaml')) {
    return await writeYAML(filepath, content);
  }
  return false;
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

export const getDatetime = (withMilliseconds = false) => {
  const dateTime = new Date();
  if (!withMilliseconds) {
    dateTime.setMilliseconds(0);
  }
  return dateTime.toISOString();
};
export const getRandomInt = (max) => {
  return Math.floor(Math.random() * max);
};

export const writeYAML = async (path, content) => {
  const saveContent = stringify(content);
  return await writeData(path, saveContent);
};
export const writeJSON = async (path, content) => {
  console.log('write Json' + path);
  const saveContent = JSON.stringify(content, null, 2);
  return await writeData(path, saveContent);
};

const writeData = async (path, content) => {
  try {
    await writeFileSync(resolve(__dirname, path), content);
    return true;
  } catch (err) {
    console.log(err);
    return false;
  }
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

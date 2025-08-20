import { writeFileSync, readFileSync, existsSync } from 'fs';
import { parse, stringify } from 'yaml';
import { fileURLToPath } from 'url';
import { dirname, resolve, normalize } from 'path';
import logger from './logger.js';

const __dirname = dirname(fileURLToPath(import.meta.url));

export const round = (number, decimals = 0) => {
  const factor = Math.pow(10, decimals);
  return Math.round(number * factor) / factor;
};

export const checkFile = (filepath) => {
  const dir = resolve(__dirname, filepath);
  if (!existsSync(dir)) {
    logger.warn(`File '${filepath}' does not exist.`);
    return null;
  }
  return dir;
};

export const readDataFile = (filepath) => {
  let dir = checkFile(filepath);
  if (dir === null) return {};
  try {
    const fileContents = readFileSync(dir, 'utf8');

    if (filepath.includes('.json')) {
      return JSON.parse(fileContents);
    }
    if (filepath.includes('.yaml')) {
      return parse(fileContents);
    }
  } catch (e) {
    logger.error(`Error reading file ${dir}: ${e.message}`);
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

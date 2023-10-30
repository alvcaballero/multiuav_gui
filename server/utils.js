import { createRequire } from 'node:module';
const require = createRequire(import.meta.url);
import { readFileSync } from 'fs';
import { parse } from 'yaml';
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
  var datetime = new Date();
  return datetime.toISOString();
};

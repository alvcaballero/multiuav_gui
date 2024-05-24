import { rename, readdir, readdirSync, existsSync } from 'fs';
import { URL } from 'url';
import { resolve } from 'path';

function renameJs(path) {
  console.log('call to function ');
  console.log(path);
  readdir(path, { withFileTypes: true }, (err, files) => {
    if (err) {
      console.log(err);
    } else {
      files.forEach((item) => {
        if (item.isDirectory()) {
          console.log('is directory');
          renameJs(`${path}/${item.name}`);
        }
        if (item.name.endsWith('.js')) {
          console.log('is file .js');
          const regex = /\.js$/;
          const newfilename = item.name.replace(regex, '.cjs');
          console.log(`${path}/${item.name} + ${path}/${newfilename}`);
          rename(`${path}/${item.name}`, `${path}/${newfilename}`, () => {
            console.log('\nFile Renamed!\n');
          });
        }
      });
    }
  });
}

const outDir = './dist';
const __dirname = new URL('', import.meta.url).pathname;
resolve(__dirname, outDir);
renameJs(outDir);

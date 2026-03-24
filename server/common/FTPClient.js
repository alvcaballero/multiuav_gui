// https://www.npmjs.com/package/basic-ftp
//
// ftp.js
//
// Use this sample code to connect to your SFTP To Go server and run some file operations using Node.js.
//
// 1) Paste this code into a new file (sftp.js)
//
// 2) Install dependencies
//   npm install basic-ftp
//
// 3) Run the script
//   node ftp.js
//
// Compatible with Node.js >= v12
// this is a emulation of format of the SFTPClient class
import * as ftp from 'basic-ftp';
import logger from './logger.js';

function filterList(fileList, pattern = /.*/) {
  let newList = [];
  newList = fileList.map((item) => {
    return {
      type: item.type === 2 ? 'd' : item.type === 1 ? '-' : 'l',
      name: item.name,
      size: item.size,
      modifyTime: item.rawModifiedAt,
      accessTime: item.rawModifiedAt,
      rights: {
        user: item.permissions.user ?? 0,
        group: item.permissions.group ?? 0,
        other: item.permissions.world ?? 0,
      },
      owner: item.user,
      group: item.group,
      longname: item.name,
    };
  });
  let regex;
  if (pattern instanceof RegExp) {
    regex = pattern;
  } else {
    let newPattern = pattern.replace(/\*([^*])*?/gi, '.*');
    regex = new RegExp(newPattern);
  }
  let filteredList = newList.filter((item) => regex.test(item.name));
  return filteredList;
}

export class FTPClient {
  constructor() {
    logger.debug(‘FTPClient initialized’);
    this.client = new ftp.Client();
  }
  /**
   * Close the client and all open socket connections.
   * @option {host, port, user, password}
   * Close the client and all open socket connections. The client can’t be used anymore after calling this method,
   * you have to either reconnect with `access` or `connect` or instantiate a new instance to continue any work.
   * A client is also closed automatically if any timeout or connection error occurs.
   */

  async connect(options) {
    logger.info(`Connecting to FTP server ${options.host}:${options.port}`);
    this.client.ftp.verbose = false;
    options.user = options.username;
    try {
      await this.client.access(options);
    } catch (err) {
      logger.error(`Failed to connect to FTP server: ${err.message}`);
      return false;
    }
    return true;
  }

  async disconnect() {
    await this.client.close();
  }

  /**
   *
   * List contents of a remote directory. If a pattern is provided,
   * filter the results to only include files with names that match
   * the supplied pattern. Return value is an array of file entry
   * objects that include properties for type, name, size, modifiyTime,
   * accessTime, rights {user, group other}, owner and group.
   * typefile = '-' =>, 'd' => directory, 'l'
   */
  async listFiles(remoteDir, fileGlob, typefile = 'all', order = false) {
    logger.debug(`Listing directory: ${remoteDir}`);
    let fileObjects = [];
    let list = [];
    try {
      list = await this.client.list(remoteDir);
      fileObjects = filterList(list, fileGlob);
    } catch (err) {
      logger.error(`Failed to list directory: ${err.message}`);
      return [];
    }

    if (order) {
      logger.debug('Sorting files by modification time');
      fileObjects = fileObjects.sort(function (a, b) {
        return b.modifyTime - a.modifyTime;
      });
    }

    let fileNames = [];

    for (const file of fileObjects) {
      if (file.type === 'd') {
        logger.debug(`[DIR] ${file.modifyTime} ${file.name}`);
      } else if (file.type === '-') {
        logger.debug(`[FILE] ${file.modifyTime} ${file.size} bytes ${file.name}`);
      } else {
        logger.debug(`[LINK] ${file.modifyTime} ${file.size} bytes ${file.name}`);
      }
      if (file.type === typefile || typefile === 'all') {
        fileNames.push(file.name);
      }
    }
    return fileNames;
  }

  async uploadFile(localFile, remoteFile) {
    logger.info(`Uploading ${localFile} to ${remoteFile}`);
    try {
      let data = await this.client.uploadFrom(localFile, remoteFile);
      logger.info(`File uploaded successfully: ${remoteFile}`);
      return { status: true, data: data };
    } catch (err) {
      logger.error(`Upload failed: ${err.message}`);
      return { status: false, data: 'Uploading failed' };
    }
  }

  async downloadFile(remoteFile, localFile) {
    logger.info(`Downloading ${remoteFile} to ${localFile}`);
    try {
      let data = await this.client.downloadTo(localFile, remoteFile);
      logger.info(`File downloaded successfully: ${localFile}`);
      return { status: true, data: data };
    } catch (err) {
      logger.error(`Download failed: ${err.message}`);
      return { status: false, data: 'download failed' };
    }
  }

  async deleteFile(remoteFile) {
    logger.info(`Deleting ${remoteFile}`);
    try {
      await this.client.remove(remoteFile);
    } catch (err) {
      logger.error(`Delete failed: ${err.message}`);
    }
  }
}

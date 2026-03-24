// https://sftptogo.com/blog/node-sftp/
// sftp.js
//
// Use this sample code to connect to your SFTP To Go server and run some file operations using Node.js.
//
// 1) Paste this code into a new file (sftp.js)
//
// 2) Install dependencies
//   npm install ssh2-sftp-client@^8.0.0
//
// 3) Run the script
//   node sftp.js
//
// Compatible with Node.js >= v12
// Using ssh2-sftp-client v8.0.0

import sftp from 'ssh2-sftp-client';
import logger from './logger.js';

export class SFTPClient {
  constructor() {
    logger.debug('SFTPClient initialized');
    this.client = new sftp();
  }

  async connect(options) {
    logger.info(`Connecting to SFTP server ${options.host}:${options.port}`);
    try {
      await this.client.connect(options);
    } catch (err) {
      logger.error(`Failed to connect to SFTP server: ${err.message}`);
      return false;
    }
    return true;
  }

  async disconnect() {
    await this.client.end();
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
    let fileObjects;
    try {
      fileObjects = await this.client.list(remoteDir, fileGlob);
    } catch (err) {
      logger.error(`Failed to list directory: ${err.message}`);
    }

    if (order) {
      logger.debug('Sorting files by modification time');
      fileObjects = fileObjects.sort(function (a, b) {
        return b.modifyTime - a.modifyTime;
      });
    }

    let fileNames = [];

    for (const file of fileObjects) {
      const timestamp = new Date(file.modifyTime).toISOString();
      if (file.type === 'd') {
        logger.debug(`[DIR] ${timestamp} ${file.name}`);
      } else if (file.type === '-') {
        logger.debug(`[FILE] ${timestamp} ${file.size} bytes ${file.name}`);
      } else {
        logger.debug(`[LINK] ${timestamp} ${file.size} bytes ${file.name}`);
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
      let data = await this.client.put(localFile, remoteFile);
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
      let data = await this.client.get(remoteFile, localFile);
      logger.info(`File downloaded successfully: ${localFile}`);
      return { status: true, data: data };
    } catch (err) {
      logger.error(`Download failed: ${err.message}`);
      return { status: false, data: 'download failed' };
    }
  }

  async deleteFile(remoteFile) {
    logger.debug(`Deleting ${remoteFile}`);
    try {
      await this.client.delete(remoteFile);
    } catch (err) {
      logger.error(`Deleting ${remoteFile} failed: ${err.message}`);
    }
  }
}

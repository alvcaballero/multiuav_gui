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

export class SFTPClient {
  constructor() {
    console.log('SFTPClient constructor');
    this.client = new sftp();
  }

  async connect(options) {
    console.log(`Connecting to ${options.host}:${options.port}`);
    try {
      await this.client.connect(options);
    } catch (err) {
      console.log('Failed to connect: device  ===== !!!'); //, err);
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
    console.log(`Listing ${remoteDir} ...`);
    let fileObjects;
    try {
      fileObjects = await this.client.list(remoteDir, fileGlob);
    } catch (err) {
      console.log('Listing failed:', err);
    }

    if (order) {
      console.log('into in order');
      fileObjects = fileObjects.sort(function (a, b) {
        return b.modifyTime - a.modifyTime;
      });
    }

    let fileNames = [];

    for (const file of fileObjects) {
      if (file.type === 'd') {
        console.log(`${new Date(file.modifyTime).toISOString()} PRE ${file.name}`);
      } else if (file.type === '-') {
        console.log(`${new Date(file.modifyTime).toISOString()} - ${file.size} ${file.name}`);
      } else {
        console.log(`${new Date(file.modifyTime).toISOString()} l ${file.size} ${file.name}`);
      }
      if (file.type === typefile || typefile === 'all') {
        fileNames.push(file.name);
      }
    }
    return fileNames;
  }

  async uploadFile(localFile, remoteFile) {
    console.log(`Uploading ${localFile} to ${remoteFile} ...`);
    try {
      let data = await this.client.put(localFile, remoteFile);
      return { status: true, data: data };
    } catch (err) {
      console.error('Uploading failed:', err);
      return { status: false, data: 'Uploading failed' };
    }
  }

  async downloadFile(remoteFile, localFile) {
    console.log(`Downloading ${remoteFile} to ${localFile} ...`);
    try {
      let data = await this.client.get(remoteFile, localFile);
      return { status: true, data: data };
    } catch (err) {
      console.error('Downloading failed:', err);
      return { status: false, data: 'download failed' };
    }
  }

  async deleteFile(remoteFile) {
    console.log(`Deleting ${remoteFile}`);
    try {
      await this.client.delete(remoteFile);
    } catch (err) {
      console.error('Deleting failed:', err);
    }
  }
}

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

function filterList(fileList, pattern = /.*/) {
  let newList = [];
  newList = fileList.map((item) => {
    return {
      type: item.type,
      name: item.name,
      size: item.size,
      modifyTime: item.modifiedAt,
      accessTime: item.modifiedAt,
      rights: {
        user: item.UnixPermissions.user,
        group: item.UnixPermissions.group,
        other: item.UnixPermissions.world,
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
  this.debugMsg('list: result: ', filteredList);
  return filteredList;
}

export class FTPClient {
  constructor() {
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
    console.log(`Connecting to ${options.host}:${options.port}`);
    this.client.ftp.verbose = true;
    try {
      await this.client.access(options);
    } catch (err) {
      console.log('Failed to connect:', err);
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
    console.log(`Listing ${remoteDir} ...`);
    let fileObjects = [];
    let list = [];
    try {
      list = await this.client.list(remoteDir);
      fileObjects = filterList(list, fileGlob);
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
      let data = await this.client.uploadFrom(localFile, remoteFile);
      return { status: true, data: data };
    } catch (err) {
      console.error('Uploading failed:', err);
      return { status: false, data: 'Uploading failed' };
    }
  }

  async downloadFile(remoteFile, localFile) {
    console.log(`Downloading ${remoteFile} to ${localFile} ...`);
    try {
      let data = await this.client.downloadTo(remoteFile, localFile);
      return { status: true, data: data };
    } catch (err) {
      console.error('Downloading failed:', err);
      return { status: false, data: 'download failed' };
    }
  }

  async deleteFile(remoteFile) {
    console.log(`Deleting ${remoteFile}`);
    try {
      await this.client.remove(remoteFile);
    } catch (err) {
      console.error('Deleting failed:', err);
    }
  }
}

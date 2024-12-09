//https://www.youtube.com/watch?v=gnM3Ld6_upE-- REVISAR
//https://medium.com/agora-io/how-does-webrtc-work-996748603141
import { port, db, RosEnable, FbEnable } from './config/config.js';
import express, { json } from 'express';
import logger from 'morgan';
import { createServer } from 'http';
import { corsMiddleware } from './middlewares/cors.js';
import { checkFile } from './common/utils.js';

console.log('use db is ' + db);

//ws - for client
import { WebsocketManager } from './WebsocketManager.js';
// express routes
import { createDevicesRouter } from './routes/devices.js';
import { categoryRouter } from './routes/category.js';
import { positionsRouter } from './routes/positions.js';
import { eventsRouter } from './routes/events.js';
import { commandsRouter } from './routes/commands.js';
import { mapRouter } from './routes/map.js';
import { createMissionRouter } from './routes/mission.js';
import { createFilesRouter } from './routes/files.js';
import { ExtAppRouter } from './routes/ExtApp.js';
import { serverRouter } from './routes/server.js';
import { planningRouter } from './routes/planning.js';

// comunication with devices
console.log(` !!!!=== RosEnable is ${RosEnable} FbEnable is ${FbEnable}`);

import { WebsocketDevices } from './WebsocketDevices.js'; // flatbuffer
import { rosModel } from './models/ros.js'; // ros model

//model

let DevicesModel = db
  ? await import('./models/devices-sql.js').then((module) => module.DevicesModel)
  : await import('./models/devices.js').then((module) => module.DevicesModel);
let MissionModel = db
  ? await import('./models/mission-sql.js').then((module) => module.missionModel)
  : await import('./models/mission.js').then((module) => module.missionModel);
let filesModel = db
  ? await import('./models/files-sql.js').then((module) => module.filesModel)
  : await import('./models/files.js').then((module) => module.filesModel);

// setting APP
const app = express();
app.set('port', port);
app.use(corsMiddleware());
app.use(json());
app.use(logger('dev'));

let wedAppPath = checkFile('../client/build');
if (wedAppPath === null) {
  wedAppPath = checkFile('../../client/build');
}
console.log('wedAppPath ' + wedAppPath);

app.use(express.static(wedAppPath));
app.use('/api/devices', createDevicesRouter({ model: DevicesModel }));
app.use('/api/category', categoryRouter);
app.use('/api/positions', positionsRouter);
app.use('/api/events', eventsRouter);
app.use('/api/commands', commandsRouter);
//app.use('/api/ros', rosRouter);
app.use('/api/map', mapRouter);
app.use('/api/missions', createMissionRouter({ model: MissionModel }));
app.use('/api/files', createFilesRouter({ model: filesModel }));
app.use('/api/planning', planningRouter);
app.use('/api/ExtApp', ExtAppRouter);
app.use('/api/server', serverRouter);

const server = createServer(app);
const ws = new WebsocketManager(server, '/api/socket');

// connect to  devices
if (RosEnable) {
  console.log('init RosEnable');
  rosModel.rosConnect();
}
if (FbEnable) {
  console.log('init FbEnable');
  var ws2 = new WebsocketDevices(8082);
}

//
// Start the server.
//
server.listen(app.get('port'), () => {
  console.log('Servidor iniciado en el puerto: ' + app.get('port'));
});
//
//

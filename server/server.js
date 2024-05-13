//https://www.youtube.com/watch?v=gnM3Ld6_upE-- REVISAR
//https://medium.com/agora-io/how-does-webrtc-work-996748603141
import * as path from 'path';
import { URL } from 'url';

const __filename = new URL('', import.meta.url).pathname;
const __dirname = new URL('.', import.meta.url).pathname;

import { port, db } from './config/config.js';
import express, { json } from 'express';
import logger from 'morgan';
import { createServer } from 'http';
import { corsMiddleware } from './middlewares/cors.js';

console.log('use db is ' + db);

//ws
import { WebsocketManager } from './WebsocketManager.js';
import { WebsocketDevices } from './WebsocketDevices.js';
// rutas
import { createDevicesRouter } from './routes/devices.js';
import { categoryRouter } from './routes/category.js';
import { positionsRouter } from './routes/positions.js';
import { eventsRouter } from './routes/events.js';
import { commandsRouter } from './routes/commands.js';
import { rosRouter } from './routes/ros.js';
import { mapRouter } from './routes/map.js';
import { createMissionRouter } from './routes/mission.js';
import { createFilesRouter } from './routes/files.js';
import { ExtAppRouter } from './routes/ExtApp.js';
import { serverRouter } from './routes/server.js';
import { planningRouter } from './routes/planning.js';

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

app.use(express.static(path.resolve(__dirname, '../client/build')));
app.use('/api/devices', createDevicesRouter({ model: DevicesModel }));
app.use('/api/category', categoryRouter);
app.use('/api/positions', positionsRouter);
app.use('/api/events', eventsRouter);
app.use('/api/commands', commandsRouter);
app.use('/api/ros', rosRouter);
app.use('/api/map', mapRouter);
app.use('/api/missions', createMissionRouter({ model: MissionModel }));
app.use('/api/files', createFilesRouter({ model: filesModel }));
app.use('/api/planning', planningRouter);
app.use('/api/ExtApp', ExtAppRouter);
app.use('/api/server', serverRouter);

const server = createServer(app);
var ws = new WebsocketManager(server, '/api/socket');
var ws2 = new WebsocketDevices(8080);
//
// Start the server.
//
server.listen(app.get('port'), () => {
  console.log('Servidor iniciado en el puerto: ' + app.get('port'));
});
//
//

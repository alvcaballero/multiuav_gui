import { port, RosEnable, FbEnable } from './config/config.js';
import { LLMType, LLM, LLMApiKey } from './config/config.js';

import express, { json } from 'express';
import logger from 'morgan';
import { createServer } from 'http';
import { corsMiddleware } from './middlewares/cors.js';
import { checkFile } from './common/utils.js';
import { initializeLLMProvider } from './controllers/llm.js'; // LLM provider initialization

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
import { geofenceRouter } from './routes/geofence.js';
import { llmRouter } from './routes/llm.js';

// comunications with devices
import { WebsocketDevices } from './WebsocketDevices.js'; // flatbuffer
import { rosModel } from './models/ros.js'; // ros model

// comunication with devices
console.log(` !!!!=== RosEnable is ${RosEnable} FbEnable is ${FbEnable}`);

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
app.use('/api/devices', createDevicesRouter());
app.use('/api/category', categoryRouter);
app.use('/api/positions', positionsRouter);
app.use('/api/events', eventsRouter);
app.use('/api/commands', commandsRouter);
app.use('/api/map', mapRouter);
app.use('/api/missions', createMissionRouter());
app.use('/api/files', createFilesRouter());
app.use('/api/planning', planningRouter);
app.use('/api/ExtApp', ExtAppRouter);
app.use('/api/server', serverRouter);
app.use('/api/geofences', geofenceRouter);
app.use('/api/chat', llmRouter);
//app.use('/api/ros', rosRouter);

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

// Initialize the LLM provider if enabled.
// Check if LLM is enabled and if the API key is provided.
console.log(`LLM is ${LLM}, LLMType is ${LLMType}, LLMApiKey is ${LLMApiKey}`);

if (LLM) {
  if (!LLMApiKey) {
    throw new Error('LLM API Key is required. Please set LLM_API_KEY in your environment variables.');
  }
  initializeLLMProvider(LLMType, LLMApiKey);
} else {
  console.warn('LLM is disabled. No LLM provider will be initialized.');
}

// Start the server.
server.listen(app.get('port'), () => {
  console.log('Servidor iniciado en el puerto: ' + app.get('port'));
});

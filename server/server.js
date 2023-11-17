//https://www.youtube.com/watch?v=gnM3Ld6_upE-- REVISAR
//https://medium.com/agora-io/how-does-webrtc-work-996748603141
import * as path from 'path';
import { URL } from 'url';

const __filename = new URL('', import.meta.url).pathname;
const __dirname = new URL('.', import.meta.url).pathname;

import express, { json } from 'express';
import { createServer } from 'http';
import { corsMiddleware } from './middlewares/cors.js';
import { devicesRouter } from './routes/devices.js';
import { categoryRouter } from './routes/category.js';
import { positionsRouter } from './routes/positions.js';
import { eventsRouter } from './routes/events.js';
import { commandsRouter } from './routes/commands.js';
import { rosRouter } from './routes/ros.js';
import { mapRouter } from './routes/map.js';
import { utilsRouter } from './routes/utils.js';
import { websocketController } from './controllers/websocket.js';

import WebSocket, { WebSocketServer } from 'ws';

const app = express();
const port = 4000;
app.set('port', port);
app.use(corsMiddleware());
app.use(json());

app.use(express.static(path.resolve(__dirname, '../build')));
app.use('/api/devices', devicesRouter);
app.use('/api/category', categoryRouter);
app.use('/api/positions', positionsRouter);
app.use('/api/events', eventsRouter);
app.use('/api/commands', commandsRouter);
app.use('/api/ros', rosRouter);
app.use('/api/map', mapRouter);
app.use('/api/utils', utilsRouter);

const server = createServer(app);

function heartbeat() {
  this.isAlive = true;
}

const wss = new WebSocket.Server({ path: '/api/socket', server: server });

wss.on('connection', async function connection(ws) {
  console.log('newclient');
  ws.isAlive = true;

  ws.on('error', console.error);

  ws.on('pong', heartbeat);

  ws.on('message', function message(data) {
    console.log('received: %s', data);
  });

  let init_msg = await websocketController.init();

  ws.send(init_msg);
});
//https://www.npmjs.com/package/ws#sending-binary-data  find "ping"

const interval_update = setInterval(async () => {
  let msg = await websocketController.update();
  wss.clients.forEach(function each(ws) {
    ws.send(msg);
  });
}, 250);

const interval_server = setInterval(async () => {
  let msg = await websocketController.updateserver();
  wss.clients.forEach(async function each(ws) {
    ws.send(msg);
  });
}, 5000);

const interval = setInterval(function ping() {
  wss.clients.forEach(function each(ws) {
    if (ws.isAlive === false) return ws.terminate();
    ws.isAlive = false;
    ws.ping();
  });
}, 30000);

wss.on('close', function close() {
  clearInterval(interval);
  clearInterval(interval_update);
  clearInterval(interval_server);
});

//
// Start the server.
//
server.listen(app.get('port'), () => {
  console.log('Servidor iniciado en el puerto: ' + app.get('port'));
});

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

import WebSocket, { WebSocketServer } from 'ws';

const app = express();
const port = 4000;
app.set('port', port);
app.use(corsMiddleware());
app.use(json());
app.use(express.static(path.resolve(__dirname, '../build')));

const server = createServer(app);

const wss = new WebSocketServer({
  path: '/api/socket',
  server: server,
});

app.use('/api/devices', devicesRouter);

wss.on('connection', function connection(ws) {
  console.log('newclient');
  ws.on('error', console.error);

  ws.on('message', function message(data) {
    console.log('received: %s', data);
  });
  ws.send(JSON.stringify({ positions: [], camera: [], server: [], devices: [] }));

  const interval = setInterval(() => {
    let currentsocket = {};
    if (true) {
      currentsocket['positions'] = [];
    }
    if (true) {
      currentsocket['camera'] = [];
    }
    if (true) {
      currentsocket['events'] = [];
    }
    ws.send(JSON.stringify(currentsocket));
  }, 200);

  const interval_server = setInterval(() => {
    ws.send(
      JSON.stringify({
        server: { rosState: true },
        devices: [],
      })
    );
  }, 5000);
});
//https://www.npmjs.com/package/ws#sending-binary-data  find "ping"
const interval = setInterval(function ping() {
  wss.clients.forEach(function each(ws) {
    //if (ws.isAlive === false) return ws.terminate();
    //ws.isAlive = false;
    //ws.ping();
  });
}, 30000);

wss.on('close', function close() {
  clearInterval(interval);
});

server.listen(app.get('port'), () => {
  console.log('Servidor iniciado en el puerto: ' + app.get('port'));
});

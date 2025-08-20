import { port, RosEnable, FbEnable } from './config/config.js';
import { LLMType, LLM, LLMApiKey } from './config/config.js';

import express, { json } from 'express';
import { createServer } from 'http';
import { corsMiddleware } from './middlewares/cors.js';
import { setupLogger } from './middlewares/logger.js';
import { checkFile } from './common/utils.js';
import { initializeLLMProvider } from './controllers/llm.js'; // LLM provider initialization
import logger, { logHelpers } from './common/logger.js';

//ws - for client
import { WebsocketManager } from './WebsocketManager.js';
import { setupRoutes } from './routes/index.js';

// comunications with devices
import { WebsocketDevices } from './WebsocketDevices.js'; // flatbuffer
import { rosModel } from './models/ros.js'; // ros model
import e from 'express';

// comunication with devices
logger.info('Iniciando servidor MultiUAV GUI', {
  nodeEnv: process.env.NODE_ENV,
  port: port,
  rosEnabled: RosEnable,
  fbEnabled: FbEnable,
  llmEnabled: LLM,
});

// setting APP
const app = express();
app.set('port', port);
app.use(corsMiddleware());
setupLogger(app);
app.use(json());

let wedAppPath = checkFile('../client/build');
if (wedAppPath === null) {
  wedAppPath = checkFile('../../client/build');
}

app.use(express.static(wedAppPath));
setupRoutes(app);

const server = createServer(app);
const ws = new WebsocketManager(server, '/api/socket');

// connect to  devices
if (RosEnable) {
  logger.info('ROS habilitado, iniciando conexión');
  rosModel.rosConnect();
  logHelpers.ros.connect('main-server', {
    status: 'connecting',
    timestamp: new Date().toISOString(),
  });
} else {
  logger.warn('ROS deshabilitado en configuración');
}
if (FbEnable) {
  logger.info('FlatBuffer habilitado, iniciando WebSocket de dispositivos en puerto 8082');
  var ws2 = new WebsocketDevices(8082);
  logHelpers.system.info('WebSocket de dispositivos iniciado correctamente', {
    port: 8082,
    protocol: 'flatbuffer',
  });
} else {
  logger.warn('FlatBuffer deshabilitado en configuración');
}

// Initialize the LLM provider if enabled.
// Check if LLM is enabled and if the API key is provided.
console.log(`LLM is ${LLM}, LLMType is ${LLMType}, LLMApiKey is ${LLMApiKey}`);

if (LLM) {
  if (!LLMApiKey) {
    const error = new Error('LLM API Key is required. Please set LLM_API_KEY in your environment variables.');
    logger.error('Error de configuración LLM', {
      error: error.message,
      type: 'configuration',
    });
    throw error;
  }
  initializeLLMProvider(LLMType, LLMApiKey);
} else {
  logger.warn('LLM deshabilitado, no se inicializará ningún proveedor');
}

// Configurar manejo de errores globales
process.on('uncaughtException', (error) => {
  //  rosModel.GCSunServicesMission();
  logger.error('Excepción no capturada', {
    error: error.message,
    stack: error.stack,
    type: 'uncaughtException',
  });
  process.exit(1);
});

process.on('unhandledRejection', (reason, promise) => {
  logger.error('Promise rechazada no manejada', {
    reason: reason,
    promise: promise.toString(),
    type: 'unhandledRejection',
  });
});

// Graceful shutdown
process.on('SIGTERM', () => {
  logger.info('SIGTERM recibido, cerrando servidor gracefully');
  server.close(() => {
    logger.info('Servidor cerrado correctamente');
    process.exit(0);
  });
});
process.on('SIGINT', () => {
  logger.info('SIGINT recibido, cerrando servidor gracefully');
  // rosModel.GCSunServicesMission();
  server.close(() => {
    logger.info('Servidor cerrado correctamente');
  });

  process.exit(0);
});
process.on('exit', (code) => {
  logger.info('Proceso finalizado con código:', code);
});

// Start the server.
server.listen(app.get('port'), () => {
  logger.info('MultiUAV server ready', {
    port: app.get('port'),
    environment: process.env.NODE_ENV || 'development',
    timestamp: new Date().toISOString(),
    pid: process.pid,
    nodeVersion: process.version,
  });

  logHelpers.system.info('WS ready', {
    websocketEndpoint: '/api/socket',
    devicePort: FbEnable ? 8082 : null,
    rosEnabled: RosEnable,
    llmEnabled: LLM,
  });
});

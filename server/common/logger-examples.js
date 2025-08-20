/**
 * Ejemplos de uso del Logger para el proyecto MultiUAV GUI
 *
 * Este archivo muestra cómo integrar el logger en diferentes partes del código
 */

import { logger, logHelpers, wsLogger, deviceLogger, rosLogger, createCustomLogger } from './logger.js';

// =====================================================
// EJEMPLOS DE USO BÁSICO
// =====================================================

// Logger básico
logger.info('Servidor iniciado correctamente');
logger.warn('Advertencia: Configuración de base de datos no encontrada');
logger.error('Error al conectar con ROS', { error: 'Connection refused' });

// =====================================================
// EJEMPLOS PARA API/EXPRESS
// =====================================================

// En middleware de Express para logging de requests
function requestLogger(req, res, next) {
  const start = Date.now();

  // Log del request
  logHelpers.api.request(req.method, req.url, req.ip, {
    userAgent: req.get('User-Agent'),
    contentType: req.get('Content-Type'),
  });

  // Log del response cuando termine
  res.on('finish', () => {
    const duration = Date.now() - start;
    logHelpers.api.response(req.method, req.url, res.statusCode, duration);
  });

  next();
}

// En route handlers
function getDevicesHandler(req, res) {
  try {
    logHelpers.system.info('Obteniendo lista de dispositivos');
    const devices = getDevices(); // función ficticia

    logHelpers.system.info('Dispositivos obtenidos exitosamente', {
      count: devices.length,
    });

    res.json(devices);
  } catch (error) {
    logHelpers.api.error(req.method, req.url, error);
    res.status(500).json({ error: 'Error interno del servidor' });
  }
}

// =====================================================
// EJEMPLOS PARA WEBSOCKET
// =====================================================

// En WebSocketManager
function onWebSocketConnection(client, request) {
  const clientId = generateClientId(); // función ficticia

  logHelpers.ws.connect(clientId, {
    url: request.url,
    origin: request.headers.origin,
  });

  client.on('message', (message) => {
    logHelpers.ws.message(clientId, message.toString());
  });

  client.on('close', () => {
    logHelpers.ws.disconnect(clientId);
  });

  client.on('error', (error) => {
    logHelpers.ws.error(clientId, error);
  });
}

// Para broadcast de mensajes
function broadcastMessage(message, clients) {
  logHelpers.ws.broadcast(message, clients.length);

  clients.forEach((client) => {
    if (client.readyState === WebSocket.OPEN) {
      client.send(message);
    }
  });
}

// =====================================================
// EJEMPLOS PARA DISPOSITIVOS/DRONES
// =====================================================

// Cuando un dispositivo se conecta
function onDeviceConnect(deviceId, deviceName, metadata) {
  logHelpers.device.connect(deviceId, deviceName, {
    category: metadata.category,
    type: metadata.type,
    firmware: metadata.firmware,
  });
}

// Actualización de posición
function updateDevicePosition(deviceId, deviceName, position) {
  logHelpers.device.position(deviceId, deviceName, position, {
    timestamp: new Date().toISOString(),
  });
}

// Envío de comando
function sendCommandToDevice(deviceId, deviceName, command, parameters) {
  logHelpers.device.command(deviceId, deviceName, command, {
    parameters: parameters,
    timestamp: new Date().toISOString(),
  });
}

// Error en dispositivo
function handleDeviceError(deviceId, deviceName, error) {
  logHelpers.device.error(deviceId, deviceName, error, {
    timestamp: new Date().toISOString(),
  });
}

// =====================================================
// EJEMPLOS PARA ROS
// =====================================================

// Conexión a ROS
function connectToROS(nodeId) {
  try {
    // lógica de conexión...
    logHelpers.ros.connect(nodeId, {
      rosVersion: 'melodic',
      masterUri: 'http://localhost:11311',
    });
  } catch (error) {
    logHelpers.ros.error(nodeId, error);
  }
}

// Publicación en topic
function publishToTopic(topic, nodeId, message) {
  logHelpers.ros.publish(topic, nodeId, {
    messageType: message.constructor.name,
    timestamp: new Date().toISOString(),
  });
}

// Mensaje recibido
function onROSMessage(topic, nodeId, message, messageType) {
  logHelpers.ros.message(topic, nodeId, messageType, {
    timestamp: new Date().toISOString(),
    size: JSON.stringify(message).length,
  });
}

// =====================================================
// LOGGER PERSONALIZADO PARA MISIONES
// =====================================================

const missionLogger = createCustomLogger('MISSION', {
  level: 'info',
  color: chalk.yellow,
  filename: 'missions.log',
});

function startMission(missionId, uavIds) {
  missionLogger.info(`Iniciando misión ${missionId}`, {
    missionId,
    uavIds,
    startTime: new Date().toISOString(),
  });
}

function completeMission(missionId, results) {
  missionLogger.info(`Misión ${missionId} completada`, {
    missionId,
    results,
    endTime: new Date().toISOString(),
  });
}

// =====================================================
// INTEGRACIÓN EN CONTROLADORES EXISTENTES
// =====================================================

// Ejemplo de cómo modificar WebsocketController
class WebsocketControllerWithLogging {
  static async update() {
    try {
      logHelpers.system.debug('Actualizando datos de WebSocket');

      let currentsocket = {};
      const positions = await positionsController.getLastPositions();
      const camera = await positionsController.getCamera();

      if (Object.values(positions).length) {
        currentsocket['positions'] = positions;
        logHelpers.system.debug('Posiciones agregadas al socket', {
          count: Object.values(positions).length,
        });
      }

      if (Object.values(camera).length) {
        currentsocket['camera'] = Object.values(camera);
        logHelpers.system.debug('Datos de cámara agregados al socket', {
          count: Object.values(camera).length,
        });
      }

      return JSON.stringify(currentsocket);
    } catch (error) {
      logHelpers.system.error('Error al actualizar datos de WebSocket', error);
      throw error;
    }
  }
}

// =====================================================
// CONFIGURACIÓN GLOBAL PARA ERRORES NO MANEJADOS
// =====================================================

// Capturar errores no manejados
process.on('uncaughtException', (error) => {
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
    promise: promise,
    type: 'unhandledRejection',
  });
});

// =====================================================
// EJEMPLOS DE LOGS ESTRUCTURADOS CON METADATOS
// =====================================================

// Para debugging con contexto completo
function debugWithContext(operation, data) {
  logger.debug(`Operación: ${operation}`, {
    operation,
    data,
    timestamp: new Date().toISOString(),
    memoryUsage: process.memoryUsage(),
    uptime: process.uptime(),
  });
}

// Para logs de performance
function logPerformance(operation, startTime, metadata = {}) {
  const duration = Date.now() - startTime;
  const level = duration > 1000 ? 'warn' : duration > 500 ? 'info' : 'debug';

  logger[level](`Performance: ${operation}`, {
    operation,
    duration: `${duration}ms`,
    slow: duration > 1000,
    ...metadata,
  });
}

// Ejemplo de uso de performance logging
async function exampleSlowOperation() {
  const start = Date.now();

  try {
    // operación lenta...
    await someSlowOperation();

    logPerformance('exampleSlowOperation', start, {
      success: true,
    });
  } catch (error) {
    logPerformance('exampleSlowOperation', start, {
      success: false,
      error: error.message,
    });
    throw error;
  }
}

export {
  requestLogger,
  getDevicesHandler,
  onWebSocketConnection,
  broadcastMessage,
  onDeviceConnect,
  updateDevicePosition,
  sendCommandToDevice,
  handleDeviceError,
  connectToROS,
  publishToTopic,
  onROSMessage,
  missionLogger,
  WebsocketControllerWithLogging,
  debugWithContext,
  logPerformance,
  exampleSlowOperation,
};

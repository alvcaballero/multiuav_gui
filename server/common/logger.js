/**
 * Configuración de Logger usando Winston y Chalk
 * Proporciona logging estructurado con colores y diferentes niveles
 */

import winston from 'winston';
import chalk from 'chalk';
import { fileURLToPath } from 'url';
import { dirname, join } from 'path';

const __filename = fileURLToPath(import.meta.url);
const __dirname = dirname(__filename);

// Configuración de colores personalizados con chalk
const colorizeLevel = {
  error: chalk.red.bold,
  warn: chalk.yellow.bold,
  info: chalk.blue.bold,
  http: chalk.green,
  verbose: chalk.cyan,
  debug: chalk.magenta,
  silly: chalk.gray,
};

// Formato para archivos (sin colores)
const fileFormat = winston.format.combine(
  winston.format.timestamp({ format: 'YYYY-MM-DD HH:mm:ss' }),
  winston.format.errors({ stack: true }),
  winston.format.json()
);

// Transports base compartidos (error.log + combined.log)
const baseFileTransports = [
  new winston.transports.File({
    filename: join(__dirname, '../logs/error.log'),
    level: 'error',
    format: fileFormat,
    maxsize: 5242880, // 5MB
    maxFiles: 5,
  }),
  new winston.transports.File({
    filename: join(__dirname, '../logs/combined.log'),
    format: fileFormat,
    maxsize: 5242880,
    maxFiles: 5,
  }),
];

/**
 * Crea un logger con label/contexto específico.
 *
 * @param {object} options
 * @param {string} [options.label]        - Label visible en consola, ej: 'WEBSOCKET'
 * @param {Function} [options.color]      - Función chalk para colorear el label
 * @param {string} [options.level]        - Nivel mínimo de log (default: process.env.LOG_LEVEL || 'info')
 * @param {string} [options.filename]     - Archivo de log adicional (relativo a logs/)
 * @param {string} [options.envLevelKey]  - Variable de entorno para override del nivel
 * @returns {winston.Logger}
 */
const createLogger = ({ label = null, color = chalk.blue, level = null, filename = null, envLevelKey = null } = {}) => {
  const resolvedLevel = (envLevelKey && process.env[envLevelKey]) || level || process.env.LOG_LEVEL || 'info';

  const consoleFormat = winston.format.combine(
    winston.format.timestamp({ format: 'YYYY-MM-DD HH:mm:ss' }),
    winston.format.errors({ stack: true }),
    winston.format.printf(({ timestamp, level: lvl, message, stack, label: msgLabel, deviceId, deviceName, topic, nodeId, ...meta }) => {
      const colorizedLevel = colorizeLevel[lvl] ? colorizeLevel[lvl](lvl.toUpperCase()) : lvl.toUpperCase();
      const colorizedTimestamp = chalk.gray(timestamp);
      const colorizedMessage = lvl === 'error' ? chalk.red(message) : message;

      let prefix = '';
      if (label || msgLabel) {
        prefix = color.bold(`[${label || msgLabel}] `);
      }

      // Campos contextuales opcionales (device, ros topic, etc.)
      let contextInfo = '';
      if (deviceId || deviceName) contextInfo += chalk.green(`[${deviceName || deviceId}] `);
      if (nodeId) contextInfo += chalk.green(`[${nodeId}] `);
      if (topic) contextInfo += chalk.blue(`[${topic}] `);

      let logLine = `${colorizedTimestamp} ${prefix}[${colorizedLevel}]: ${contextInfo}${colorizedMessage}`;

      if (Object.keys(meta).length > 0) {
        logLine += ` ${chalk.gray(JSON.stringify(meta))}`;
      }

      if (stack) {
        logLine += `\n${chalk.red(stack)}`;
      }

      return logLine;
    })
  );

  const transports = [
    new winston.transports.Console({
      level: resolvedLevel,
      format: consoleFormat,
    }),
    ...baseFileTransports,
  ];

  if (filename) {
    transports.push(
      new winston.transports.File({
        filename: join(__dirname, `../logs/${filename}`),
        format: fileFormat,
        maxsize: 5242880,
        maxFiles: 3,
      })
    );
  }

  return winston.createLogger({
    level: resolvedLevel,
    transports,
    exitOnError: false,
  });
};

// Logger principal (sin label, maneja excepciones globales)
const logger = winston.createLogger({
  level: process.env.LOG_LEVEL || 'info',
  transports: [
    new winston.transports.Console({
      level: process.env.LOG_LEVEL || 'info',
      format: winston.format.combine(
        winston.format.timestamp({ format: 'YYYY-MM-DD HH:mm:ss' }),
        winston.format.errors({ stack: true }),
        winston.format.printf(({ timestamp, level: lvl, message, stack, ...meta }) => {
          const colorizedLevel = colorizeLevel[lvl] ? colorizeLevel[lvl](lvl.toUpperCase()) : lvl.toUpperCase();
          const colorizedTimestamp = chalk.gray(timestamp);
          const colorizedMessage = lvl === 'error' ? chalk.red(message) : message;

          let logLine = `${colorizedTimestamp} [${colorizedLevel}]: ${colorizedMessage}`;

          if (Object.keys(meta).length > 0) {
            logLine += ` ${chalk.gray(JSON.stringify(meta))}`;
          }

          if (stack) {
            logLine += `\n${chalk.red(stack)}`;
          }

          return logLine;
        })
      ),
    }),
    ...baseFileTransports,
  ],
  exitOnError: false,
  handleExceptions: true,
  handleRejections: true,
});

// Loggers con contexto — creados con la factory unificada
const wsLogger = createLogger({
  label: 'WEBSOCKET',
  color: chalk.cyan,
  envLevelKey: 'WS_LOG_LEVEL',
  filename: 'websocket.log',
});

const deviceLogger = createLogger({
  label: 'DEVICE',
  color: chalk.yellow,
  envLevelKey: 'DEVICE_LOG_LEVEL',
  filename: 'devices.log',
});

const rosLogger = createLogger({
  label: 'ROS',
  color: chalk.magenta,
  envLevelKey: 'ROS_LOG_LEVEL',
  filename: 'ros.log',
});

const chatLogger = createLogger({
  label: 'CHAT',
  color: chalk.green,
  envLevelKey: 'CHAT_LOG_LEVEL',
  filename: 'chat.log',
});

const missionLogger = createLogger({
  label: 'MISSION',
  color: chalk.blue,
  envLevelKey: 'MISSION_LOG_LEVEL',
  filename: 'mission.log',
});

// Funciones helper para logging fácil
const logHelpers = {
  // Helper para logs de sistema
  system: {
    info: (message, meta = {}) => logger.info(message, { category: 'system', ...meta }),
    warn: (message, meta = {}) => logger.warn(message, { category: 'system', ...meta }),
    error: (message, error = null, meta = {}) => {
      if (error) {
        logger.error(message, { category: 'system', error: error.message, stack: error.stack, ...meta });
      } else {
        logger.error(message, { category: 'system', ...meta });
      }
    },
    debug: (message, meta = {}) => logger.debug(message, { category: 'system', ...meta }),
  },

  // Helper para logs de API
  api: {
    request: (method, url, ip, meta = {}) => {
      logger.info(`${chalk.green(method)} ${url}`, {
        category: 'api',
        method,
        url,
        ip,
        type: 'request',
        ...meta,
      });
    },
    response: (method, url, statusCode, duration, meta = {}) => {
      const statusColor = statusCode >= 400 ? chalk.red : statusCode >= 300 ? chalk.yellow : chalk.green;
      logger.info(`${chalk.green(method)} ${url} ${statusColor(statusCode)} - ${duration}ms`, {
        category: 'api',
        method,
        url,
        statusCode,
        duration,
        type: 'response',
        ...meta,
      });
    },
    error: (method, url, error, meta = {}) => {
      logger.error(`${chalk.green(method)} ${url} - API Error`, {
        category: 'api',
        method,
        url,
        error: error.message,
        stack: error.stack,
        type: 'error',
        ...meta,
      });
    },
  },

  // Helper para logs de WebSocket
  ws: {
    connect: (clientId, meta = {}) =>
      wsLogger.info(`Client connected: ${chalk.green(clientId)}`, { clientId, type: 'connect', ...meta }),
    disconnect: (clientId, meta = {}) =>
      wsLogger.info(`Client disconnected: ${chalk.red(clientId)}`, { clientId, type: 'disconnect', ...meta }),
    message: (clientId, message, meta = {}) =>
      wsLogger.debug(`Message from ${chalk.green(clientId)}: ${message}`, { clientId, type: 'message', ...meta }),
    broadcast: (message, clientCount, meta = {}) =>
      wsLogger.info(`Broadcasting to ${chalk.yellow(clientCount)} clients: ${message}`, {
        clientCount,
        type: 'broadcast',
        ...meta,
      }),
    error: (clientId, error, meta = {}) =>
      wsLogger.error(`WebSocket error for ${chalk.green(clientId)}`, {
        clientId,
        error: error.message,
        stack: error.stack,
        type: 'error',
        ...meta,
      }),
  },

  // Helper para logs de dispositivos
  device: {
    connect: (deviceId, deviceName, meta = {}) =>
      deviceLogger.info(`Device connected`, { deviceId, deviceName, type: 'connect', ...meta }),
    disconnect: (deviceId, deviceName, meta = {}) =>
      deviceLogger.info(`Device disconnected`, { deviceId, deviceName, type: 'disconnect', ...meta }),
    position: (deviceId, deviceName, position, meta = {}) =>
      deviceLogger.debug(`Position update`, { deviceId, deviceName, position, type: 'position', ...meta }),
    command: (deviceId, deviceName, command, meta = {}) =>
      deviceLogger.info(`Command sent: ${command}`, { deviceId, deviceName, command, type: 'command', ...meta }),
    status: (deviceId, deviceName, status, meta = {}) =>
      deviceLogger.info(`Status update: ${status}`, { deviceId, deviceName, status, type: 'status', ...meta }),
    error: (deviceId, deviceName, error, meta = {}) =>
      deviceLogger.error(`Device error`, {
        deviceId,
        deviceName,
        error: error.message,
        stack: error.stack,
        type: 'error',
        ...meta,
      }),
  },

  // Helper para logs de ROS
  ros: {
    connect: (nodeId, meta = {}) => rosLogger.info(`ROS node connected`, { nodeId, type: 'connect', ...meta }),
    disconnect: (nodeId, meta = {}) => rosLogger.info(`ROS node disconnected`, { nodeId, type: 'disconnect', ...meta }),
    publish: (topic, nodeId, meta = {}) =>
      rosLogger.debug(`Published to topic`, { topic, nodeId, type: 'publish', ...meta }),
    subscribe: (topic, nodeId, meta = {}) =>
      rosLogger.debug(`Subscribed to topic`, { topic, nodeId, type: 'subscribe', ...meta }),
    message: (topic, nodeId, messageType, meta = {}) =>
      rosLogger.debug(`Message received`, { topic, nodeId, messageType, type: 'message', ...meta }),
    error: (nodeId, error, meta = {}) =>
      rosLogger.error(`ROS error`, { nodeId, error: error.message, stack: error.stack, type: 'error', ...meta }),
  },
};

// Exportar loggers y utilidades
export { logger, wsLogger, deviceLogger, rosLogger, chatLogger, missionLogger, logHelpers, createLogger, colorizeLevel, chalk };

// createCustomLogger es alias de createLogger para backwards compatibility
export const createCustomLogger = createLogger;

// Export default como el logger principal
export default logger;

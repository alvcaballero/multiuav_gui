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

// Formato personalizado para consola con colores
const consoleFormat = winston.format.combine(
  winston.format.timestamp({ format: 'YYYY-MM-DD HH:mm:ss' }),
  winston.format.errors({ stack: true }),
  winston.format.printf(({ timestamp, level, message, stack, ...meta }) => {
    const colorizedLevel = colorizeLevel[level] ? colorizeLevel[level](level.toUpperCase()) : level.toUpperCase();
    const colorizedTimestamp = chalk.gray(timestamp);
    const colorizedMessage = level === 'error' ? chalk.red(message) : message;

    let logLine = `${colorizedTimestamp} [${colorizedLevel}]: ${colorizedMessage}`;

    // Agregar metadatos si existen
    if (Object.keys(meta).length > 0) {
      logLine += ` ${chalk.gray(JSON.stringify(meta))}`;
    }

    // Agregar stack trace para errores
    if (stack) {
      logLine += `\n${chalk.red(stack)}`;
    }

    return logLine;
  })
);

// Formato para archivos (sin colores)
const fileFormat = winston.format.combine(
  winston.format.timestamp({ format: 'YYYY-MM-DD HH:mm:ss' }),
  winston.format.errors({ stack: true }),
  winston.format.json()
);

// Configuración de transports
const transports = [
  // Consola con colores
  new winston.transports.Console({
    level: process.env.LOG_LEVEL || 'info',
    format: consoleFormat,
  }),

  // Archivo para errores
  new winston.transports.File({
    filename: join(__dirname, '../logs/error.log'),
    level: 'error',
    format: fileFormat,
    maxsize: 5242880, // 5MB
    maxFiles: 5,
  }),

  // Archivo para todos los logs
  new winston.transports.File({
    filename: join(__dirname, '../logs/combined.log'),
    format: fileFormat,
    maxsize: 5242880, // 5MB
    maxFiles: 5,
  }),
];

// Crear el logger principal
const logger = winston.createLogger({
  level: process.env.LOG_LEVEL || 'info',
  transports,
  exitOnError: false,
  // Manejar excepciones no capturadas
  handleExceptions: true,
  handleRejections: true,
});

// Logger específico para WebSocket
const wsLogger = winston.createLogger({
  level: process.env.WS_LOG_LEVEL || 'info',
  format: winston.format.combine(
    winston.format.label({ label: 'WEBSOCKET' }),
    winston.format.timestamp({ format: 'YYYY-MM-DD HH:mm:ss' }),
    winston.format.printf(({ timestamp, level, message, label }) => {
      const colorizedLevel = colorizeLevel[level] ? colorizeLevel[level](level.toUpperCase()) : level.toUpperCase();
      const colorizedLabel = chalk.cyan.bold(`[${label}]`);
      const colorizedTimestamp = chalk.gray(timestamp);
      return `${colorizedTimestamp} ${colorizedLabel} [${colorizedLevel}]: ${message}`;
    })
  ),
  transports: [
    new winston.transports.Console(),
    new winston.transports.File({
      filename: join(__dirname, '../logs/websocket.log'),
      format: fileFormat,
      maxsize: 5242880,
      maxFiles: 3,
    }),
  ],
});

// Logger específico para dispositivos/drones
const deviceLogger = winston.createLogger({
  level: process.env.DEVICE_LOG_LEVEL || 'info',
  format: winston.format.combine(
    winston.format.label({ label: 'DEVICE' }),
    winston.format.timestamp({ format: 'YYYY-MM-DD HH:mm:ss' }),
    winston.format.printf(({ timestamp, level, message, label, deviceId, deviceName, ...meta }) => {
      const colorizedLevel = colorizeLevel[level] ? colorizeLevel[level](level.toUpperCase()) : level.toUpperCase();
      const colorizedLabel = chalk.yellow.bold(`[${label}]`);
      const colorizedTimestamp = chalk.gray(timestamp);

      let deviceInfo = '';
      if (deviceId || deviceName) {
        deviceInfo = chalk.green(`[${deviceName || deviceId}] `);
      }

      let logLine = `${colorizedTimestamp} ${colorizedLabel} [${colorizedLevel}]: ${deviceInfo}${message}`;

      if (Object.keys(meta).length > 0) {
        logLine += ` ${chalk.gray(JSON.stringify(meta))}`;
      }

      return logLine;
    })
  ),
  transports: [
    new winston.transports.Console(),
    new winston.transports.File({
      filename: join(__dirname, '../logs/devices.log'),
      format: fileFormat,
      maxsize: 5242880,
      maxFiles: 3,
    }),
  ],
});

// Logger específico para ROS
const rosLogger = winston.createLogger({
  level: process.env.ROS_LOG_LEVEL || 'info',
  format: winston.format.combine(
    winston.format.label({ label: 'ROS' }),
    winston.format.timestamp({ format: 'YYYY-MM-DD HH:mm:ss' }),
    winston.format.printf(({ timestamp, level, message, label, topic, nodeId, ...meta }) => {
      const colorizedLevel = colorizeLevel[level] ? colorizeLevel[level](level.toUpperCase()) : level.toUpperCase();
      const colorizedLabel = chalk.magenta.bold(`[${label}]`);
      const colorizedTimestamp = chalk.gray(timestamp);

      let topicInfo = '';
      if (topic) {
        topicInfo = chalk.blue(`[${topic}] `);
      }

      let nodeInfo = '';
      if (nodeId) {
        nodeInfo = chalk.green(`[${nodeId}] `);
      }

      let logLine = `${colorizedTimestamp} ${colorizedLabel} [${colorizedLevel}]: ${nodeInfo}${topicInfo}${message}`;

      if (Object.keys(meta).length > 0) {
        logLine += ` ${chalk.gray(JSON.stringify(meta))}`;
      }

      return logLine;
    })
  ),
  transports: [
    new winston.transports.Console(),
    new winston.transports.File({
      filename: join(__dirname, '../logs/ros.log'),
      format: fileFormat,
      maxsize: 5242880,
      maxFiles: 3,
    }),
  ],
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

// Función para crear logger personalizado
const createCustomLogger = (label, options = {}) => {
  const defaultOptions = {
    level: 'info',
    color: chalk.blue,
    filename: `${label.toLowerCase()}.log`,
    ...options,
  };

  return winston.createLogger({
    level: defaultOptions.level,
    format: winston.format.combine(
      winston.format.label({ label: label.toUpperCase() }),
      winston.format.timestamp({ format: 'YYYY-MM-DD HH:mm:ss' }),
      winston.format.printf(({ timestamp, level, message, label: logLabel, ...meta }) => {
        const colorizedLevel = colorizeLevel[level] ? colorizeLevel[level](level.toUpperCase()) : level.toUpperCase();
        const colorizedLabel = defaultOptions.color.bold(`[${logLabel}]`);
        const colorizedTimestamp = chalk.gray(timestamp);

        let logLine = `${colorizedTimestamp} ${colorizedLabel} [${colorizedLevel}]: ${message}`;

        if (Object.keys(meta).length > 0) {
          logLine += ` ${chalk.gray(JSON.stringify(meta))}`;
        }

        return logLine;
      })
    ),
    transports: [
      new winston.transports.Console(),
      new winston.transports.File({
        filename: join(__dirname, `../logs/${defaultOptions.filename}`),
        format: fileFormat,
        maxsize: 5242880,
        maxFiles: 3,
      }),
    ],
  });
};

// Exportar loggers y utilidades
export { logger, wsLogger, deviceLogger, rosLogger, logHelpers, createCustomLogger, colorizeLevel, chalk };

// Export default como el logger principal
export default logger;

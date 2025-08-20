import logger, { logHelpers } from '../common/logger.js';
import morgan from 'morgan'; // Renamed to avoid conflict with our logger

// Middleware para logging de requests usando nuestro logger
function requestLoggingMiddleware(req, res, next) {
  const start = Date.now();

  // Log del request entrante
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

export function setupLogger(app) {
  app.use(requestLoggingMiddleware);

  // Mantener morgan para logs básicos si es necesario
  app.use(
    morgan('combined', {
      stream: {
        write: (message) => logger.info(message.trim(), { source: 'morgan' }),
      },
    })
  );

  // Configurar manejo de errores con logging
  app.use((err, req, res, next) => {
    logHelpers.api.error(req.method, req.url, err, {
      stack: err.stack,
      statusCode: err.status || 500,
    });

    res.status(err.status || 500).json({
      error: process.env.NODE_ENV === 'production' ? 'Internal Server Error' : err.message,
    });
  });
}

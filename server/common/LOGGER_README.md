# Guía de Integración del Logger

## Instalación

El logger ya está configurado e instalado. Las dependencias necesarias son:

- `winston`: Para el sistema de logging estructurado
- `chalk`: Para colores en la consola

## Importación

```javascript
// Importar el logger principal
import logger from './utils/logger.js';

// Importar loggers específicos y helpers
import { logHelpers, wsLogger, deviceLogger, rosLogger } from './utils/logger.js';
```

## Configuración de Variables de Entorno

Agrega estas variables a tu archivo `.env`:

```bash
# Niveles de logging
LOG_LEVEL=info
WS_LOG_LEVEL=info
DEVICE_LOG_LEVEL=info
ROS_LOG_LEVEL=info
```

## Uso Rápido

### 1. Logging Básico

```javascript
import logger from './utils/logger.js';

logger.info('Servidor iniciado');
logger.warn('Advertencia');
logger.error('Error ocurrido', { error: errorObject });
logger.debug('Información de debug');
```

### 2. Logging de API con Helpers

```javascript
import { logHelpers } from './utils/logger.js';

// En middleware de Express
logHelpers.api.request(req.method, req.url, req.ip);
logHelpers.api.response(req.method, req.url, statusCode, duration);
logHelpers.api.error(req.method, req.url, error);
```

### 3. Logging de WebSocket

```javascript
import { logHelpers } from './utils/logger.js';

logHelpers.ws.connect(clientId);
logHelpers.ws.disconnect(clientId);
logHelpers.ws.message(clientId, message);
logHelpers.ws.broadcast(message, clientCount);
```

### 4. Logging de Dispositivos

```javascript
import { logHelpers } from './utils/logger.js';

logHelpers.device.connect(deviceId, deviceName);
logHelpers.device.position(deviceId, deviceName, position);
logHelpers.device.command(deviceId, deviceName, command);
logHelpers.device.error(deviceId, deviceName, error);
```

### 5. Logging de ROS

```javascript
import { logHelpers } from './utils/logger.js';

logHelpers.ros.connect(nodeId);
logHelpers.ros.publish(topic, nodeId);
logHelpers.ros.message(topic, nodeId, messageType);
logHelpers.ros.error(nodeId, error);
```

## Archivos de Log Generados

- `logs/combined.log`: Todos los logs
- `logs/error.log`: Solo errores
- `logs/websocket.log`: Logs específicos de WebSocket
- `logs/devices.log`: Logs específicos de dispositivos
- `logs/ros.log`: Logs específicos de ROS

## Integración en Archivos Existentes

### 1. server.js

Reemplaza `console.log` con:

```javascript
import logger from './utils/logger.js';

// En lugar de: console.log('init RosEnable');
logger.info('ROS habilitado, iniciando conexión');

// En lugar de: console.log('init FbEnable');
logger.info('FlatBuffer habilitado, iniciando WebSocket de dispositivos');
```

### 2. WebsocketManager.js

```javascript
import { logHelpers } from './utils/logger.js';

// En lugar de: console.log('received: %s', message.toString());
logHelpers.ws.message(clientId, message.toString());

// En lugar de: console.log('New client connected');
logHelpers.ws.connect(clientId);
```

### 3. WebsocketDevices.js

```javascript
import { logHelpers } from './utils/logger.js';

// En lugar de: console.log('New device is connected');
logHelpers.device.connect(deviceId, deviceName);

// En lugar de: console.log('received message device %s on topic: %s - %s', deviceName, metadata.topic(), metadata.type());
logHelpers.ros.message(metadata.topic(), deviceName, metadata.type());
```

## Ventajas del Nuevo Sistema de Logging

1. **Colores en consola**: Fácil identificación visual de tipos de log
2. **Logging estructurado**: Metadatos organizados en JSON para archivos
3. **Rotación de archivos**: Previene que los logs crezcan indefinidamente
4. **Loggers específicos**: Diferentes loggers para diferentes componentes
5. **Configuración flexible**: Control via variables de entorno
6. **Helpers especializados**: Funciones predefinidas para casos comunes
7. **Captura de errores**: Manejo automático de excepciones no capturadas

## Migración Gradual

Puedes migrar gradualmente:

1. Importa el logger en archivos específicos
2. Reemplaza `console.log` por `logger.info`
3. Reemplaza `console.error` por `logger.error`
4. Usa helpers específicos para casos especiales

## Ejemplos de Salida

### Consola (con colores)

```
2025-01-20 10:30:15 [INFO]: Servidor iniciado en puerto 4000
2025-01-20 10:30:16 [WEBSOCKET] [INFO]: Client connected: client-123
2025-01-20 10:30:17 [DEVICE] [INFO]: [M300-drone] Device connected
2025-01-20 10:30:18 [ROS] [DEBUG]: [/mavros/global_position/global] Message received
```

### Archivo (JSON estructurado)

```json
{
  "timestamp": "2025-01-20 10:30:15",
  "level": "info",
  "message": "Servidor iniciado en puerto 4000",
  "category": "system",
  "port": 4000
}
```

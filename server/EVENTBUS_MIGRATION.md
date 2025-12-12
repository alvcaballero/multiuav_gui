# Migración a EventBus - Sistema de Eventos Desacoplado

## 📋 Resumen de Cambios

Se ha implementado una arquitectura basada en **EventEmitter** para desacoplar la comunicación entre componentes del sistema, eliminando dependencias directas entre modelos y el transporte WebSocket.

---

## 🎯 Objetivos Logrados

### ✅ Desacoplamiento Total
- Los modelos ya no dependen de `getWebsocketController()`
- La lógica de negocio está separada del transporte
- Fácil de testear con mocks

### ✅ Extensibilidad
- Agregar nuevos canales de comunicación (webhooks, MQTT, etc.) es trivial
- Solo se necesita crear un nuevo subscriber

### ✅ Centralización
- Un solo punto de gestión de eventos (EventBus)
- Logging centralizado de todos los eventos
- Fácil debugging y observabilidad

### ✅ Robustez
- Manejo de errores en todos los listeners
- Cleanup automático en shutdown
- Prevención de memory leaks

---

## 📁 Archivos Creados

### 1. `/server/common/eventBus.js`
**Sistema central de eventos basado en EventEmitter.**

**Características:**
- Singleton exportado como `eventBus`
- Método `emitSafe()` con manejo de errores
- Método `onSafe()` para registrar listeners seguros
- Logging automático de todos los eventos
- Estadísticas y cleanup

**Eventos disponibles:**
```javascript
EVENTS.MISSION_CREATED       // Nueva misión creada
EVENTS.MISSION_UPDATED       // Misión actualizada
EVENTS.MISSION_INIT          // Misión inicializada
EVENTS.EVENT_CREATED         // Nuevo evento del sistema
EVENTS.POSITION_UPDATED      // Posición actualizada
EVENTS.CAMERA_UPDATED        // Cámara actualizada
EVENTS.DEVICE_UPDATED        // Dispositivo actualizado
EVENTS.SERVER_UPDATED        // Estado del servidor actualizado
```

### 2. `/server/subscribers/websocketSubscriber.js`
**Subscriber que escucha eventos y los envía por WebSocket.**

**Responsabilidades:**
- Escucha eventos del EventBus
- Transforma eventos a mensajes WebSocket
- Maneja el envío a clientes conectados
- Solo envía datos cuando hay información relevante

**Métodos principales:**
- `onMissionCreated()` - Maneja creación de misiones
- `onMissionInit()` - Maneja inicialización de misiones
- `onEventCreated()` - Maneja eventos del sistema
- `cleanup()` - Limpia todas las suscripciones

---

## 🔧 Archivos Modificados

### 1. `/server/models/events.js`
**Antes:**
```javascript
const ws = getWebsocketController();
ws.sendMessage(JSON.stringify({ events: [myEvent] }));
```

**Después:**
```javascript
import { eventBus, EVENTS } from '../common/eventBus.js';
eventBus.emitSafe(EVENTS.EVENT_CREATED, myEvent);
```

**Cambios:**
- ❌ Eliminado: `import { getWebsocketController }`
- ❌ Eliminado: `import { WebsocketManager }`
- ✅ Agregado: `import { eventBus, EVENTS }`
- ✅ Cambiado: Emisión de eventos en lugar de envío directo

---

### 2. `/server/models/mission.js`
**Antes (2 lugares):**
```javascript
const ws = getWebsocketController();
ws.sendMessage(JSON.stringify({ mission: { ...mission, name: 'name' } }));
```

**Después:**
```javascript
import { eventBus, EVENTS } from '../common/eventBus.js';

// En setMission()
eventBus.emitSafe(EVENTS.MISSION_CREATED, { ...mission, name: 'name' });

// En initMission()
eventBus.emitSafe(EVENTS.MISSION_INIT, { ...mission, name: 'name' });
```

**Cambios:**
- ❌ Eliminado: `import { getWebsocketController }`
- ✅ Agregado: `import { eventBus, EVENTS }`
- ✅ Cambiado: Emisión de eventos específicos según el contexto

---

### 3. `/server/controllers/websocket.js`
**Mejoras agregadas:**

```javascript
// 1. Importar logger
import logger from '../common/logger.js';

// 2. Manejo de errores en updates
async updateclient() {
  try {
    const msg = await this.updateMessage();
    // Solo enviar si hay datos
    if (Object.keys(msg).length > 0) {
      this.sendMessage(msg, null);
    }
  } catch (error) {
    logger.error('Error in updateclient', { error: error.message });
  }
}

// 3. Método de cleanup
destroy() {
  logger.info('websocketController cleanup');
  clearInterval(this.interval_update);
  clearInterval(this.interval_server);
}
```

**Beneficios:**
- ✅ No envía objetos vacíos `{}`
- ✅ Captura errores en callbacks async
- ✅ Cleanup apropiado en shutdown

---

### 4. `/server/server.js`
**Cambios principales:**

```javascript
// 1. Importar sistema de eventos
import { WebSocketSubscriber } from './subscribers/websocketSubscriber.js';
import { eventBus } from './common/eventBus.js';

// 2. Inicializar subscriber
const wsSubscriber = new WebSocketSubscriber(websocketController);
logger.info('EventBus system initialized', {
  subscribers: ['WebSocketSubscriber']
});

// 3. Cleanup en shutdown
process.on('SIGTERM', () => {
  wsSubscriber.cleanup();
  websocketController.destroy();
  eventBus.cleanup();
  // ... resto del código
});

process.on('SIGINT', () => {
  wsSubscriber.cleanup();
  websocketController.destroy();
  eventBus.cleanup();
  // ... resto del código
});
```

**Beneficios:**
- ✅ Inicialización centralizada
- ✅ Graceful shutdown completo
- ✅ Prevención de memory leaks

---

### 5. `/server/WebsocketManager.js`
**Limpieza de código:**

```javascript
// ❌ ELIMINADO: Código no utilizado
- this.clientsToRoom = new Map();  // Nunca usado
- this.clients = new Set();         // Nunca usado
- broadcastToRoom()                 // Nunca implementado
```

**Beneficios:**
- ✅ Código más limpio
- ✅ Menos confusión
- ✅ Menor memoria utilizada

---

## 🔄 Flujo de Datos (Nueva Arquitectura)

```
┌─────────────────────────────────────────────────────────────┐
│                         MODELOS                             │
│  (eventsModel, missionModel, etc.)                          │
└─────────────────┬───────────────────────────────────────────┘
                  │
                  │ eventBus.emitSafe(EVENTS.XXX, data)
                  │
                  ▼
┌─────────────────────────────────────────────────────────────┐
│                       EVENT BUS                             │
│  - Centraliza comunicación                                  │
│  - Logging automático                                       │
│  - Manejo de errores                                        │
└─────────────────┬───────────────────────────────────────────┘
                  │
                  │ event listeners
                  │
                  ▼
┌─────────────────────────────────────────────────────────────┐
│                     SUBSCRIBERS                             │
│  - WebSocketSubscriber  → Envía a clientes WS              │
│  - (Futuro) LogSubscriber → Guarda en DB                   │
│  - (Futuro) WebhookSubscriber → Notifica externos          │
└─────────────────┬───────────────────────────────────────────┘
                  │
                  ▼
                Transporte (WebSocket, HTTP, etc.)
```

---

## 🚀 Cómo Agregar Nuevos Subscribers

### Ejemplo: LogSubscriber para guardar eventos en archivos

```javascript
// server/subscribers/logSubscriber.js
import { eventBus, EVENTS } from '../common/eventBus.js';
import fs from 'fs/promises';

export class LogSubscriber {
  constructor(logPath) {
    this.logPath = logPath;
    this.setupSubscriptions();
  }

  setupSubscriptions() {
    eventBus.onSafe(EVENTS.MISSION_CREATED, this.onMissionCreated.bind(this));
    eventBus.onSafe(EVENTS.EVENT_CREATED, this.onEventCreated.bind(this));
  }

  async onMissionCreated(mission) {
    await fs.appendFile(
      this.logPath,
      `[MISSION] ${new Date().toISOString()} - ${JSON.stringify(mission)}\n`
    );
  }

  async onEventCreated(event) {
    await fs.appendFile(
      this.logPath,
      `[EVENT] ${new Date().toISOString()} - ${JSON.stringify(event)}\n`
    );
  }

  cleanup() {
    // Implementar cleanup si es necesario
  }
}

// En server.js:
import { LogSubscriber } from './subscribers/logSubscriber.js';
const logSubscriber = new LogSubscriber('./logs/events.log');
```

---

## 📊 Comparación: Antes vs Después

| Aspecto | Antes | Después |
|---------|-------|---------|
| **Acoplamiento** | Alto (modelos → WS) | Bajo (modelos → eventos) |
| **Testabilidad** | Difícil (mock singleton) | Fácil (mock EventEmitter) |
| **Extensibilidad** | Modificar cada lugar | Agregar subscriber |
| **Observabilidad** | Dispersa en logs | Centralizada en EventBus |
| **Manejo de Errores** | Incompleto | Completo con try/catch |
| **Memory Leaks** | Posibles | Prevenidos con cleanup |
| **Código Duplicado** | Serialización dispersa | Centralizada en subscribers |

---

## 🧪 Testing

### Test de EventBus
```javascript
import { eventBus, EVENTS } from '../common/eventBus.js';

describe('EventBus', () => {
  it('should emit events safely', () => {
    let received = null;
    eventBus.on(EVENTS.MISSION_CREATED, (data) => {
      received = data;
    });

    eventBus.emitSafe(EVENTS.MISSION_CREATED, { id: 1, name: 'test' });

    expect(received).toEqual({ id: 1, name: 'test' });
  });
});
```

### Test de Models (ahora más fácil)
```javascript
import { eventsModel } from '../models/events.js';
import { eventBus, EVENTS } from '../common/eventBus.js';

describe('eventsModel', () => {
  it('should emit event when creating', async () => {
    const spy = jest.spyOn(eventBus, 'emitSafe');

    await eventsModel.addEvent({ type: 'info', deviceId: 1 });

    expect(spy).toHaveBeenCalledWith(
      EVENTS.EVENT_CREATED,
      expect.objectContaining({ type: 'info' })
    );
  });
});
```

---

## 🐛 Bugs Corregidos

### 1. WebsocketManager.js
- ✅ Eliminado código muerto (`clientsToRoom`, `clients`)
- ✅ Eliminado método no implementado (`broadcastToRoom`)

### 2. websocketController
- ✅ Agregado manejo de errores en `updateclient()` y `updateserver()`
- ✅ Agregado método `destroy()` para cleanup
- ✅ Evita enviar objetos vacíos `{}`

### 3. Graceful Shutdown
- ✅ Limpieza de intervals en SIGTERM/SIGINT
- ✅ Limpieza de EventBus listeners
- ✅ Limpieza de subscribers

---

## 📝 Notas Importantes

1. **Backward Compatibility**: La función `getWebsocketController()` todavía existe pero ya no se usa en models. Si otros archivos la usan, funciona normalmente.

2. **Performance**: El EventBus tiene overhead mínimo. Los eventos son síncronos por defecto (usando EventEmitter de Node.js).

3. **Debugging**: Para ver todos los eventos emitidos, el EventBus tiene logging automático en nivel `debug`.

4. **Extensiones Futuras**: Es trivial agregar:
   - WebhookSubscriber (notificar a sistemas externos)
   - MetricsSubscriber (recolectar métricas de eventos)
   - DatabaseSubscriber (guardar eventos en DB)
   - SlackSubscriber (notificaciones a Slack)

---

## ✅ Checklist de Migración Completada

- [x] EventBus central creado
- [x] WebSocketSubscriber implementado
- [x] eventsModel migrado
- [x] missionModel migrado
- [x] server.js actualizado con inicialización
- [x] Cleanup en graceful shutdown
- [x] Manejo de errores agregado
- [x] Bugs en WebsocketManager corregidos
- [x] Documentación creada

---

## 🎓 Lecciones Aprendidas

### Patrón EventBus (Mediator Pattern)
Este patrón es ideal cuando:
- Tienes múltiples componentes que necesitan comunicarse
- Quieres desacoplar emisores de receptores
- Necesitas agregar nuevos canales de comunicación dinámicamente
- Quieres centralizar logging y observabilidad

### Ventajas sobre Singleton directo
- **Testeable**: Puedes mockear el EventBus fácilmente
- **Flexible**: Agregar/quitar subscribers sin tocar modelos
- **Escalable**: Soporta múltiples subscribers por evento
- **Observable**: Un solo lugar para ver todos los eventos

---

**Autor**: Claude Code
**Fecha**: 2025-12-10
**Versión**: 1.0.0

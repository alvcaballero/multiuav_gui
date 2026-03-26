# Server Refactor TODO

Tareas ordenadas por prioridad. Marcar con `[x]` al completar.

---

## CRÍTICO — Bugs reales

- [x] **FIX: `writeFileSync` en función async** (`common/utils.js:73`)
  - Reemplazar `writeFileSync` de `fs` por `writeFile` de `fs/promises`
  - `writeFileSync` no devuelve Promise — el `await` no hace nada y bloquea el event loop

- [x] **FIX: Validación MCP invertida** (`config/config.js:46`)
  - La condición actual lanza error cuando transport ESTÁ definido (lógica al revés)
  - Validar que `transport` sea uno de `'stdio' | 'http' | 'sse'`

- [x] **FIX: `setMission` miente sobre DB** (`models/mission.js:67-76`)
  - Emite `MISSION_CREATED` y retorna `{ success: true }` pero no persiste en DB
  - Clarificar semántica: o guarda en DB o renombrar el método/evento

---

## ALTO — Deuda técnica activa

- [x] **Sweep global de `console.log`** (351 ocurrencias)
  - Reemplazar todos los `console.log/console.error` por el logger correspondiente
  - Archivos más afectados:
    - `models/commands.js:12-13, 53, 72, 95`
    - `controllers/mission.js:20, 25-26, 31, 48`
    - `models/devices.js:64, 92`
    - `controllers/devices.js:47, 60`
    - `common/utils.js:66, 76`
  - Eliminar logs de debug obvios (ej: `console.log('constructor device model')`)

- [x] **Extraer `decodeMissionMsg`** (`models/commands.js:8-90`)
  - Función de 80 líneas con lógica de negocio + transformación de datos mezcladas
  - Crear clase `CommandDecoder` en `models/commands/CommandDecoder.js`
  - Separar: parsing de formato → validación → transformación a estructura interna

- [x] **Unificar naming de env vars** (`config/config.js`)
  - Standarizar a UPPER_SNAKE_CASE para todas las variables
  - Casos problemáticos:
    - `Files_PATH` → `FILES_PATH`
    - `Process_Thermal_Img` → `PROCESS_THERMAL_IMG`
  - Actualizar `.env.example` y todos los usos

- [x] **Extraer magic numbers a config**
  - `WebsocketManager.js:43` → `PING_INTERVAL_MS = 30000`
  - `models/ros/ros.js:35` → `ROS_RECONNECT_INTERVAL_MS = 30000`
  - `controllers/websocket.js:13` → `POSITIONS_BROADCAST_INTERVAL_MS = 2000`
  - `controllers/websocket.js:14` → `STATE_BROADCAST_INTERVAL_MS = 10000`
  - `models/devices.js:24` → mover `CHECK_INTERVAL` a `config/config.js`
  - Centralizar en `config/config.js` con valores por defecto desde `.env`

- [x] **Invertir dependency direction en chat** (`models/chat/chat.js:10-11`)
  - Eliminado import muerto de `missionController` — no se usaba en ninguna línea
  - Nota: el patrón models→controllers es endémico en toda la codebase (los "controllers"
    actúan como service facades, no como HTTP handlers). Refactor completo requiere
    introducir Repository layer (ver tarea MEDIO)

---

## MEDIO — Calidad y arquitectura

- [x] **Split `rosModel` en clases separadas** (`models/ros/ros.js`)
  - Actualmente: God Object de ~200 líneas con estado global mutable (`var ros = null`)
  - Separar en:
    - `RosConnectionManager` — conexión, reconexión, estado
    - `RosSubscriptionManager` — suscripción a topics, callbacks
    - `RosDeviceRegistry` — mapeo de devices a topics/servicios
  - Eliminar `var` a nivel módulo, encapsular estado

- [x] **Unificar loggers** (`common/logger.js`)
  - Existen 5 instancias separadas: `logger`, `wsLogger`, `deviceLogger`, `chatLogger`
  - Cada una repite 80% del mismo setup de Winston
  - Crear un único logger con soporte de `label`/`context`:
    ```javascript
    const logger = createLogger({ label: 'ros' });
    ```
  - Actualizar todos los imports en el codebase

- [x] **Fix CORS duplicado y security hole** (`middlewares/cors.js`)
  - Líneas 20 y 24 chequean la misma condición `origin.includes('http://10.42.0.')` dos veces
  - Refactorizar a array de patrones permitidos con un `.some()`
  - Revisar que `CORS_ENABLE=false` no habilite acceso universal sin intención

- [ ] **Crear Repository layer para DB**
  - Models acceden a Sequelize directamente (`sequelize.models.Device.findAll(...)`)
  - Crear:
    - `schemas/database/repositories/DeviceRepository.js`
    - `schemas/database/repositories/MissionRepository.js`
  - Exponer métodos nombrados (`findAll`, `findById`, `update`, `delete`)
  - Facilita testing y desacopla lógica de negocio del ORM

- [x] **Lifecycle management para timers** (`models/devices.js:41-73`)
  - Dos `setTimeout` encadenados iniciados al cargar el módulo, no se pueden parar
  - Convertir a clase con `.start()` / `.stop()`
  - Usar `setInterval` con referencia guardada para poder limpiar en shutdown

- [ ] **Fix migraciones en startup** (`common/sequelize.js:61-75`)
  - SQL de migración corre en CADA startup (frágil, detecta errores por string matching)
  - Implementar sistema de versioning simple o usar Sequelize migrations (`sequelize-cli`)
  - Al menos: guardar versión aplicada en tabla `schema_version`

---

## BAJO — Limpieza y pulido

- [ ] **Guard para `devices.at()` sin resultado** (`controllers/devices.js:16`)
  - `Array.isArray(devices) ? devices.at() : devices`
  - Si `devices` es array vacío, `at()` devuelve `undefined` silenciosamente
  - Agregar guard o manejo explícito del caso vacío

- [ ] **Agregar try-catch en funciones async críticas**
  - `models/mission.js:79` — `createMission` sin catch, puede tirar sin contexto
  - `controllers/devices.js:5` — `getDevice` sin catch
  - Patrón consistente: try/catch + logger.error + respuesta HTTP apropiada

- [ ] **Esquema de error HTTP consistente**
  - Actualmente mezcla: `{ error: JSON.parse(...) }`, `{ error: string }`, status codes aleatorios
  - Definir tipo de respuesta de error uniforme: `{ success: false, error: { code, message } }`
  - Crear middleware de error handler global en `middlewares/errorHandler.js`

- [ ] **Limpiar estado global mutable en ROS** (`models/ros/ros.js:14`)
  - `var ros = null` a nivel módulo dificulta testing y razonamiento
  - Encapsular en clase o módulo con acceso controlado (parte del split de rosModel)

- [ ] **Consolidar definiciones de status**
  - Status definidos en: `config/status.js`, `models/devices.js:30-38`, `models/mission.js:26-44`, `schemas/zod/mission.js:3-22`
  - Centralizar en `config/status.js` como fuente única de verdad
  - Importar desde ahí en todos los otros lugares

- [x] **Documentar `GCSunServicesMission`** (`server.js:118`)
  - Método llamado en uncaught exception handler sin documentación de qué hace
  - Verificar que el cleanup sea correcto o agregar comentario explicativo

---

## COMPLETADO

<!-- Mover tareas completadas aquí con fecha -->


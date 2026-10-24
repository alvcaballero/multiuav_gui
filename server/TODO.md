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

- [x] **Watchdog de telemetría: timeout derivado de `rate_hz`** (`models/inspect/telemetryWatchdog.js`)
  - Hoy el timeout de un slot es `timeout_s` de su entrada o `TELEMETRY_SLOT_TIMEOUT_MS` (5 s), e
    IGNORA `rate_hz`, que las categorías MAVLink (`px4_mavlink`, `ardupilot_mavlink`) ya declaran
    como la tasa pedida con SET_MESSAGE_INTERVAL
  - Riesgo: un slot lento (< ~0.5 Hz) sin `timeout_s` oscila `ok`↔`stale` con un par de pérdidas
    en una radio SiK, y cada transición es un Event en la DB + snackbar. `home_position` (0.5 Hz)
    lleva hoy `timeout_s: 10` a mano por esto: si alguien cambia `rate_hz` y no `timeout_s`, se
    desfasan sin que nada avise
  - Propuesta: si la entrada declara `rate_hz` y no `timeout_s`, timeout = max(5 s, 3 / rate_hz)
    (tres períodos). `timeout_s` sigue ganando. Test en `test/test-telemetry-watchdog.js`
  - Al hacerlo: sacar el `timeout_s: 10` explícito de `home_position` en las dos categorías MAVLink
  - Tradeoff: el watchdog pasa a leer una key que hoy solo tiene sentido en MAVLink (en ROS ningún
    slot la declara, así que no cambia nada ahí)

<!-- Devices: lo que quedó afuera a propósito de F5 de MAVLink (2026-10-21, MAVLINK_PLAN.md). Lo que
     SÍ entró: protocolo primero (uno de los 4: ros, mqtt, robofleet, mavlink, cada uno con su
     gateway) y categoría filtrada por el FORMATO DE MENSAJE que ese protocolo transporta (`mavlink`
     o `ros_msg`, deducido de sus messageType: categoryModel.messageFormatOf); System ID solo en
     mavlink; coherencia validada al crear y editar (categoryModel.deviceProblem). -->

- [ ] **Capacidades declaradas por categoría** (gimbal, cámara, descarga de archivos, offboard computer)
  - Hoy se DERIVAN de los endpoints: un device tiene un comando si su categoría declara un endpoint que
    lo implementa (`/api/commands/types`), y `shouldDownloadFiles` mira si ofrece `CameraFileDownload`
  - Declararlas permitiría mostrarlas en la UI y filtrar por ellas
  - Decidir antes qué pasa cuando lo declarado y lo derivado no coinciden (hoy no pueden divergir)

- [ ] **Tipos de vehículo** (UAV multirrotor, UAV ala fija, USV, UGV) como dato de la categoría
  - Lo necesitan el ícono del mapa, el planner y los modos de vuelo
  - En MAVLink el tipo ya viaja en el HEARTBEAT (`MAV_TYPE`); `mavlinkFlightMode` solo mapea ArduPilot
    multirrotor (Plane/Rover numeran distinto y caen en OTHER)

- [ ] **Renombrar `category` → `interface`** (en otra fase, después de capacidades/tipos de vehículo)
  - `category` (`devices_msg.yaml`, `Device.category`) nunca fue una categoría de dispositivo: es la INTERFAZ,
    el conjunto de mensajes + topics + encoders de un stack (`dji_M210_noetic` = DJI OSDK en ROS Noetic)
  - Mientras tanto se queda el nombre y se documenta como interfaz. NO reutilizar "categoría" para el tipo de
    vehículo: cada referencia existente pasaría a mentir sin error
  - Toca DB (migración de columna), API, cliente, MCP (`mcp_server/`) y los prompts de los agentes

- [ ] **MAVLink: el dialecto PX4/ArduPilot no es una interfaz**
  - En MAVLink los mensajes los fija el PROTOCOLO, no la categoría: `px4_mavlink` y `ardupilot_mavlink`
    solo difieren en el tag de dialecto (`mavlink/PX4_*` vs `mavlink/ARDUPILOT_*`) y un subscriber de gimbal,
    y el firmware ya viaja en el HEARTBEAT (`autopilot`)
  - Por ahora se quedan como dos categorías declaradas (la UI y el planner necesitan saberlo sin conexión;
    el HEARTBEAT solo verifica)
  - Revisar cuando aparezca un segundo modelo MAVLink que difiera en algo más que el firmware

- [ ] **Slots de posición satelital: nombres y datum**
  - Hoy `position` es la posición GNSS sin corrección y `position_rtk` la corregida por RTK; el cache usa la de
    mayor prioridad que tenga mensajes recientes y cae a la otra. Definir nombres definitivos (`position_gnss`?)
    al renombrar `category` → `interface`
  - ALTITUD: se toma tal como viene de cada fuente. RTK y GPS pueden reportar en datums distintos (elipsoide vs
    MSL, ~50 m en España): al cambiar de fuente en vuelo puede saltar la altitud en el mapa 3D y en
    `missionSignals`. Verificar con datos reales del M210/M300 antes de confiar en ella
  - Calidad del fix RTK: hoy solo se mira la frescura (+ lat/lon 0). Si el nodo DJI publica `rtk_position` sin
    fix, hay que mirar `status.status` o el topic de estado RTK del OSDK. Hoy `attributes.position_gps|rtk.fix`
    es `'nofix'` solo con lat/lon 0; `NavSatFix.status.status` (-1/0/1/2) no se lee y no está verificado qué rellena el OSDK
  - Salto al volver el RTK: no hay histéresis; al recuperarse gana de inmediato y la posición (y la altitud, ver
    arriba) salta los metros que separan ambas fuentes. Si molesta en el mapa o en el tracking de waypoints,
    exigir N mensajes seguidos antes de volver a RTK
  - Hecho: arbitraje por `priority`/`fresh_ms` de la entrada (`positionReducers.js`) y `attributes.positionSource`.
    El cambio de fuente NO genera Event (fila en DB + snackbar por cada vaivén del fix RTK): si hace falta avisar
    al operador, mejor un chip en el cliente que lea `positionSource` / `position_rtk.at`

- [ ] **Enlace MAVLink por device** (UDP / TCP / serie-SiK)
  - Hoy los enlaces son del SERVER (`MAVLINK_LINKS`) y el vehículo se encuentra por sysid en
    cualquiera, así que el device solo necesita el System ID
  - Atarlo a un enlace serviría para rechazar un sysid que llegue por otro
  - Los enlaces serie siguen sin implementar (`parseLinkSpec` los rechaza) hasta tener las radios

- [x] **Unicidad del System ID entre devices MAVLink**
  - Se valida al crear y al editar (`systemIdTaken`, `controllers/devices.js`): 400 con `path: ['systemId']`
  - `MavlinkServer._lookupDevice` sigue ignorando duplicados: cubre los que ya estaban en la DB antes de la validación

---

## BAJO — Limpieza y pulido

- [x] **Guard para `devices.at()` sin resultado** (`controllers/devices.js:16`)
  - Era peor que el caso vacío: con un id no numérico o `0`, `getAll` devuelve TODOS y `.at()` daba el primer device. Ahora busca por id exacto y devuelve `null`
  - `Array.isArray(devices) ? devices.at() : devices`
  - Si `devices` es array vacío, `at()` devuelve `undefined` silenciosamente
  - Agregar guard o manejo explícito del caso vacío

- [x] **Agregar try-catch en funciones async críticas**
  - `createMission` del model loguea con contexto y relanza; el handler HTTP `POST /missions` responde 500. `getDevice` ya no necesita catch (no lanza)
  - `models/mission.js:79` — `createMission` sin catch, puede tirar sin contexto
  - `controllers/devices.js:5` — `getDevice` sin catch
  - Patrón consistente: try/catch + logger.error + respuesta HTTP apropiada

- [ ] **Esquema de error HTTP consistente**
  - Actualmente mezcla: `{ error: JSON.parse(...) }`, `{ error: string }`, status codes aleatorios
  - Definir tipo de respuesta de error uniforme: `{ success: false, error: { code, message } }`
  - Crear middleware de error handler global en `middlewares/errorHandler.js`

- [x] **Limpiar estado global mutable en ROS** (`models/ros/ros.js:14`)
  - El `var ros` ya no estaba en `ros.js` (el split se hizo) sino en `rosConnection.js`, con 6 variables de módulo. Ahora es la clase `RosConnection`; las funciones exportadas (`getRos`, `rosConnect`…) delegan en una instancia compartida, así `ros.js` no cambió
  - `var ros = null` a nivel módulo dificulta testing y razonamiento
  - Encapsular en clase o módulo con acceso controlado (parte del split de rosModel)

- [x] **Consolidar definiciones de status**
  - Casi todo ya vivía en `config/status.js`; quedaban el enum de `schemas/zod/devices.js`, los `default('init')` de zod, la lista inline de `MISSION_ALIVE_STATUS` y un `'offline'` en `models/mission/mission.js`
  - Se deja afuera `{ state: 'error' }`: es el contrato de respuesta de un comando, no un status de entidad
  - Status definidos en: `config/status.js`, `models/devices.js:30-38`, `models/mission.js:26-44`, `schemas/zod/mission.js:3-22`
  - Centralizar en `config/status.js` como fuente única de verdad
  - Importar desde ahí en todos los otros lugares

- [x] **Documentar `GCSunServicesMission`** (`server.js:118`)
  - Método llamado en uncaught exception handler sin documentación de qué hace
  - Verificar que el cleanup sea correcto o agregar comentario explicativo

---

## COMPLETADO

<!-- Mover tareas completadas aquí con fecha -->


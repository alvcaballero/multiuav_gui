# MAVLink protocol — plan (CERRADO 2026-10-26)

**Estado:** F0–F6 hechas. Lo que queda (enlaces serie/SiK, timeouts por enlace, pruebas en SITL de KillMotors,
gimbal y pausa en PX4, formularios en el navegador) está en `TODO.md` › MAVLink. Este archivo queda como
registro: decisiones, mediciones en SITL y hallazgos; el código lo cita por sección (`› Resultados F0`, `› F4`).
Las secciones de diseño de F1–F6 describen el plan, no necesariamente el código actual: lo vigente está en
`server/CLAUDE.md` › MAVLink Protocol.

Cuarto protocolo de device (`protocol: 'mavlink'`), al lado de `ros`, `robofleet` y `mqtt`.
Objetivo: un drone PX4 o ArduPilot conectado por MAVLink (UDP/TCP o radio SiK por serie) tiene
la misma funcionalidad que un PX4 por ROS2: telemetría en el mapa, estado canónico, comandos de
vuelo, gimbal, misiones con seguimiento de waypoints y el inspector de mensajes raw.

## Decisiones tomadas

| Tema | Decisión |
|---|---|
| Librería | `node-mavlink` 2.x (`github.com/ArduPilot/node-mavlink`, LGPL). Mensajes pregenerados (`minimal`, `common`, `ardupilotmega`) |
| Firmware | PX4 **y** ArduPilot (Copter) desde el principio |
| Enlaces | UDP, TCP y serie (radios SiK) desde el principio → dependencia `serialport` (nativa, trae prebuilds) |
| Identidad GCS | `sysid 255`, `compid 190` (MAV_COMP_ID_MISSIONPLANNER), las mismas que ya usa `vehicleCommand.js` |
| Device ↔ vehículo | Campo explícito `systemId` en el device. Un sysid desconocido se loguea y se ignora, nunca se auto-crea un device |
| Altitud de los wp | `pos[2]` es AMSL → `MAV_FRAME_GLOBAL` con coordenadas int (ver riesgo R1) |
| StopMission | Hold (no RTL). `ReturnHome` sigue siendo su propio comando |
| Versión del protocolo | Se envía MAVLink v2, se aceptan v1 y v2 al parsear |

## Cómo encaja en la arquitectura existente

El patrón es el de MQTT (`MqttServer` + `mqttGateway` + `initMqttServer` + flag opt-in), con una
diferencia de fondo: **MAVLink no transporta mensajes ROS**. MQTT y robofleet reusan
`msgDecode`/`msgEncode` porque llevan la misma forma de mensaje. Acá cada mensaje MAVLink se
declara como un pseudo-msgType `mavlink/<MENSAJE>`, y entra en los mismos registries:

```
devices_msg.yaml (categoría px4_mavlink / ardupilot_mavlink)
  subscribers.<slot>.messageType: mavlink/GLOBAL_POSITION_INT  → decoder en msgDecode (slot, msgType)
  publishers.vehicle_command.messageType: mavlink/COMMAND_INT   → encoders en commandRegistry
  services.configure_mission.messageType: mavlink/MISSION       → encoder de misión + protocolo de upload
```

`commandsModel`, `missionModel`, `positions` y el cliente no se enteran del protocolo, porque
todo pasa por el gateway.

### Firmware: dato del mensaje, no nombre de categoría

PX4 y ArduPilot difieren en los números de modo, en la semántica de algunos MAV_CMD y en la
misión (home en seq 0). Eso **no** se resuelve con un branch por categoría (regla de
`msgDecode`). El decoder de HEARTBEAT guarda `autopilot` (`MAV_AUTOPILOT_PX4` = 12,
`MAV_AUTOPILOT_ARDUPILOTMEGA` = 3) en `position.attributes`, igual que `landedStateOf` usa
`airframe_type`. Los encoders eligen la tabla del dialecto según ese dato. Si el autopilot es
desconocido, el comando se **rechaza** (mismo criterio que con `systemId` desconocido).

Las dos categorías existen igual, para los defaults de rates y de `commands:` (límites), pero
ningún código pregunta por su nombre.

## Fases

### F0 · Spike con SITL (sin código de producción)

Script suelto en el scratchpad, contra PX4 SITL multi-vehículo y ArduPilot `sim_vehicle.py`:

- [x] Parsear por UDP. Resultado: un pipeline por remote, no por socket (ver Resultados F0).
- [x] Escribir. Resultado: `protocol.serialize()` + `socket.send` para UDP; `send(stream)`
      para serie y TCP.
- [ ] `COMMAND_LONG` vs `COMMAND_INT`: confirmar cuál acepta cada firmware para
      `DO_REPOSITION` y `NAV_TAKEOFF` (ArduPilot solo acepta `DO_REPOSITION` por `COMMAND_INT`).
- [x] `SET_MESSAGE_INTERVAL` (MAV_CMD 511): verificado en PX4, falta ArduPilot. `REQUEST_DATA_STREAM` está
      deprecado: no se usa.
- [ ] Medir la diferencia entre la altitud AMSL de `GLOBAL_POSITION_INT.alt` y lo que guarda
      hoy la GCS como `pos[2]` (riesgo R1).

#### Resultados F0 (2026-10-21, PX4 1.18.0 SITL, `node-mavlink` 2.3.0, Node 20)

Spike: `scratchpad/f0/spike.mjs`. Solo comandos inofensivos; no se armó ni se voló.

Verificado:

- **Import ESM**: `import { ... } from 'node-mavlink'` funciona (named exports desde CJS).
- **Header**: los campos son `header.sysid` / `header.compid` / `header.msgid`. **No existen
  `srcSystem`/`srcComponent`**: con esos nombres el registro queda indexado por `undefined` y
  todos los drones colapsan en una sola entrada.
- **`CommandLong` / `CommandInt` usan `_param1…_param7`.** Asignar `m.param1` no da error: crea
  una propiedad suelta y **por el cable sale 0**. Es la trampa más peligrosa: un
  `DO_REPOSITION` con param5/6/7 en 0 va a lat 0, lon 0. Son las únicas dos clases con `_`
  (`MissionItemInt` usa `param1..4` normales). Mitigación obligatoria:
  - Un único builder de comandos que escriba `_paramN`.
  - Un test que serialice → parsee y compare cada param.
- **Dialectos**: `AutopilotVersion` está en `standard`, no en `common`. El REGISTRY tiene que
  incluir `minimal + standard + common + ardupilotmega`.
- **Enviar**: `protocol.serialize(msg, seq)` devuelve el `Buffer`. Para UDP alcanza con
  `socket.send(buf, port, addr)` y un `seq` por enlace (`& 0xff`); no hace falta un `Writable`.
  `send(stream, …)` queda para serie y TCP.
- **Recibir por UDP**: el pipeline `Splitter → Parser` procesa **sincrónicamente** dentro de
  `write()`, así que un `rinfo` guardado en una variable compartida llega tarde. Diseño
  correcto: **un pipeline por remote** (`address:port`), con el remote en el closure. Ni uno
  por datagrama ni uno por socket.
- **`MavEsp8266`** (helper UDP de la librería) guarda un solo remote: no sirve para multi-drone.
- **PX4 SITL** sale del puerto local 18570 (instancia 0) hacia 14550, a ~44 Hz en
  ATTITUDE / GLOBAL_POSITION_INT / LOCAL_POSITION_NED. Solo compid 1. Hay mensajes fuera de
  los dialectos cargados (#441), que se cuentan pero no se decodifican.
- **HEARTBEAT** en Hold: `autopilot=PX4`, `baseMode=29`, `customMode` main 4 / sub 3
  (AUTO.LOITER). Coincide con la tabla de F2.
- **`SET_MESSAGE_INTERVAL`**: ACCEPTED, el rate pasa de 48 → 1.7 Hz, y con param2 = 0 vuelve
  al default.
- **`REQUEST_MESSAGE`**: ACCEPTED por `COMMAND_LONG` y por `COMMAND_INT`, y el mensaje llega.
  Un MAV_CMD inexistente responde `UNSUPPORTED`.
- **AUTOPILOT_VERSION.capabilities** = `0x4e8ff`: MISSION_INT, COMMAND_INT, MAVLINK2 y
  MISSION_FENCE presentes. Sirve para detectar el soporte al descubrir el vehículo.
- **R1 no se puede medir en SITL**: `GPS_RAW_INT.altEllipsoid == alt` (el simulador no modela
  el geoide). El home del SITL está en 43.47, -8.18 (Galicia), donde la ondulación real ronda
  los +55 m. Hay que medirlo con hardware.

#### Resultados F0 de vuelo PX4 (`scratchpad/f0/flight_px4.mjs`, 12/12 OK)

- **`CommandInt`: `x`/`y`/`z` son `_param5`/`_param6`/`_param7`.** Es la misma trampa que
  `_paramN`, y se manifestó en vuelo: PX4 recibió lat 0, lon 0, alt 0. Para Takeoff, navigator
  trata (0,0) como "posición actual", y el ACK fue ACCEPTED aunque no despegó ("Already higher
  than takeoff altitude"). **El ACK no detecta este bug**: solo lo detecta el test de ida y
  vuelta, que pasa a ser **bloqueante** para F3.
- **Armar sin heartbeat previo de la GCS** → `TEMPORARILY_REJECTED`. Tras ~1 s de heartbeats,
  PX4 emite "GCS connection regained" y arma. El server no manda comandos a un vehículo
  descubierto hasta haberle enviado heartbeats durante ≥ 1 s.
- **Posicionales: siempre `COMMAND_INT`.**

  | | Error final | Tiempo hasta llegar |
  |---|---|---|
  | `DO_REPOSITION` por `COMMAND_LONG` (float32, 0.18 m de cuantización en este lat/lon) | 0.94 m | 36 s |
  | `DO_REPOSITION` por `COMMAND_INT` | 0.05–0.10 m | 9–14 s |

  `NAV_TAKEOFF` funciona por los dos.
- **`NAV_TAKEOFF` en PX4: param7 / `z` es altitud AMSL** (frame `MAV_FRAME_GLOBAL`). El modo
  pasa por AUTO.TAKEOFF (sub 2) y termina en AUTO.LOITER (sub 3).
- **`DO_REPOSITION`** con param1 = -1 (velocidad por defecto) y param2 = 1 (cambiar de modo)
  deja el vehículo en AUTO.LOITER.
- **Hold**: `DO_SET_MODE [1, 4, 3]` → ACCEPTED y el HEARTBEAT confirma main 4 / sub 3.
- **Land**: `NAV_LAND` con lat/lon NaN aterriza en el sitio. "Landing detected" y auto-disarm
  a los ~2 s.
- **RTL**: sube a 30 m sobre el destino (`RTL_RETURN_ALT`), vuelve y aterriza. Aterrizó a
  28 m del home *original* porque **PX4 redefine el home en cada armado**. El home de la GCS
  tiene que salir de `HOME_POSITION` (se re-pide tras cada armado), nunca de la primera
  posición vista. Ojo con `SaveHome` (comando local) para devices MAVLink.
- **ACK**: los 10 comandos respondieron `ACCEPTED` en < 1 s y ninguno usó `IN_PROGRESS`. El
  ACK significa "aceptado", no "completado". Que el comando se cumplió (altura alcanzada,
  aterrizado, posición) lo observa la telemetría, como ya hace `missionSignals`. El gateway
  devuelve success con el ACK y no espera a la maniobra.

Pendiente de F0 (requiere armar y volar el SITL, o hardware):

- [x] `NAV_TAKEOFF` y `DO_REPOSITION` por `COMMAND_LONG` vs `COMMAND_INT` en PX4.
- [x] ArduPilot (ver Resultados F0 ArduCopter).
- [x] Multi-vehículo PX4 + ArduPilot en el mismo puerto (`multi.mjs`). PX4 sysid 1 desde
      :18570 y ArduCopter sysid 2 desde :53383 llegan al mismo socket 14550. El pipeline por
      remote los separa, y `REQUEST_MESSAGE` a cada sysid devuelve la versión de cada uno
      (1.18.0 / 4.7.2).

**PX4 ignora el frame de `COMMAND_INT` en Takeoff: verificado en vuelo, no solo en el código**
(`takeoff_rel_px4.mjs`). `NAV_TAKEOFF` por `COMMAND_INT`, frame `GLOBAL_RELATIVE_ALT`, `z = 10`:

- ACK `ACCEPTED`, y a continuación "Already higher than takeoff altitude".
- Pico relAlt 0.0 m; auto-disarm a los 11 s.

O sea, PX4 leyó 10 m **AMSL**. `VehicleCommand.msg` no tiene campo `frame`: se pierde en
`mavlink_receiver.cpp:701`. **No hay regla única de frame para el Takeoff guiado**: con frame
relativo en los dos firmwares, PX4 se queda en tierra con un ACK de éxito. Distinto es el
**item de misión** (`MISSION_ITEM_INT`), que conserva su frame en los dos firmwares (a
verificar en F4).

**F0 cerrado** (salvo serie/SiK, que se posterga hasta tener hardware).

#### Resultados F0 ArduCopter (4.7.2 SITL en Docker, `scratchpad/ardupilot/Dockerfile`)

`flight_ardupilot.mjs`: todos los pasos OK. `takeoff_amsl_ardupilot.mjs`: confirma el DENIED
con frame GLOBAL.

- **Imagen**: no hay imagen oficial de ArduPilot SITL ya compilada. Se compila `Copter-4.7.2`
  desde el repo oficial sobre `ardupilot/ardupilot-dev-base`.
- **`SYSID_THISMAV` se renombró a `MAV_SYSID` en 4.7.** La conversión de legado solo migra el
  valor guardado; un archivo de defaults con el nombre viejo **se ignora sin aviso** y el
  vehículo queda en sysid 1 (choca con el PX4). Documentarlo para los operadores.
- **De fábrica no manda telemetría**: solo HEARTBEAT, STATUSTEXT y PARAM_VALUE (en PX4: ~25
  mensajes, varios a 44 Hz). MAVProxy y QGC piden los streams al conectar. Sin
  `SET_MESSAGE_INTERVAL`, un device ArduPilot aparece online y nunca se mueve en el mapa.
  - Los 6 intervalos pedidos (GLOBAL_POSITION_INT, EXTENDED_SYS_STATE, HOME_POSITION,
    SYS_STATUS, VFR_HUD, BATTERY_STATUS) dieron ACCEPTED.
  - **Los intervalos viven mientras corre el proceso**: no sobreviven a un reboot. El server los
    re-pide en cada (re)descubrimiento, después de un timeout de heartbeat.
- **Armado**: los primeros ~30 s responde `FAILED` (EKF/GPS sin converger; el motivo va por
  STATUSTEXT "PreArm: …"). PX4 usa `TEMPORARILY_REJECTED` para "falta GCS". Ninguno de los dos
  pone el motivo en el ACK, así que **el gateway adjunta al error los STATUSTEXT que llegaron
  en la ventana del comando**. Si no, el operador ve un "FAILED" sin explicación.
- **HEARTBEAT**: `autopilot=ARDUPILOTMEGA`, `baseMode` = 81 desarmado. El bit SAFETY_ARMED
  funciona igual que en PX4. `customMode` es el número de modo de Copter directo (GUIDED 4,
  LOITER 5, RTL 6, LAND 9).
- **Home**: igual que PX4, se redefine en cada armado (el RTL volvió al punto del 2º armado,
  a 28 m del original).
- **AUTOPILOT_VERSION**: `fw 4.7.2`, capability COMMAND_INT presente.
- [ ] Serie / SiK (sin hardware conectado).
- [ ] Multi-vehículo: hay una sola instancia de PX4 corriendo.

### F1 · Transporte, presencia, heartbeat

- `models/mavlink/MavlinkServer.js`: el server de MAVLink, espejo de `MqttServer`.
- `models/mavlink/links.js`: `MavlinkLink` con tres variantes:
  - UDP: socket + remote aprendido por sysid.
  - TCP: un cliente.
  - Serie: `serialport`.
- `models/mavlink/index.js`: `initMavlinkServer` / `getMavlinkServer`.
- `models/gateways/mavlinkGateway.js` + registro en `gateways/index.js`.
- `'mavlink'` en `DEVICE_PROTOCOL` (`schemas/zod/devices.js`). `test-gateways.js` ya falla si se
  desincronizan.
- `systemId` en `DeviceSchema`, más la columna Sequelize con su migración, y DevicePublic/Private
  según corresponda.
- Config (`config/config.js`):
  - `MAVLINK_CONNECTION` (default `false`, opt-in como MQTT).
  - `MAVLINK_LINKS`, por ejemplo `udp:0.0.0.0:14550,serial:/dev/ttyUSB0:57600,tcp:127.0.0.1:5760`.
- Recepción UDP: un pipeline `Splitter → Parser` por remote `address:port` (ver Resultados F0).
- Registro `sysid → { link, remote, lastSeen }`. Un sysid visto en dos enlaces a la vez se
  loguea como conflicto: puede ser un sysid duplicado, o la misma radio por dos caminos.
- HEARTBEAT del GCS a 1 Hz por cada enlace. PX4 (`NAV_DLL_ACT`) y ArduPilot (`FS_GCS_ENABLE`)
  lo exigen para armar y para no disparar el failsafe de GCS.
- Presencia: online con el primer HEARTBEAT del drone, offline tras N segundos sin heartbeat.
  Se refleja en `device.status`.
- Solo se aceptan paquetes del autopilot (`compid` 1) para telemetría de vuelo. Los demás
  componentes (cámara, gimbal) se aceptan para sus slots.
- `rawTap.publish({ transport: 'mavlink', raw: () => … })` con thunk lazy: el payload a JSON
  solo se arma si alguien inspecciona.

#### Estado F1 (2026-10-21): hecho, salvo serie (con SiK)

- `models/mavlink/`: `mavlinkMessages.js` (registry, identidad GCS, `toPlain`), `links.js`
  (`UdpLink`, `TcpLink`, `parseLinkSpec`), `MavlinkServer.js`, `index.js`. Más
  `gateways/mavlinkGateway.js`, `'mavlink'` en `DEVICE_PROTOCOL`, `MAVLINK_CONNECTION` /
  `MAVLINK_LINKS` en `config.js` y `mavlink` en `serverModel.Protocol()`.
- **Device**: columna `systemId` (ALTER aditivo en `common/sequelize.js`). zod: 1..254, y
  obligatorio si `protocol: mavlink`. `DevicesModel.getBySystemId`. `create()` ya no intenta
  suscribir en ROS un device MAVLink (antes respondía "Ros no está conectado" aunque el device
  quedara creado).
- **Presencia**: solo cuenta el HEARTBEAT del autopiloto (`autopilot != INVALID`,
  `type != GCS`); se ignoran componentes, otra GCS y sysid 0/255. Offline tras 5 s.
  - Un sysid vivo que aparece por otra ruta conserva la primera y se loguea como posible
    `MAV_SYS_ID` duplicado. Si la ruta anterior ya está offline, se adopta la nueva
    (ArduPilot reinicia con otro puerto efímero).
  - Eventos `online` / `offline` en el server: F2 los usa para pedir los intervalos.
  - El `status` del device sigue saliendo de `DeviceHealthMonitor` (`deviceTime` de la
    posición), así que se refleja recién cuando F2 decodifique telemetría.
- **Heartbeat de GCS**: 1 Hz, uno por ruta. `gcsHeartbeatSince` marca desde cuándo lo recibe
  cada vehículo; F3 no manda comandos hasta ≥ 1 s después.
- **Inspector raw**: el slot es el subscriber de la categoría cuyo `messageType` es
  `mavlink/<MSG>`. El decode es lazy, en el thunk. `toPlain` usa los nombres del spec y
  convierte los BigInt (el `JSON.stringify` del WS tira con BigInt).
- **Bug evitado**: el cache `sysid → device` guarda la promesa. Si guarda el resultado, cada
  paquete que llega mientras la primera consulta está en vuelo lanza otra. `MqttServer._getDevice`
  tiene ese defecto (fuera del alcance de F1).
- Tests: `test/test-mavlink.js`, 23 tests, con paquetes reales serializados y parseados por
  node-mavlink y UDP real en loopback.
- **Verificado contra los dos SITL a la vez** (`scratchpad/f1/live.mjs`, `MavlinkServer` de
  producción): los dos online por 14550, PX4 respondió "GCS connection regained" y el inspector
  entrega JSON válido.

### F2 · Telemetría

> Nota de la sesión que hizo `telemetryWatchdog`: el watchdog espera **todos** los subscribers
> de la categoría. Los slots por evento (`vehicle_command_ack` ← COMMAND_ACK, STATUSTEXT,
> MISSION_ITEM_REACHED) llevan `watchdog: false`, o se reportan como `missing`.
> `DevicesModel.getAll` agrega `telemetry` a cada device: conservarlo si se toca ese map.

Decoders nuevos en `msgDecode.js` y slots en las categorías:

| Slot | Mensaje | Notas |
|---|---|---|
| `vehicle_status` | HEARTBEAT | Armado: bit `MAV_MODE_FLAG_SAFETY_ARMED` de `base_mode`. Modo: `custom_mode` según `autopilot`. `systemId` = `header.srcSystem`; `autopilot` → attributes |
| `position_global` | GLOBAL_POSITION_INT | lat/lon ×1e-7, `alt` en mm (AMSL), `hdg` en cdeg (65535 = desconocido) |
| `speed` | VFR_HUD | `groundspeed` |
| `attitude` | ATTITUDE | rad → deg |
| `battery` | BATTERY_STATUS (fallback SYS_STATUS) | `battery_remaining` en %, -1 = desconocido |
| `flight_status` | EXTENDED_SYS_STATE | `landed_state` es MAV_LANDED_STATE → `px4FlightPhase` sirve tal cual, también para ArduPilot |
| `vehicle_command_ack` | COMMAND_ACK | `result` es MAV_RESULT → `px4CommandResult` sirve tal cual |
| `gimbal` | GIMBAL_DEVICE_ATTITUDE_STATUS | cuaternión → pitch/roll/yaw en grados |
| `mission_state` | MISSION_ITEM_REACHED / MISSION_CURRENT | `seq` → `wp_reached` mediante el mapa de F4 |

Adapters nuevos en `vehicleStates.js`:

- `px4CustomMode(custom_mode)`: `main = (m >> 16) & 0xff`, `sub = (m >> 24) & 0xff`.
  - main: MANUAL 1, ALTCTL 2, POSCTL 3, AUTO 4, ACRO 5, OFFBOARD 6, STABILIZED 7.
  - sub de AUTO: TAKEOFF 2, LOITER 3, MISSION 4, RTL 5, LAND 6, RTGS 8, FOLLOW 9, PRECLAND 10,
    VTOL_TAKEOFF 11, DESCEND 12 (verificado en `px4_custom_mode.h`: el enum cuenta
    implícitamente desde 1; la versión anterior de esta tabla, escrita de memoria, estaba
    corrida).
- `ardupilotCopterMode(custom_mode)`: STABILIZE 0, ACRO 1, ALT_HOLD 2, AUTO 3, GUIDED 4,
  LOITER 5, RTL 6, CIRCLE 7, LAND 9, POSHOLD 16, BRAKE 17, SMART_RTL 21, AUTO_RTL 27.
- `mavlinkFlightMode(autopilot, custom_mode)` elige entre los dos. Un autopilot desconocido da
  `FLIGHT_MODE.OTHER`, con el código raw al lado.

Rates: cada slot puede declarar `rate_hz`. Al descubrir un drone, el server manda
`SET_MESSAGE_INTERVAL` por cada mensaje de la categoría. **SiK**: el ancho de banda real ronda
los ~1–2 KB/s útiles a 57600, así que los rates por defecto de la categoría tienen que ser bajos:
posición 2–4 Hz, el resto 1 Hz, y gimbal/attitude solo si hace falta.

#### Estado F2 (2026-10-21): hecho, salvo `mission_state` (va con F4)

- **Categorías** `px4_mavlink` y `ardupilot_mavlink` al final de `devices_msg.yaml`. Slots:
  - `vehicle_status` ← HEARTBEAT (sin rate: siempre 1 Hz)
  - `position_global` ← GLOBAL_POSITION_INT, 4 Hz
  - `flight_status` ← EXTENDED_SYS_STATE, 1 Hz
  - `battery` ← BATTERY_STATUS, 1 Hz
  - `home_position` ← HOME_POSITION, 0.5 Hz, `timeout_s: 10`: con el timeout por defecto de
    5 s, dos mensajes perdidos en SiK harían oscilar al watchdog
  - `vehicle_command_ack` ← COMMAND_ACK, `watchdog: false`
  - `gimbal` ← GIMBAL_DEVICE_ATTITUDE_STATUS, 2 Hz (solo PX4)

  Sin `name:`, porque no hay topic. Ninguna declara endpoints todavía (F3): solo exponen SaveHome.
- **Decoders** en `msgDecode.js`. Reciben la vista `toPlain` (nombres del spec, enteros del
  cable).
  - GLOBAL_POSITION_INT: con 0/0 (sin fix) devuelve `null`, para no dibujar el vehículo en el
    golfo de Guinea.
  - `hdg` = 65535 no pisa el `course`.
  - BATTERY_STATUS: solo la batería con id 0, y −1 (desconocido) se descarta.
  - El gimbal comparte la matemática con el decoder de `px4_msgs`
    (`gimbalFromAttitudeStatus`).
- **Estados** (`vehicleStates.js`): `mavlinkArmState` y `mavlinkFlightMode({autopilot, type,
  customMode})`. Las tablas de PX4 y ArduCopter están verificadas contra `px4_custom_mode.h` y
  `ArduCopter/mode.h` (la tabla de PX4 escrita de memoria estaba corrida). Un ArduPilot que no
  es multirotor da OTHER: Plane numera distinto. EXTENDED_SYS_STATE y COMMAND_ACK reusan
  `px4FlightPhase` / `px4CommandResult`, porque son los enums MAVLink.
- **Home**: SubscriberKey `home_position` y reducer `home`. Así el home de la GCS sigue al
  del autopiloto, que cambia en cada armado.
- **Streams**: al pasar a online se pide `SET_MESSAGE_INTERVAL` para cada slot con `rate_hz`.
  Cada 2 s se vuelve a pedir lo que no llega (nunca visto, o en silencio durante 3 periodos),
  con 5 s mínimo entre reintentos. Un ACK que no es ACCEPTED se loguea (rate-limited).
- **Ingesta**: la vista plana se construye una sola vez por mensaje declarado, y esa misma va
  al inspector, al watchdog y al decoder. El HEARTBEAT de un componente (cámara) sobre el sysid
  del vehículo se descarta: no es el estado del vehículo.
- **`UdpLink` sin `reuseAddr`**: con él, una segunda GCS hace bind al mismo puerto sin error y
  el kernel reparte los datagramas entre las dos. Ahora falla con `EADDRINUSE`.
- `test-mavlink.js`: 44 tests. Fixtures en `test/helpers/mavlinkFixtures.js`.
- **Verificado contra los dos SITL** (`scratchpad/f2/live.mjs`, categorías y decoders reales):
  - ArduCopter pasó de no mandar telemetría a mandar exactamente los rates pedidos.
  - PX4 bajó de ~50 Hz a 3.8–3.9 Hz en posición.
  - Estado, home y gimbal decodificados en los dos.
- **Observación abierta**: en la primera corrida, PX4 mandó **solo HEARTBEAT** durante 12 s
  (el resto de sus slots en 0). No se reprodujo en 4 corridas más, 3 de ellas desde los rates
  de fábrica. Hipótesis no verificada: un trabón del simulador. Los streams de PX4 se mandan
  cuando se actualiza el topic uORB, y el heartbeat sale igual. En la corrida 2 todos los slots
  de PX4 bajaron juntos a 0.92 Hz. Con un vehículo real, el watchdog lo marca como `stale`.

### F3 · Comandos

- Endpoint `publishers.vehicle_command` con `messageType: mavlink/COMMAND_INT`, o `COMMAND_LONG`
  según lo que salga de F0.
- Se registran sobre ese msgType: Takeoff, Land, ReturnHome, Hold, Arm, Disarm, KillMotors,
  MoveRelative y Gimbal (con `sequence([CONFIGURE, PITCHYAW])`, igual que PX4 ROS2).
- **Refactor de `vehicleCommand.js`**: hoy emite semántica uORB (yaw en **radianes**, ver la
  nota de la cabecera). Pasa a emitir contenido de spec MAV_CMD (grados, frame explícito), y el
  encoder de `px4_msgs/msg/VehicleCommand` convierte a radianes. Así el dominio es el estándar y
  uORB es la excepción, no al revés. Los tests actuales de PX4 ROS2 son la red de seguridad del
  refactor.
  - Alternativa: un flag de unidades en el builder. Es menos cambio, pero deja uORB como la
    "verdad" del dominio.
- Diferencias de dialecto (tabla por `autopilot`):

  Verificado en SITL (PX4 1.18.0, ArduCopter 4.7.2), salvo lo marcado como *sin verificar*:

  | Comando | PX4 | ArduPilot Copter |
  |---|---|---|
  | Hold | `DO_SET_MODE [1, 4, 3]` (AUTO.LOITER) | `DO_SET_MODE [1, 17]` (**BRAKE**). LOITER sigue el stick de throttle y sin RC **baja hasta el piso** (visto en F4) |
  | Takeoff | `COMMAND_INT`, `z` = AMSL actual + h. **El frame se ignora** (`mavlink_receiver.cpp:701` copia `z` → param7) | Primero `DO_SET_MODE [1, 4]` (GUIDED), armar, y `COMMAND_INT` **frame `GLOBAL_RELATIVE_ALT`**, `z` = h. Con cualquier otro frame responde `DENIED` (`GCS_MAVLink_Copter.cpp`, `handle_MAV_CMD_NAV_TAKEOFF`). lat/lon se ignoran |
  | MoveRelative | `DO_REPOSITION` por `COMMAND_INT`, `z` AMSL | `DO_REPOSITION` por `COMMAND_INT`, frame `GLOBAL` (AMSL) o `GLOBAL_RELATIVE_ALT`: los dos funcionan. Por `COMMAND_LONG` → `UNSUPPORTED`. Requiere GUIDED |
  | Land | `NAV_LAND` (lat/lon NaN = en el sitio) | `NAV_LAND` → pasa a modo LAND (9) |
  | ReturnHome | `NAV_RETURN_TO_LAUNCH` | `NAV_RETURN_TO_LAUNCH` → modo RTL (6). Al desarmar sigue en RTL |
  | KillMotors | `COMPONENT_ARM_DISARM` param2 = 21196 (*sin verificar en vuelo*) | igual (*sin verificar en vuelo*) |
  | Gimbal | gimbal manager v2 (*sin verificar*) | gimbal manager (4.4+), fallback `DO_MOUNT_CONTROL` (*sin verificar*) |

  **El mismo número significa cosas distintas**: `NAV_TAKEOFF` param7 = 110 es "110 m AMSL" en
  PX4 y "110 m sobre el home" en ArduPilot. El dominio ya está bien definido
  (`Takeoff.altitude` = altura sobre la posición actual, `command_schema.yaml`). Cada dialecto
  convierte desde ahí: PX4 suma la AMSL actual, ArduPilot la usa tal cual. Un test por dialecto
  verifica la conversión.

  **Regla para Reposition** (la misma en los dos): `COMMAND_INT`, frame `GLOBAL`, `z` AMSL.

- **El gateway espera el `COMMAND_ACK`**, con correlación `(sysid, command)` como el `pending`
  de MQTT:
  - `ACCEPTED` → success.
  - `IN_PROGRESS` → sigue esperando y extiende el timeout.
  - Cualquier otro resultado → error con el MAV_RESULT canónico.
  - Sin ACK: reintenta con `confirmation++`, y al agotar los reintentos devuelve timeout.

  Es mejor semántica que el publisher ROS, que dispara y se olvida.
- `target_system` = `systemId` del device. No se manda nunca un 0 (broadcast): un KillMotors
  llegaría a todos los vehículos del enlace.
- Un solo builder `toCommandLong` / `toCommandInt` escribe los campos `_param1…_param7` de
  node-mavlink (en `CommandInt`, `x`/`y`/`z` también son `_param5..7`). Ningún otro módulo
  instancia `CommandLong` ni `CommandInt` a mano. Hay un test de ida y vuelta
  (serialize → parse) por cada comando, **bloqueante**: el ACK no detecta un param en 0.
- Los comandos con lat/lon (`DO_REPOSITION`, y `NAV_TAKEOFF` cuando lleva posición) van por
  `COMMAND_INT`. Los demás van por `COMMAND_LONG`.

#### Estado F3 (2026-10-21): hecho

- **Dialecto = msgType del endpoint** (`publishers.vehicle_command`: `mavlink/PX4_COMMAND` o
  `mavlink/ARDUPILOT_COMMAND`), no el heartbeat. Esto cambia lo que decía el plan: el encoder se
  elige por msgType, como en el resto del registry, y el set de comandos no depende de que haya
  telemetría. `MavlinkServer.sendCommand` **rechaza sin enviar** si el `autopilot` del heartbeat
  no coincide con el dialecto: un device mal categorizado da un error claro en vez de un Takeoff
  con la semántica de altura equivocada. Para los decoders sigue valiendo el heartbeat.
- `models/commands/mavlinkCommand.js` arma descriptores puros (`kind: long|int`), y
  `mavlinkMessages.toCommandMessage` los convierte. `commandLong` / `commandInt` son el único
  lugar que escribe `_paramN`.
- **Verificado en el código, además de en SITL**: PX4 toma el yaw de `DO_REPOSITION` en
  **radianes** también por MAVLink (`navigator_main.cpp` copia param4; el spec dice grados).
  ArduCopter **ignora** ese yaw (`set_destination(..., use_yaw=false)`), así que `MoveRelative`
  con `dyaw` agrega un `CONDITION_YAW` en grados.
- Gimbal solo en PX4 (CONFIGURE + PITCHYAW, componente 0). En ArduPilot no se ofrece: el SITL no
  tiene montura y no está verificado.
- **Envío**:
  - espera ≥ 1 s de heartbeat de GCS (hasta 3 s);
  - ACK correlacionado por comando, ignorando los ACK dirigidos a otra GCS;
  - IN_PROGRESS mantiene la espera (máx. 15 s);
  - sin ACK, 3 intentos de 1.5 s con `confirmation++`;
  - en un rechazo, se adjuntan los STATUSTEXT de severidad ≤ WARNING desde el envío, con 300 ms
    de gracia;
  - los STATUSTEXT en chunks (> 50 caracteres) se re-ensamblan por `id` sin recortar en los
    cortes.
- `test-mavlink.js`: 65 tests.
- **E2E en SITL** por `commandsModel.sendCommand` (schema → registry → gateway → server), con
  los dos drones en paralelo y la DB en memoria (`scratchpad/f3/e2e.mjs`):
  - PX4: 12/12.
  - ArduCopter: todos los comandos OK, incluido `CONDITION_YAW` (giro exacto de 90°). El motivo
    de un rechazo llega al operador (`FAILED — Arm: Land mode not armable`).
- **Abierto: ArduCopter no arma después de Land/RTL.** Se queda en LAND/RTL, que no son modos
  armables. PX4 vuelve solo a Hold. Opciones:
  - (a) `Arm` en ArduPilot = `[GUIDED, ARM]`: paridad con PX4, pero cambia el modo al armar;
  - (b) dejarlo así: el operador manda Hold antes de Arm, y el motivo ya le llega.

  **Decisión del usuario (2026-10-21): (b), se deja así.**
- **Bug previo encontrado (no es de MAVLink, sin arreglar)**: `positionsModel.getByDeviceId`
  es `async`, y tres lugares lo llaman sin `await` y leen campos de la `Promise`:
  - `missionSignals.signalFlightState`: las anomalías DISARMED / RTH / ON_GROUND nunca se
    detectan;
  - `signalAutopilotFeedback`: `wp_reached` nunca se lee, y **F4 lo necesita**;
  - `events.js:50`: los eventos se guardan sin posición.

  **Arreglado (2026-10-21, a pedido del usuario)**: `positionsModel.getByDeviceId` es
  sincrónico. `test/test-mission-signals.js` falló antes del fix (3/4) y pasa después.
  Consecuencias, en todos los protocolos:
  - las anomalías ahora se emiten a la UI (son informativas, no disparan acciones);
  - `wp_reached` con confianza alta **avanza la task** y, en el último waypoint, la completa
    (`missionWpTracking`). Era el diseño previsto, pero nunca había corrido: vigilar a los
    devices con `muav_state_machine` (CATEC/DJI) en la próxima misión real;
  - los eventos se guardan con la posición del device.

### F4 · Misiones

- Protocolo de upload, como máquina de estados por device:
  `MISSION_COUNT → MISSION_REQUEST_INT(seq) → MISSION_ITEM_INT(seq) … → MISSION_ACK`.
  - Timeout y reintento por item.
  - Un `MISSION_REQUEST` (no `_INT`) se contesta igual con `MISSION_ITEM_INT`.
  - Se cancela si llega otro upload al mismo device.
  - Endpoint `services.configure_mission` con `messageType: mavlink/MISSION`, agregado a
    `MISSION_SERVICE_TYPES` para que el encoder de `LOAD_MISSION` matchee.
- Encoder task → items (`missionEncodeMavlink.js`), con `MAVLINK_SYMBOLS` en `missionSymbols.js`
  para que la UI filtre las opciones con el mecanismo que ya existe:
  - `NAV_TAKEOFF`, `NAV_WAYPOINT` (param1 hold), `DO_CHANGE_SPEED`, `CONDITION_YAW`,
    `DO_GIMBAL_MANAGER_PITCHYAW`, `IMAGE_START_CAPTURE` / `VIDEO_START_CAPTURE`.
  - Fin según `mode_landing`: `NAV_RETURN_TO_LAUNCH`, `NAV_LAND` o nada (hover).
  - Frame `MAV_FRAME_GLOBAL` con `pos[2]` AMSL.
- **ArduPilot: el seq 0 es el home**. Lo que se sube en seq 0 lo pisa el firmware, así que el
  encoder mete un item de relleno en seq 0 para ArduPilot y no lo mete para PX4.
- **Invariante: mapa `seq MAVLink → índice de wp de la task`**, guardado por device al subir.
  Los items extra (speed, yaw, gimbal, foto, home) corren la numeración. Sin el mapa,
  `wp_reached` miente y el DAG de tasks se completa antes de tiempo o nunca. El decoder de
  `MISSION_ITEM_REACHED` traduce con ese mapa y deja pasar solo los seq que son waypoints.
- `command_mission`:
  - PX4: `sequence([ARM, MISSION_START])`.
  - ArduPilot: `sequence([DO_SET_MODE GUIDED, ARM, DO_SET_MODE AUTO, MISSION_START])`.
  - Verificar el orden exacto en SITL (F0/F4).
- `pause_mission` / `resume_mission`: `DO_PAUSE_CONTINUE` con param1 = 0 / 1.
- `stop_mission`: **Hold**, el mismo comando de F3.
- Las señales del `missionExecutionSM` no cambian: leen `armState` / `flightMode` /
  `flightPhase` canónicos y `wp_reached`.

#### Estado F4 (2026-10-21): hecho en ArduCopter, falta validar PX4 en vuelo

- **Endpoints** de las categorías mavlink:
  - `services.configure_mission`: `mavlink/PX4_MISSION` / `mavlink/ARDUPILOT_MISSION`, que es
    la familia de misión. Está en `MISSION_SERVICE_TYPES`, así que LoadMission matchea por
    msgType.
  - `command_mission` / `pause_mission` / `resume_mission` / `stop_mission` con el msgType de
    comandos. Todos los comandos MAVLink se registran por **(key, msgType)**: con varios
    endpoints que comparten msgType, Takeoff solo por msgType también matchearía
    `command_mission`.
- **Familia `MAVLINK_SYMBOLS` y perfil `mavlink`** (`mission_schema.yaml`): solo lo volado en
  SITL.
  - `mode_yaw`: AUTO y WAYPOINT_CUSTOM.
  - `mode_trace`: STRAIGHT_AND_STOP.
  - `mode_landing`: hover, home y land.
  - Acciones: stay, photo, video start/stop, yaw y gimbal.
  - Sin `max_vel`. El resto se filtra de la UI, y el encoder lo **rechaza** si igual llega.
- **`missionEncodeMavlink.js`**: task → items en `MAV_FRAME_GLOBAL` (AMSL), más `wpSeqs` (seq
  de cada waypoint) y `finish`.
  - ArduPilot: placeholder en seq 0 (home).
  - Yaw: PX4 en param4 de NAV_WAYPOINT; ArduPilot con `CONDITION_YAW` después.
  - **ArduPilot rechaza la subida entera si un param es NaN** donde
    `AP_Mission::sanity_check_params` no lo permite (solo param4 de NAV_WAYPOINT/TAKEOFF/LAND,
    entre otros). El item de gimbal con yaw/rates NaN dio `INVALID_PARAM2`; en ArduPilot va
    `[pitch, 0, 0, 0]`.
- **Subida** (`MavlinkServer._uploadMission`):
  - COUNT → items a pedido → ACK, con reintentos de 1.5 s ×5 y reenvío de COUNT o del último
    item;
  - una subida nueva cancela la anterior;
  - un rechazo trae `MAV_MISSION_RESULT` + STATUSTEXT;
  - al aceptarse: guarda `wpSeqs` y **resetea `wp_reached` a 0**. En SITL, T2 arrancó con el
    `wp_reached=3` de T1 en la caché; con una ruta más larga, el tracking lo habría saltado sin
    volar.
- **Progreso y fin**:
  - `MISSION_ITEM_REACHED` → `wp_reached` = índice + 1; los items extra no cuentan.
  - Fin = último waypoint (hover), o último waypoint + desarmado (home/land).
  - Se avisa a `missionController.deviceFinishMission`.
  - ArduPilot no reporta el NAV_TAKEOFF como alcanzado: `wp_reached` salta de 0 a 2, y el salto
    de alta confianza lo absorbe.
- **Start**: PX4 `[ARM, AUTO.MISSION]` (falta verificar en vuelo); ArduPilot
  `[GUIDED, ARM, MISSION_START]` (verificado).
- **Pause/Stop** = Hold (en ArduPilot, **BRAKE**). **Resume** = PX4 AUTO.MISSION, ArduPilot
  AUTO.
- **Corrección de F3 — Hold en ArduPilot era inseguro.** LOITER usa el stick de throttle
  (`mode_loiter.cpp`: `get_pilot_desired_climb_rate_ms`). Sin RC, o con el throttle abajo,
  **desciende**: en SITL una misión pausada bajó de 12 m al piso y después no pudo reanudar
  ("Auto: Missing Takeoff Cmd"). El "Hold OK" de F3 era un falso positivo: verificaba el modo,
  no la altura. Ahora es BRAKE (autopilot, ignora al piloto; su timeout a LOITER solo lo activa
  el botón de los Solo).
- **Bug previo arreglado: máquina de estados trabada sin descarga de archivos.** Un device sin
  `CameraFileDownload` se quedaba para siempre en `UAVDownloadFiles` (no hay `onDone`) y el
  scheduler lo mantenía ocupado. Ahora `shouldDownloadFiles` devuelve `false` y la máquina va
  por `FinishWithoutDownload` → END. Afecta a todo device sin descarga, no solo MAVLink.
- **Bug arreglado: cache de "no encontrado" de 30 s.** Un device creado con el vehículo ya
  conectado tardaba hasta 30 s en aparecer. Ahora un miss se cachea 3 s.
- Tests: `test-mavlink.js` (83), `test-task-scheduler.js` (+1), `test-mission-signals.js`.
- **E2E ArduCopter por el flujo manual** (`scratchpad/f4/e2e_mission.mjs ardu`): T1 (hover) →
  T2 (land, mismo device, `depends_on`). T1 voló 3 waypoints y llegó a END. El device se liberó
  y T2 se subió y arrancó **en el aire**, aterrizó y llegó a END. Misión `finish`.
- **E2E pausa** (`scratchpad/f4/pause_resume.mjs ardu`): Pause (Hold, 0.37 m de deriva en 6 s a
  12 m), Resume (sigue hasta el waypoint siguiente) y Stop (Hold a 12 m): OK.
- **PX4 en vuelo (2026-10-21, SITL 1.18.0, mundo `template_obstacles`)**: `[ARM, AUTO.MISSION]`
  desde tierra, el NaN del gimbal y el `NAV_TAKEOFF` en `MAV_FRAME_GLOBAL` (AMSL) funcionan. El
  "no despega" eran **dos bugs de PX4, ninguno del frame ni de MISSION_START**:
  1. **El cursor de misión sobrevive entre misiones.** Todos los `MISSION_ITEM_INT` salían con
     `current: 0`, así que PX4 deja `_transfer_current_seq = -1` (`mavlink_mission.cpp`) y el
     navigator lo reemplaza por el índice anterior (`mission_base.cpp:101`). Con la misión previa
     terminada en su último item, la nueva arranca ahí: `Mission finished, landed`, `No valid
     mission available, refusing takeoff` y desarme automático a los ~10 s. La primera misión
     tras un arranque limpio volaba, por eso parecía intermitente. **Fix**: el encoder marca el
     item 0 como `current` (solo PX4).
  2. **`MISSION_ITEM_REACHED` de PX4 no trae el seq del waypoint.** Reporta el cursor
     (`set_mission_item_reached`) después de haber corrido los items sin posición que siguen
     (velocidad, gimbal, foto, RTL). Con `wpSeqs [0, 2, 6]` y RTL en 7 llegaron 1, 5 y 7: ninguno
     calzaba, `wp_reached` no avanzaba y la misión nunca se cerraba. **Fix**:
     `_waypointReached`: en PX4 el seq pertenece al waypoint cuyo grupo lo contiene; en
     ArduPilot sigue siendo exacto. PX4 repite el último reached cada segundo: se deduplica.
  - Mi primera ruta de prueba chocó con `wall_1` (y=15, 15 m de alto): `Attitude failure
    (roll)`. En ese mundo las rutas de prueba van al **este** (norte tiene paredes).
  - **E2E**: PX4 (RTL) → ArduCopter (land), sobre un PX4 que ya había volado varias misiones:
    T1 y T2 `end`, misión `finish`. `e2e_mission.mjs ardu` y `pause_resume.mjs ardu`: OK.
  - Tests: `test-mavlink.js` (85); los dos nuevos fallan sin su fix.
  - **Pendiente**: pausa/resume en PX4 (`pause_resume.mjs px4`), fuera de este alcance.

#### Estado F5 (2026-10-21): hecho, falta probarlo en el navegador

Diseño acordado con el usuario: primero el protocolo, después la categoría filtrada por
protocolo, y el System ID solo en mavlink. Capacidades declaradas, tipos de vehículo y enlace por
device van a `TODO.md` (raíz del GCS).

- **Server**:
  - Los protocolos siguen siendo **4** (ros, mqtt, robofleet, mavlink), cada uno con su gateway
    y su software en el robot; el `protocol` del device elige el gateway. Lo que se agregó es el
    **formato de mensaje** de una categoría (`categoryModel.messageFormatOf`, deducido de sus
    `messageType`): `mavlink` (solo lo transporta el protocolo mavlink) o `ros_msg` (lo
    transportan ros, mqtt y robofleet). El filtro ofrece las categorías cuyo formato transporta
    el protocolo elegido.
  - `GET /api/category?protocol=…` filtra; sin el parámetro devuelve lo mismo que antes.
  - `categoryModel.deviceProblem` se valida en `POST` y `PUT /api/devices`, en el controller
    porque los schemas no importan models. El `PUT` se valida sobre **el device resultante**
    (lo guardado + el parcial): cambiar solo el protocolo a mavlink sin sysid también se rechaza.
  - Tests: `test/test-device-protocol.js`.
- **Cliente**:
  - `components/devices/DeviceIdentityEditor.jsx` es el editor controlado de nombre → protocolo
    → categoría filtrada → System ID (solo mavlink, 1..254) → IP, y lo usan `Adduav` y
    `SettingsDevicesPageEdit`, que tenían el bloque duplicado.
  - La categoría se remonta por familia (`key`): `SelectField` conserva la lista vieja hasta que
    llega la nueva y si no autoseleccionaría una categoría de la familia anterior.
  - `deviceIdentity.js` es la regla compartida: `withProtocol` limpia la categoría al cambiar de
    familia y el System ID al salir de mavlink (`null`, así la edición también lo borra en la DB).
  - `Adduav` ya no envía formularios incompletos.
- **Verificado contra un `server.js` real** (puerto 4001, DB en memoria): filtro, rechazos y alta
  correcta, con telemetría del PX4 SITL en el device recién creado. Lint limpio en los archivos
  tocados.
- **Pendiente**: probar los formularios en el navegador. El GCS del puerto 4000 corre con el
  código viejo y sin `MAVLINK_CONNECTION`.

### F5 · Cliente

- `mavlink` en el selector de protocolo y el campo `systemId` (1–254) en el form del device.
- `DeviceMessagesPage` ya muestra `transport`: no hay que tocarla.

### F6 · Docs

- Sección "MAVLink Protocol" en `server/CLAUDE.md`, en Device Communication Protocols:
  pseudo-msgTypes, dialecto por `autopilot`, mapa seq→wp, ACK correlacionado.
- Variables de entorno nuevas en la sección Environment.
- Este archivo queda con los riesgos abiertos.

## Tests

- `test/test-mavlink-server.js`: enlace falso con un par de `PassThrough`, sin SITL ni hardware.
  Cubre descubrimiento por sysid, presencia y timeout, sysid desconocido ignorado, y heartbeat
  saliente.
- Decoders: un caso por slot y por firmware (modos PX4 y ArduPilot).
- Encoders de comandos: tabla por comando × autopilot. Incluye el rechazo sin `systemId` o sin
  `autopilot`.
- Correlación de ACK: ACCEPTED, IN_PROGRESS → ACCEPTED, DENIED, timeout con reintento.
- Upload de misión contra un "vehículo falso" que pide items, incluyendo items fuera de orden,
  reintentos y ACK de error, más el mapa seq→wp con el offset de ArduPilot.
- `npm test`, que usa `DB_STORAGE=:memory:`. Nunca `node --test` pelado.
- Validación manual: PX4 SITL ×2 y ArduCopter SITL ×2 en simultáneo, misión de 2 tasks con
  `depends_on`.

## Riesgos abiertos

- **R1 · DESACTUALIZADO (2026-10-25):** `calculateLocalOrigin` hoy suma `0` (`sumAlt += 0`): `origin.alt`
  es 0 y la altura de una misión es RELATIVA al despegue, no AMSL. El encoder MAVLink manda
  `MAV_FRAME_GLOBAL_RELATIVE_ALT` salvo que la tarea diga `altitude_ref: amsl`
  (`vehicle-platforms.md` › F4/F5 VTOL). Lo de abajo describe el código anterior.
  Texto original: Verificado: `ENUToGeodetic` hace
  `alt = origin.alt + z` (`coordinateConverter.js:118`), y `origin.alt` es el **promedio** de
  `position.altitude` de todos los devices de la misión (`calculateLocalOrigin`). Para un device
  MAVLink, `position.altitude` sale de `GLOBAL_POSITION_INT.alt` (AMSL), así que una misión solo
  MAVLink es coherente. Pero `sensor_msgs/NavSatFix.altitude` (DJI, mavros) suele ser altura
  **elipsoidal WGS84**, y la ondulación del geoide en España ronda los +50 m. Una misión que
  mezcla los dos tipos promedia referencias distintas, y cada drone vuela ~25–50 m fuera de lo
  planeado. Mitigación a decidir: normalizar todas las altitudes a AMSL en los decoders, con un
  param `altitude_ref: ellipsoid` por categoría y un modelo de geoide (EGM96), o como mínimo
  advertir cuando una misión mezcla referencias. Verificar en F0 con SITL.
- **R2 · Ancho de banda SiK.** Un upload de misión larga más la telemetría a rates altos satura
  la radio, y el protocolo de misión empieza a hacer timeouts. Mitigación: rates bajos por
  categoría, y timeouts de item más largos para enlaces serie.
- **R3 · SiK multipunto.** Varias radios en la misma red y canal comparten el aire. Con
  firmware multipoint, o una radio por drone en canales distintos, el modelo "un enlace por
  puerto" aguanta. Sysids únicos son obligatorios.
- **R4 · Un PX4 con ROS2 y MAVLink a la vez.** Un device tiene un solo protocolo. Si el mismo
  vehículo está dado de alta dos veces (ROS2 y MAVLink), sus comandos compiten. No se soporta:
  documentarlo.
- **R5 · `serialport` nativo.** Verificar que el `Dockerfile` del server instale los prebuilds
  (o tenga toolchain) para la arquitectura de despliegue.
- **R6 · Licencia LGPL** de `node-mavlink`. Sin problema como dependencia npm del server; tenerlo
  presente si se redistribuye cerrado.

## Fuera de alcance (v1)

- Video: va por `device.camera` (RTSP/WebRTC) y no depende del protocolo.
- Descarga de archivos (MAVLink FTP, `camera_file_*`, `sync_files`).
- MAVLink signing.
- ArduPilot Plane/Rover: solo Copter.
- Parámetros (`PARAM_*`) desde la UI.

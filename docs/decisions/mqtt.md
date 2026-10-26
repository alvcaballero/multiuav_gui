# Plan: MQTT como tercer protocolo de la GCS (CERRADO 2026-10-26)

Estado: **lado GCS terminado** (F0, F2, F4: `server/models/mqtt/`, documentado en `server/CLAUDE.md` › MQTT
Protocol). Lo que queda es del dron (fork de `mqtt_client`: F1, F3a, F3b, broker/ACL, F6 en SITL) y dos huecos
del GCS (presencia LWT, cache de `_getDevice`): todo en `TODO.md` › MQTT. Este archivo queda como registro de
decisiones y del diseño del lado dron, que no está documentado en ningún otro lado. Fecha del plan: 2026-10-14.

## Objetivo

Recibir telemetría de los drones y comandar sus servicios ROS por MQTT, en paralelo a `ros` (rosbridge) y `robofleet` (FlatBuffers). El device elige su transporte con `protocol: 'mqtt'`.

## Decisiones tomadas

- Alcance: **PX4 / ROS 2 (Jazzy)**. ROS 1 (DJI Noetic) queda fuera; `mqtt_client` es solo ROS 2. Puede escalar a más robots.
- MQTT **convive** con rosbridge, no lo reemplaza.
- `mqtt_client` corre **a bordo de cada dron**. El **broker (mosquitto) corre en la máquina de la GCS**.
- Servicios: **fork de `ika-rwth-aachen/mqtt_client`** que suma un bridge de servicios ROS con `rclcpp::GenericClient` + `rosx_introspection`. Un solo nodo a bordo para telemetría y servicios. La alternativa `rclpy` (nodo aparte) queda como **plan de reserva** si el spike de F3a falla (ver riesgos).
- El `id` de correlación va **en el payload JSON** con un topic de respuesta único, no en MQTT 5 Correlation Data ni en un topic por id (`.../response/<id>`). Una sola GCS lee las respuestas, así que la suscripción efímera por llamada no aporta nada y es más difícil de depurar con `mosquitto_sub`.
- Transitive Robotics: **descartado**. Corre sobre MQTTSync (otro modelo de datos) y trae runtime de sandboxes y JWT. Lo único que se toma es el patrón RPC genérico sobre MQTT (request con `id`, respuesta con `id`), que ya es el nuestro, con lo que su `call()` no hace: **timeout del lado cliente**.

## Contrato MQTT

| Qué | Topic |
|---|---|
| Telemetría dron → GCS | `muav/<device>/pub/<slot>` |
| Request GCS → dron | `muav/<device>/srv/<name>/request` |
| Response dron → GCS | `muav/<device>/srv/<name>/response` |
| Presencia (LWT, retained) | `muav/<device>/status` |

- `<device>` = `name` del device en la DB (mismo namespace ROS que arma `buildDeviceName`).
- `<slot>` y `<name>` salen de `devices_msg.yaml` (`subscribers` / `services`).
- Telemetría: JSON con la misma forma que entrega rosbridge, así `decodeMsg` se reutiliza tal cual.

Servicios:

```json
// request  (retain=false)
{ "id": "<uuid>", "ts": 1760000000000, "request": { } }

// response
{ "id": "<uuid>", "success": true, "response": { }, "error": "" }
```

## Arquitectura

```
 DRON (PX4/ROS2)                    GCS (Node.js + mosquitto)
 ┌─────────────────────┐            ┌──────────────────────┐
 │ mqtt_client (FORK)  │─ pub/slot ▶│ models/mqtt/         │
 │  ├ bridge de topics │            │   MqttServer.js      │
 │  │  (upstream)      │            │                      │
 │  └ bridge de        │◀ request ──│  decodeMsg →      │
 │    servicios (nuevo)│─ response ▶│  positionsController │
 │    GenericClient    │            │  pending[id]         │
 └─────────────────────┘            └──────────────────────┘
```

### Pieza 1: telemetría (parte upstream del nodo)

- Una instancia por dron. `bridge.ros2mqtt` con `json: true` + `ros_type` por topic.
- Mapeo `/<dev>/<topic ROS>` → `muav/<dev>/pub/<slot>`, tomado de `devices_msg.yaml`.
- Fijar `client.id` (sin él no se puede usar el buffer de cliente).
- Presencia: `mqtt_client` ya expone configuración de last will; se usa para `muav/<device>/status`.

### Pieza 2: bridge de servicios (el fork)

Se agrega a `mqtt_client` siguiendo su propio patrón: configuración por parámetros ROS, igual que `bridge.ros2mqtt` / `bridge.mqtt2ros`.

```yaml
bridge:
  services:
    names: [command_mission, configure_mission]
    command_mission:
      ros_service: /dji_control/start_mission
      ros_type: std_srvs/srv/SetBool
      mqtt_topic: muav/uav1/srv/command_mission   # /request y /response se derivan
      timeout_s: 5.0
```

Agregar un servicio = agregar una entrada al YAML, sin recompilar.

Por cada request:

1. Suscripción a `<mqtt_topic>/request`; parsear el sobre `{id, ts, request}`.
2. Si el request es viejo (`ts`), descartarlo (ver Seguridad: depende de sincronía de relojes).
3. Si el servicio no está disponible (`wait_for_service(0)`), responder error de inmediato.
4. JSON → request ROS con `rosx_introspection`, `GenericClient::async_send_request`, sin bloquear.
5. Al llegar la respuesta o vencer `timeout_s`: ROS → JSON y publicar `{id, success, response, error}` en `<mqtt_topic>/response`. Limpiar pendientes (`prune_requests_older_than`).

Aislamiento del fork: el código nuevo va en archivos propios (`ServiceBridge.hpp/.cpp`), con el diff mínimo en `MqttClient.cpp`, para poder rebasar contra upstream y proponer el PR al issue #72 (soporte de servicios y acciones, abierto desde 2024).

Notas:

- `devices_msg.yaml` usa tipos estilo ROS 1 (`std_srvs/SetBool`); en ROS 2 el tipo es `std_srvs/srv/SetBool`. La GCS (o el generador de config) normaliza.

### Pieza 3: servidor (`server/models/mqtt/`, espejo de `models/flatbuffer/`)

- Suscripción a `muav/+/pub/+` → device por nombre → `messageType` de la categoría → `decodeMsg` → `positionsController.updatePosition`.
- `sendCommand`: publica el request con `uuid`, guarda `pending[id] = {resolve, timer}`, devuelve una Promise que resuelve la respuesta real o el timeout. (robofleet devuelve `success` al enviar; acá no.)
- Cambios en archivos existentes:
  - `schemas/zod/devices.js:4`: agregar `'mqtt'` a `DEVICE_PROTOCOL`.
  - `models/commands.js`: rama `if (protocol == 'mqtt')` en `standarCommand`.
  - `config/config.js` + `server.js`: flag `MqttEnable` y arranque.
  - `package.json`: dependencia `mqtt`.
  - `server/CLAUDE.md`: documentar el protocolo.

## Riesgo técnico principal: GenericClient ↔ JSON

`GenericClient` trabaja con `void*` (`Request`/`Response`) y el header los llama "serialized data pointer". **No está confirmado si eso es CDR o la struct C del mensaje en memoria** (las fuentes `.cpp` de `rclcpp` no están instaladas, solo los headers).

- Si es **CDR**: `rosx_introspection` (JSON ↔ CDR) encaja directo, igual que ya hace `mqtt_client` con topics.
- Si es la **struct**: hay un paso extra JSON → CDR → deserializar a la struct con el typesupport del request (y lo inverso para la respuesta). Es más C++ y más superficie de error.
- Además: `rosx_introspection` necesita la **definición** del tipo. `mqtt_client` ya la obtiene con `GetMessageDefinition(type)` para mensajes; **no está verificado** que sirva para `Name_Request` / `Name_Response` de un `.srv`.

Por eso F3 se divide en un spike (F3a) antes de construir el bridge completo. Si F3a falla, se vuelve al plan de reserva `rclpy` (`get_service`, `set_message_fields`, `message_to_ordereddict` de `rosidl_runtime_py`, todos presentes en la instalación). El contrato y el servidor no cambian.

## Seguridad (obligatorio, no opcional)

Un request de `command_mission` arma un dron.

1. **Replay**: con QoS 1 y sesión persistente el broker puede entregar un comando viejo al reconectar. Mitigación principal: sesión **limpia** del dron (`clean_session=true`) y `retain=false`, así no hay cola de mensajes para un dron offline. Mitigación secundaria: descartar por `ts`.
2. **Reloj**: el descarte por `ts` compara el reloj de la GCS contra el del dron. Si el dron no tiene NTP/chrony, el TTL falla en un sentido o en otro. Requiere sincronía de relojes (o usar el *message expiry interval* de MQTT 5 en vez de `ts`).
3. **Escritura abierta**: sin auth/ACL cualquiera en el broker comanda drones. ACL en mosquitto: solo la GCS escribe en `srv/+/request`; cada dron solo escribe en `muav/<su device>/#`.
4. **Presencia**: LWT retained en `muav/<device>/status` para detectar un dron caído.
5. **Timeout en la GCS**: toda llamada tiene timeout propio y limpia su pendiente; nunca queda colgada.

## Roadmap

| Fase | Qué | Criterio de salida |
|---|---|---|
| F0 | Instalar `mosquitto` y el paquete npm `mqtt`. Prueba manual con `mosquitto_pub/sub`. | Un mensaje va y vuelve por el broker. |
| F1 | Config de `mqtt_client` (upstream) + publisher falso de `NavSatFix`. | Llega JSON a `muav/<dev>/pub/position`. |
| F2 | `models/mqtt/` + tests con cliente inyectado (sin broker real). | `npm test` pasa; la posición llega a `positionsController`. |
| **F3a** | **Spike**: programa mínimo con `GenericClient` que llama `std_srvs/srv/SetBool` desde JSON y devuelve JSON. Resolver CDR vs struct y la definición de tipos del `.srv`. | Un `SetBool` ida y vuelta con JSON. Decisión: seguir con el fork o volver a `rclpy`. |
| F3b | Bridge de servicios completo en el fork (`ServiceBridge`), con config por parámetros, timeout, error rápido y `id`. | Request → response con el mismo `id`; error rápido si el servicio no existe; timeout cierra el pendiente. |
| F4 | Protocolo `mqtt` en schema, `standarCommand`, flags, docs. | Un comando desde la GCS llega al servicio y la respuesta vuelve. |
| F5 | Sesión limpia, ACL/auth, LWT, reconexión, relojes. | Dron caído detectado; escritura no autorizada rechazada por el broker. |
| F6 | PX4 SITL y luego dron real. | Misión cargada y ejecutada de punta a punta. |

## Verificado vs. pendiente

Verificado:

- `mqtt_client` (ika-rwth-aachen) solo puentea topics; sus únicos servicios son `is_connected`, `new_ros2mqtt_bridge` y `new_mqtt2ros_bridge`. Ya usa `rosx_introspection` para JSON ↔ mensaje y `GetMessageDefinition(type)` (leído en el código local).
- `rclcpp` 28.1.22 (Jazzy) instalado trae `generic_client.hpp`; expone `async_send_request(const Request)` con `Request = void*` y `prune_requests_older_than`.
- `rosidl_runtime_py` instalado trae `utilities.py`, `set_message.py`, `convert.py` (plan de reserva).
- `mosquitto` y `paho-mqtt` **no** están instalados en esta máquina.
- No existe un bridge ROS 2 ↔ MQTT genérico con servicios y correlación (según una búsqueda por fetch, sin leer el código fuente de cada candidato). El issue #72 de `mqtt_client` sigue abierto.

Pendiente de probar:

- CDR vs struct en `GenericClient` y definición de tipos para `.srv` (F3a).
- Conversión de respuestas con tipos anidados y arrays.
- Comportamiento de `mqtt_client` con QoS `best_effort` de los topics de `px4_msgs`.
- Sincronía de relojes dron ↔ GCS.

## Fuera de alcance (primera versión)

- Acciones ROS (la GCS las usa, p. ej. `camera_file_download`). Quedan para después; el mismo diseño del bridge de servicios podría extenderse.
- Publishers GCS → dron (`mqtt2ros` de `mqtt_client` lo soporta, sin diseñar acá).
- Video / cámaras.
- Autodescubrimiento de servicios (Transitive publica su lista cada 2 min; acá se declara en YAML).

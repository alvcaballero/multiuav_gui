// Device-msg catalog — SSOT de TODAS las keys válidas en devices_msg.yaml.
//
// Un device declara sus capacidades ROS en cuatro bloques: `subscribers:`,
// `publishers:`, `services:` y `actions:`. Las keys NO son el nombre ROS (ese es `name:`):
// nombran el slot, y todas van en snake_case (convención ROS para topics/services, y no se
// confunden con los CommandType PascalCase: `command_mission` es el endpoint que implementa
// CommandMission). Este módulo las enumera y es el único lugar donde viven esos literales.
//
// OJO: la key viaja en el cable MQTT (muav/<device>/pub/<slot>, muav/<device>/srv/<key>, ver
// models/mqtt/mqttTopics.js): renombrar una obliga a actualizar el mqtt_client del dron.
// Robofleet no: fbEncode/fbDecode tienen sus propios nombres de topic.
//
// subscribers → tópicos a los que el server se AUTOSUSCRIBE al arrancar (telemetría
// entrante); sus keys son de telemetría (position, battery...) → SubscriberKey.
// publishers → tópicos sobre los que el server PUBLICA bajo demanda como respuesta
// a un COMANDO de usuario; sus keys son capacidades invocables (gimbal_control...), misma
// naturaleza que services/actions → PublisherKey.
//
// Es config del DEVICE, no de los comandos. Qué command implementa cada endpoint, y
// cómo se arma su mensaje, lo decide models/commands/commandRegistry.js (por msgType, o por estas
// keys cuando el msgType es genérico); los literales de commands viven en commandTypes.js.
//
// Bloques y su enum/validación:
//   subscribers:       → SubscriberKey → KNOWN_SUBSCRIBER_KEYS
//   publishers:        → PublisherKey  → KNOWN_PUBLISHER_KEYS
//   services:/actions: → ServiceKey    → KNOWN_SERVICE_KEYS
//
// category.js valida cada bloque del YAML contra su set de keys conocidas.

// ─── Subscribers ────────────────────────────────────────────────────────────────

// subscriber keys — strings de `type` que circulan entre el YAML (keys de
// subscribers:), rosSubscriptions (suscripción) y rosDecode (comparación). Usalo en
// rosDecode.js:
// `if (type == SubscriberKey.POSITION_GLOBAL)` en vez del literal 'position_global'.
export const SubscriberKey = Object.freeze({
  POSITION: 'position',
  POSITION_GLOBAL: 'position_global',
  LOCAL_POSITION: 'local_position',
  VEHICLE_STATUS: 'vehicle_status',
  VEHICLE_COMMAND_ACK: 'vehicle_command_ack',
  IMU: 'imu',
  HEADING: 'heading',
  MISSION_STATE: 'mission_state',
  SPEED: 'speed',
  BATTERY: 'battery',
  GIMBAL: 'gimbal',
  OBSTACLE_INFO: 'obstacle_info',
  CAMERA: 'camera',
  FLIGHT_STATUS: 'flight_status',
  ATTITUDE: 'attitude',
  HUMIDITY: 'humidity',
  THREAT: 'threat',
  STATE_MACHINE: 'state_machine',
  HEIGHT_ABOVE_TAKEOFF: 'height_above_takeoff',
  VO_POSITION: 'vo_position',
});

// subscriberKey → { decoded }. Indexado por SubscriberKey (el enum es la fuente de
// los literales).
//   decoded: true   → rosDecode.js tiene una rama `if (type == '<key>')`.
//   decoded: false  → key válida y suscribible pero SIN decodificador (comentada
//                     en el decoder, o de ciclo action). No es un error; el
//                     mensaje se recibe pero no se consume.
export const SUBSCRIBER_CATALOG = {
  [SubscriberKey.POSITION]: { decoded: true },
  [SubscriberKey.POSITION_GLOBAL]: { decoded: true },
  [SubscriberKey.LOCAL_POSITION]: { decoded: true },
  [SubscriberKey.VEHICLE_STATUS]: { decoded: true },
  [SubscriberKey.VEHICLE_COMMAND_ACK]: { decoded: true },
  [SubscriberKey.IMU]: { decoded: true },
  [SubscriberKey.HEADING]: { decoded: true },
  [SubscriberKey.MISSION_STATE]: { decoded: true },
  [SubscriberKey.SPEED]: { decoded: true },
  [SubscriberKey.BATTERY]: { decoded: true },
  [SubscriberKey.GIMBAL]: { decoded: true },
  [SubscriberKey.OBSTACLE_INFO]: { decoded: true },
  [SubscriberKey.CAMERA]: { decoded: true },
  [SubscriberKey.FLIGHT_STATUS]: { decoded: true },
  [SubscriberKey.ATTITUDE]: { decoded: true },
  [SubscriberKey.HUMIDITY]: { decoded: true },
  [SubscriberKey.THREAT]: { decoded: true },
  [SubscriberKey.STATE_MACHINE]: { decoded: true },
  // Válidos/suscribibles pero sin decodificador (deshabilitados o de ciclo action).
  [SubscriberKey.HEIGHT_ABOVE_TAKEOFF]: { decoded: false }, // ignore() en rosDecode.js
  [SubscriberKey.VO_POSITION]: { decoded: false }, // ignore() en rosDecode.js
};

// subscriberKeys conocidas — para validar el bloque subscribers: de devices_msg.
export const KNOWN_SUBSCRIBER_KEYS = new Set(Object.values(SubscriberKey));

// ─── Publishers ─────────────────────────────────────────────────────────────────

// publisher keys — capacidades que el server PUBLICA como respuesta a un comando
// de usuario (no telemetría). Un mismo command puede resolverse por service, action o
// publisher según la categoría (commandRegistry.js).
export const PublisherKey = Object.freeze({
  GIMBAL_CONTROL: 'gimbal_control',
  // PX4 /fmu/in/vehicle_command (px4_msgs/msg/VehicleCommand). Habilita los comandos
  // de vuelo directos (Takeoff, Land, ReturnHome, Hold, Arm/Disarm, KillMotors, MoveRelative).
  VEHICLE_COMMAND: 'vehicle_command',
});

// publisherKeys conocidas — para validar el bloque publishers: de devices_msg.
export const KNOWN_PUBLISHER_KEYS = new Set(Object.values(PublisherKey));

// ─── Services / Actions ───────────────────────────────────────────────────────

// serviceKeys — strings que son key EXACTA en devices_msg.yaml (services:/actions:)
// y que el ROS gateway invoca como service/action ROS. Incluye tanto
// services como actions (ambos bloques invocables). commandRegistry.js las usa para
// reconocer los commands que van sobre un msgType genérico (std_srvs/SetBool, Trigger…).
export const ServiceKey = Object.freeze({
  CONFIGURE_MISSION: 'configure_mission',
  COMMAND_MISSION: 'command_mission',
  SYNC_FILES: 'sync_files',
  SETUP_CAMERA: 'setup_camera',
  GIMBAL_CONTROL: 'gimbal_control',
  RESUME_MISSION: 'resume_mission',
  STOP_MISSION: 'stop_mission',
  PAUSE_MISSION: 'pause_mission',
  UPLOAD_MISSION: 'upload_mission',
  CAMERA_FILE_DOWNLOAD: 'camera_file_download',
  CAMERA_FILE_LIST: 'camera_file_list',
  NAVIGATE_TO_POSE: 'navigate_to_pose',
  THREAT_CONFIRMATION: 'threat_confirmation',
  THREAT_DEFUSE: 'threat_defuse',
  // px4_ros2 /offboard/command (muav_gcs_interfaces/srv/OffboardCommand). Todavía ningún
  // command lo requiere: en definición.
  OFFBOARD_COMMAND: 'offboard_command',
});

// serviceKeys conocidas — para validar los bloques services:/actions: de devices_msg.
export const KNOWN_SERVICE_KEYS = new Set(Object.values(ServiceKey));

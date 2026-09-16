// Device-msg catalog — SSOT de TODAS las keys válidas en devices_msg.yaml.
//
// Un device declara sus capacidades ROS en cuatro bloques: `subscribers:`,
// `publishers:`, `services:` y `actions:`. Las keys de esos bloques están FIJADAS
// por el lado ROS (el nombre del topic/.srv/.action del UAV); no se renombran acá.
// Este módulo las enumera y es el único lugar donde viven esos literales.
//
// subscribers → tópicos a los que el server se AUTOSUSCRIBE al arrancar (telemetría
// entrante); sus keys son de telemetría (position, battery...) → SubscriberKey.
// publishers → tópicos sobre los que el server PUBLICA bajo demanda como respuesta
// a un COMANDO de usuario; sus keys son capacidades invocables (Gimbal...), misma
// naturaleza que services/actions → PublisherKey (PascalCase).
//
// Es config del DEVICE, no de los comandos. La lógica de commands (qué acción
// dispara el usuario, cómo se despacha) vive en commandCatalog.js, que IMPORTA
// `ServiceKey` de acá — la relación es command → requires → serviceKey, no al revés.
//
// Bloques y su enum/validación:
//   subscribers:       → SubscriberKey → KNOWN_SUBSCRIBER_KEYS
//   publishers:        → PublisherKey  → KNOWN_PUBLISHER_KEYS
//   services:/actions: → ServiceKey    → KNOWN_SERVICE_KEYS
//
// category.js valida cada bloque del YAML contra su set de keys conocidas.

// ─── Subscribers ────────────────────────────────────────────────────────────────

// subscriber keys — strings de `type` que circulan entre el YAML (keys de
// subscribers:), rosTopics (suscripción) y rosDecode (comparación). Usalo en
// rosDecode.js:
// `if (type == SubscriberKey.POSITION_GLOBAL)` en vez del literal 'position_global'.
export const SubscriberKey = Object.freeze({
  POSITION: 'position',
  POSITION_GLOBAL: 'position_global',
  LOCAL_POSITION: 'local_position',
  VEHICLE_STATUS: 'vehicle_status',
  VEHICLE_COMMAND_ACK: 'vehicle_command_ack',
  IMU: 'IMU',
  HDG: 'hdg',
  MISSION_STATE: 'mission_state',
  SPEED: 'speed',
  BATTERY: 'battery',
  GIMBAL: 'gimbal',
  OBSTACLE_INFO: 'obstacle_info',
  CAMERA: 'camera',
  FLIGHT_STATUS: 'flight_status',
  ATTITUDE: 'attitude',
  SENSORS_HUMIDITY: 'sensors_humidity',
  THREAT: 'threat',
  STATE_MACHINE: 'state_machine',
  SENSOR_HEIGHT: 'sensor_height',
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
  [SubscriberKey.HDG]: { decoded: true },
  [SubscriberKey.MISSION_STATE]: { decoded: true },
  [SubscriberKey.SPEED]: { decoded: true },
  [SubscriberKey.BATTERY]: { decoded: true },
  [SubscriberKey.GIMBAL]: { decoded: true },
  [SubscriberKey.OBSTACLE_INFO]: { decoded: true },
  [SubscriberKey.CAMERA]: { decoded: true },
  [SubscriberKey.FLIGHT_STATUS]: { decoded: true },
  [SubscriberKey.ATTITUDE]: { decoded: true },
  [SubscriberKey.SENSORS_HUMIDITY]: { decoded: true },
  [SubscriberKey.THREAT]: { decoded: true },
  [SubscriberKey.STATE_MACHINE]: { decoded: true },
  // Válidos/suscribibles pero sin decodificador (deshabilitados o de ciclo action).
  [SubscriberKey.SENSOR_HEIGHT]: { decoded: false }, // decoder comentado — rosDecode.js:78
  [SubscriberKey.VO_POSITION]: { decoded: false }, // decoder comentado — rosDecode.js:85
};

// subscriberKeys conocidas — para validar el bloque subscribers: de devices_msg.
export const KNOWN_SUBSCRIBER_KEYS = new Set(Object.values(SubscriberKey));

// ─── Publishers ─────────────────────────────────────────────────────────────────

// publisher keys — capacidades que el server PUBLICA como respuesta a un comando
// de usuario (no telemetría). Su casing es PascalCase porque son keys de comando,
// no de topic: matchean el `type` que despacha commandsModel (ej. 'Gimbal'). Un
// mismo command puede resolverse por service, action o publisher según la categoría.
export const PublisherKey = Object.freeze({
  GIMBAL: 'Gimbal',
});

// publisherKeys conocidas — para validar el bloque publishers: de devices_msg.
export const KNOWN_PUBLISHER_KEYS = new Set(Object.values(PublisherKey));

// ─── Services / Actions ───────────────────────────────────────────────────────

// serviceKeys — strings que son key EXACTA en devices_msg.yaml (services:/actions:)
// y que se pasan a standarCommand/callService como service ROS. Incluye tanto
// services como actions (ambos bloques invocables). commandCatalog.js las
// referencia vía el campo `requires` de cada command.
export const ServiceKey = Object.freeze({
  CONFIGURE_MISSION: 'configureMission',
  COMMAND_MISSION: 'commandMission',
  SINCRONIZE: 'sincronize',
  SETUP_CAMERA: 'setupcamera',
  GIMBAL: 'Gimbal',
  RESUME_MISSION: 'resumemission',
  STOP_MISSION: 'stopMission',
  PAUSE_MISSION: 'pausemission',
  UPLOAD_MISSION: 'uploadMission',
  CAMERA_FILE_DOWNLOAD: 'CameraFileDownload',
  CAMERA_FILE_LIST: 'CameraFileList',
  NAVIGATE_TO_POSE: 'navigateToPose',
  THREAT_CONFIRMATION: 'threat_confirmation',
  THREAT_DEFUSE: 'threat_defuse',
});

// serviceKeys conocidas — para validar los bloques services:/actions: de devices_msg.
export const KNOWN_SERVICE_KEYS = new Set(Object.values(ServiceKey));

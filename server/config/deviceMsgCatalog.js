// Device-msg catalog — SSOT de TODAS las keys válidas en devices_msg.yaml.
//
// Un device declara sus capacidades ROS en tres bloques: `topics:`, `services:`
// y `actions:`. Las keys de esos bloques están FIJADAS por el lado ROS (el nombre
// del topic/.srv/.action del UAV); no se renombran acá. Este módulo las enumera y
// es el único lugar donde viven esos literales.
//
// Es config del DEVICE, no de los comandos. La lógica de commands (qué acción
// dispara el usuario, cómo se despacha) vive en commandCatalog.js, que IMPORTA
// `ServiceKey` de acá — la relación es command → requires → serviceKey, no al revés.
//
// Bloques y su enum/validación:
//   topics:            → TopicKey    → KNOWN_TOPIC_KEYS
//   services:/actions: → ServiceKey  → KNOWN_SERVICE_KEYS
//
// category.js valida cada bloque del YAML contra su set de keys conocidas.

// ─── Topics ───────────────────────────────────────────────────────────────────

// topic keys — strings de `type` que circulan entre el YAML (keys de topics:),
// rosTopics (suscripción) y rosDecode (comparación). Usalo en rosDecode.js:
// `if (type == TopicKey.POSITION_GLOBAL)` en vez del literal 'position_global'.
export const TopicKey = Object.freeze({
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

// topicKey → { decoded }. Indexado por TopicKey (el enum es la fuente de los
// literales).
//   decoded: true   → rosDecode.js tiene una rama `if (type == '<key>')`.
//   decoded: false  → key válida y suscribible pero SIN decodificador (comentada
//                     en el decoder, o de ciclo action). No es un error; el
//                     mensaje se recibe pero no se consume.
export const TOPIC_CATALOG = {
  [TopicKey.POSITION]: { decoded: true },
  [TopicKey.POSITION_GLOBAL]: { decoded: true },
  [TopicKey.LOCAL_POSITION]: { decoded: true },
  [TopicKey.VEHICLE_STATUS]: { decoded: true },
  [TopicKey.VEHICLE_COMMAND_ACK]: { decoded: true },
  [TopicKey.IMU]: { decoded: true },
  [TopicKey.HDG]: { decoded: true },
  [TopicKey.MISSION_STATE]: { decoded: true },
  [TopicKey.SPEED]: { decoded: true },
  [TopicKey.BATTERY]: { decoded: true },
  [TopicKey.GIMBAL]: { decoded: true },
  [TopicKey.OBSTACLE_INFO]: { decoded: true },
  [TopicKey.CAMERA]: { decoded: true },
  [TopicKey.FLIGHT_STATUS]: { decoded: true },
  [TopicKey.ATTITUDE]: { decoded: true },
  [TopicKey.SENSORS_HUMIDITY]: { decoded: true },
  [TopicKey.THREAT]: { decoded: true },
  [TopicKey.STATE_MACHINE]: { decoded: true },
  // Válidos/suscribibles pero sin decodificador (deshabilitados o de ciclo action).
  [TopicKey.SENSOR_HEIGHT]: { decoded: false }, // decoder comentado — rosDecode.js:78
  [TopicKey.VO_POSITION]: { decoded: false }, // decoder comentado — rosDecode.js:85
};

// topicKeys conocidas — para validar el bloque topics: de devices_msg.
export const KNOWN_TOPIC_KEYS = new Set(Object.values(TopicKey));

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

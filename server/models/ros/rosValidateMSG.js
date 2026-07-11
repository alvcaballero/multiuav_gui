export function buildTypeMap(definitions) {
  const typeMap = {};
  for (const def of definitions) {
    typeMap[def.type] = def;
  }
  return typeMap;
}

function validatePrimitiveType(val, fieldType, type) {
  switch (fieldType) {
    case 'double':
    case 'float':
      if (typeof val !== 'number') {
        throw new Error(`Field '${fieldName}' in ${type} must be a number`);
      }
      break;
    case 'int32':
    case 'int64':
    case 'uint32':
      if (!Number.isInteger(val)) {
        throw new Error(`Field '${fieldName}' in ${type} must be an integer`);
      }
      break;
    case 'string':
      if (typeof val !== 'string') {
        throw new Error(`Field '${fieldName}' in ${type} must be a string`);
      }
      break;
    default:
      // otros tipos primitivos...
      break;
  }

  //return primitiveTypes.includes(type);
}

export function validateRosMsg(typeMsg, msg, typeMap, Checkallparrams = true) {
  let type = typeMsg;
  if (type.includes('/msg/')) {
    type = typeMsg.replace('/msg/', '/');
  }
  if (type.includes('/srv/')) {
    type = typeMsg.replace('/srv/', '/');
    type = `${type}_Request`;
  }
  //console.log("Validating message of type:", type);
  //console.log("Validating message data:", msg);
  // ROS1: el bridge puede retornar el tipo con sufijo "Request" (ej: ConfigMissionRequest)
  // en vez del nombre base (ConfigMission). Si no se encuentra, intentar con el sufijo.
  let def = typeMap[type];
  if (!def && typeMap[`${type}Request`]) {
    def = typeMap[`${type}Request`];
  }
  if (!def) {
    throw new Error(`No definition found for type ${type}`);
  }
  if (msg === null || msg === undefined) {
    if (Checkallparrams) {
      throw new Error(`Message of type ${type} is null or undefined`);
    }
    return true;
  }

  // verify defined fields
  const expectedFields = def.fieldnames.map((fn) => fn.replace(/^_/, ''));
  for (const key of Object.keys(msg)) {
    if (!expectedFields.includes(key)) {
      throw new Error(
        `Invalid field '${key}' in message for type ${type}. Valid fields are: ${expectedFields.join(', ')}`
      );
    }
  }

  // verify message structure
  for (let i = 0; i < def.fieldnames.length; i++) {
    const fieldName = def.fieldnames[i].replace(/^_/, ''); // quito el "_" inicial
    const fieldType = def.fieldtypes[i];
    const fieldArrayLen = def.fieldarraylen[i];

    if (!(fieldName in msg)) {
      if (Checkallparrams) {
        throw new Error(`Missing field '${fieldName}' in message of type ${type}`);
      } else {
        //console.warn(`Warning: Missing field '${fieldName}' in message of type ${type}`);
        continue;
      }
    }

    const val = msg[fieldName];

    if (fieldArrayLen >= 0 && Array.isArray(val) && val.length > 0) {
      // es un array
      //console.log(`Field '${fieldName}' is an array of type '${fieldType}' with length ${val.length}`);
      for (const item of val) {
        if (fieldType.includes('/')) {
          validateRosMsg(fieldType, item, typeMap, false);
        } else {
          validatePrimitiveType(item, fieldType, type);
        }
      }
    } else {
      // es tipo primitivo
      //console.log(`Field '${fieldName}' is of type '${fieldType}' with value:`, val);
      if (fieldType.includes('/')) {
        validateRosMsg(fieldType, val, typeMap, false);
      } else {
        validatePrimitiveType(val, fieldType, type);
      }
    }
  }

  return true;
}

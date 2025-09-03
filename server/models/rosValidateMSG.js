export function buildTypeMap(definitions) {
  const typeMap = {};
  for (const def of definitions) {
    typeMap[def.type] = def;
  }
  return typeMap;
}

export function validateRosMsg(typeMsg, msg, typeMap) {
  let type = typeMsg;
  if (type.includes("/msg/")) {
    type = typeMsg.replace("/msg/", "/");
  }

  const def = typeMap[type];
  if (!def) {
    throw new Error(`No definition found for type ${type}`);
  }
  // verify defined fields
  const expectedFields = def.fieldnames.map(fn => fn.replace(/^_/, ""));
  for (const key of Object.keys(msg)) {
    if (!expectedFields.includes(key)) {
      throw new Error(
        `Invalid field '${key}' in message for type ${type}. Valid fields are: ${expectedFields.join(", ")}`
      );
    }
  }

  // verify message structure
  for (let i = 0; i < def.fieldnames.length; i++) {
    const fieldName = def.fieldnames[i].replace(/^_/, ""); // quito el "_" inicial
    const fieldType = def.fieldtypes[i];

    if (!(fieldName in msg)) {
      throw new Error(`Missing field '${fieldName}' in message of type ${type}`);
    }

    if (fieldType.startsWith("geometry_msgs/") || fieldType.includes("/")) {
      // es otro mensaje compuesto → validar recursivamente
      validateRosMsg(fieldType, msg[fieldName], typeMap);
    } else {
      // es tipo primitivo
      const val = msg[fieldName];
      switch (fieldType) {
        case "double":
        case "float":
          if (typeof val !== "number") {
            throw new Error(`Field '${fieldName}' in ${type} must be a number`);
          }
          break;
        case "int32":
        case "int64":
        case "uint32":
          if (!Number.isInteger(val)) {
            throw new Error(`Field '${fieldName}' in ${type} must be an integer`);
          }
          break;
        case "string":
          if (typeof val !== "string") {
            throw new Error(`Field '${fieldName}' in ${type} must be a string`);
          }
          break;
        default:
          // otros tipos primitivos...
          break;
      }
    }
  }

  return true;
}

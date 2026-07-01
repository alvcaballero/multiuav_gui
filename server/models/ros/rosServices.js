import * as ROSLIB from 'roslib';
import { readDataFile } from '../../common/utils.js';
import { encodeRosSrv } from './rosEncode.js';
import { buildTypeMap, validateRosMsg } from './rosValidateMSG.js';
import { getServices, getServicesType, getServiceRequestDetails } from './rosInspect.js';
import logger from '../../common/logger.js';

const devices_msg = readDataFile('../config/devices/devices_msg.yaml');

// Module-level registry used by GCSServicesMission / GCSunServicesMission.
// Keyed by service name (e.g. 'ServiceMission') → ROSLIB.Service instance.
const service_list = {};

export async function callRosService({ service, messageType, message }, ros) {
  if (!ros || !ros.isConnected) throw new Error('ROS not connected');

  logger.debug(`Calling ROS service -'${service}'- with message type -'${messageType}'-`);

  if (!service || typeof service !== 'string') {
    throw new Error('The "service" parameter is required and must be a string');
  }
  if (!messageType || typeof messageType !== 'string') {
    throw new Error('The "messageType" parameter is required and must be a string');
  }
  if (message === undefined) {
    throw new Error('The "message" parameter is required');
  }

  const servicesResult = await getServices(ros);
  const servicesList = Array.isArray(servicesResult) ? servicesResult : servicesResult.services || [];
  if (!servicesList.includes(service)) {
    logger.debug(`Available services: ${JSON.stringify(servicesList)}`);
    throw new Error(`Service '${service}' not available`);
  }

  const topicType = await getServicesType(service, ros);
  if (!topicType) {
    throw new Error(`Service '${service}' has no type info — the node may not be running`);
  }
  logger.debug(`service type: ${JSON.stringify(topicType)}`);

  const auxtopicType = topicType.replace('/srv/', '/');
  if (!(topicType == messageType || auxtopicType == messageType)) {
    throw new Error(`Service type mismatch: expected '${topicType}', got '${messageType}'`);
  }

  const responseSrvStructure = await getServiceRequestDetails(messageType, ros);
  logger.debug(`Service request structure: ${JSON.stringify(responseSrvStructure)}`);
  const srvStructure = responseSrvStructure.typedefs || [];

  if (!srvStructure || srvStructure.length === 0) {
    throw new Error(`No structure found for service type '${messageType}'`);
  }

  const typeMap = buildTypeMap(srvStructure);

  validateRosMsg(messageType, message, typeMap);

  let Message = new ROSLIB.Service({
    ros: ros,
    name: service,
    serviceType: messageType,
  });

  let MsgRequest = message ? message : {};

  return new Promise((resolve, rejects) => {
    Message.callService(
      MsgRequest,
      (result) => {
        logger.debug(`Service call result for ${service}: ${JSON.stringify(result)}`);
        resolve(result);
      },
      (error) => {
        logger.debug(`Error calling service ${service}: ${error}`);
        rejects(error);
      }
    );
  });
}

export async function callService({ name, category, type, request }, ros) {
  if (!ros || !ros.isConnected) throw new Error('ROS not connected');

  if (!devices_msg[category]['services'].hasOwnProperty(type)) {
    return { state: 'warning', msg: type + ' to:' + name + ' dont have this service' };
  }

  let msgType = devices_msg[category]['services'][type]['serviceType'];
  let myRequest = encodeRosSrv({ type, msg: request, msgType: msgType });
  const service = `/${name}${devices_msg[category]['services'][type]['name']}`;
  try {
    const response = await callRosService({ service, messageType: msgType, message: myRequest }, ros);
    if (response.success || response.result) {
      return { state: 'success', msg: type + ' to ' + name + ' ok' };
    } else {
      return { state: 'error', msg: type + ' to  ' + name + ' error' };
    }
  } catch (error) {
    const errMsg = error instanceof Error ? error.message : String(error);
    logger.error(`Error calling service: ${errMsg}`);
    return { state: 'error', msg: 'Failed to call service: ' + errMsg };
  }
}

export function serviceServer({ serviceName, serviceType, callback }, ros) {
  const service = new ROSLIB.Service({
    ros: ros,
    name: serviceName,
    serviceType: serviceType,
  });

  // Tap the raw WebSocket to detect if rosbridge dispatches the call_service
  // message to our handler. If we see the message here but NOT the REQUEST log
  // below, rosbridge is dropping it internally before calling advertise handlers.
  if (ros.socket) {
    const _origOnMessage = ros.socket.onmessage;
    ros.socket.onmessage = function (evt) {
      try {
        const parsed = JSON.parse(evt.data);
        if (parsed.op === 'call_service' && parsed.service === serviceName) {
          logger.info(`[serviceServer] RAW call_service on '${serviceName}' id=${parsed.id}`);
        }
      } catch (_) {}
      return _origOnMessage.call(this, evt);
    };
  }

  service.advertise(function (request, response) {
    logger.info(`[serviceServer] REQUEST on '${serviceName}': ${JSON.stringify(request)}`);
    try {
      callback(request, response);
    } catch (err) {
      logger.error(`[serviceServer] callback threw on '${serviceName}': ${err.message}`);
      Object.assign(response, { success: false, msg: err.message });
    }
    logger.info(`[serviceServer] RESPONSE on '${serviceName}': ${JSON.stringify(response)}`);
    return true;
  });
  return service;
}

// Advertise the GCS mission services. The service definitions (including
// the business callbacks) are supplied by the facade — this layer only owns
// the ROSLIB advertise/registry lifecycle, not what the callbacks do.
export function GCSServicesMission(gcs_services, ros) {
  for (let srv of gcs_services) {
    // Unadvertise stale instance before re-advertising to avoid
    // rosbridge routing requests to a dead handler on reconnect.
    if (service_list[srv.name]) {
      try {
        service_list[srv.name].unadvertise();
      } catch (_) {}
      delete service_list[srv.name];
    }
    service_list[srv.name] = serviceServer(
      {
        serviceName: srv.serviceName,
        serviceType: srv.serviceType,
        callback: srv.callback,
      },
      ros
    );
  }
}

// Unadvertise all GCS ROS services and clear the registry.
// Called on graceful shutdown (SIGINT/SIGTERM) and uncaughtException
// to avoid leaving orphaned service advertisements on the ROS master.
export function GCSunServicesMission() {
  for (const [name, service] of Object.entries(service_list)) {
    service.unadvertise();
    delete service_list[name];
  }
}

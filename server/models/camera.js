import { logger } from '../common/logger.js';
import { VideoUtils } from '../common/videoUtils.js';
import { positionsController } from '../controllers/positions.js';

const apiURL = 'http://localhost:9997/v3/config/paths';

// A camera on this machine (the simulator) is served on 8553, a real one on 8554.
const rtspPort = (host) => (host === '127.0.0.1' ? 8553 : 8554);

export class cameraModel {
  /**
   * Obtiene una captura de imagen (snapshot) del dron.
   * Centraliza la lógica de decidir si usar ROS (caché) o FFmpeg (RTSP).
   */
  static async getSnapshot(device) {
    if (!device) return null;

    // 1. Prioridad: Caché de ROS (via positionsController)
    const cameraData = await positionsController.getCamera();
    const deviceCamera = cameraData[device.id];

    if (deviceCamera && deviceCamera.camera) {
      return {
        buffer: deviceCamera.camera,
        base64: deviceCamera.camera.toString('base64'),
        mimeType: 'image/jpeg',
      };
    }

    // 2. Alternativa: Captura vía FFmpeg desde MediaMTX/RTSP
    if (device.camera && device.camera.length > 0) {
      try {
        const cameraConfig = device.camera[0];

        let rtspUrl = null;
        if (cameraConfig.type === 'WebRTC') {
          // MediaMTX path format
          rtspUrl = `rtsp://localhost:8554/${device.name}_${cameraConfig.source}`;
        } else if (cameraConfig.host) {
          rtspUrl = `rtsp://${cameraConfig.host}:${rtspPort(cameraConfig.host)}/${cameraConfig.source}`;
        }
        // A Websocket camera has no stream to grab: its frames only come through the GCS cache above.
        if (!rtspUrl) return null;

        const imgBuffer = await VideoUtils.captureFrame(rtspUrl);
        if (imgBuffer) {
          return {
            buffer: imgBuffer,
            base64: imgBuffer.toString('base64'),
            mimeType: 'image/jpeg',
          };
        }
      } catch (error) {
        logger.error(`Error en captura FFmpeg para ${device.name}: ${error.message}`);
      }
    }

    return null;
  }

  static async addCameraWebRTC(device) {
    logger.info(`Adding WebRTC cameras for device ${device.name}`);
    // Device.camera is a nullable JSON column: a device without cameras has none to add.
    const cameras = device.camera ?? [];
    for (let i = 0; i < cameras.length; i = i + 1) {
      const rtspUrl = `rtsp://${cameras[i].host}:${rtspPort(cameras[i].host)}/${cameras[i].source}`;
      logger.info(`Camera stream: ${rtspUrl}`);
      if (cameras[i]['type'] == 'WebRTC') {
        try {
          let response = await fetch(
            `http://localhost:9997/v3/config/paths/add/${device.name}_${cameras[i].source}`,
            {
              method: 'POST',
              body: JSON.stringify({
                source: rtspUrl,
              }),
              headers: {
                'Content-Type': 'application/json',
              },
            }
          );
          if (response.status == 200) {
            logger.info(`Camera added: ${cameras[i].source}`);
          } else {
            logger.error(`Error adding camera ${cameras[i].source}: HTTP ${response.status}`);
            return false;
          }
        } catch (e) {
          logger.error(`Error adding camera ${cameras[i].source}: ${e.message}`);
          return false;
        }
      }
    }
  }

  static async removeCameraWebRTC(device) {
    if (device.camera) {
      for (let i = 0; i < device.camera.length; i = i + 1) {
        if (device.camera[i]['type'] == 'WebRTC') {
          try {
            await fetch(`${apiURL}/remove/${device.name}_${device.camera[i].source}`, {
              method: 'POST',
            });
          } catch (e) {
            logger.error(`Error removing camera ${device.camera[i].source}: ${e.message}`);
            return false;
          }
        }
      }
    }
  }
}

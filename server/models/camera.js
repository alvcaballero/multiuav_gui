import { logger } from '../common/logger.js';
import { VideoUtils } from '../common/videoUtils.js';
import { positionsController } from '../controllers/positions.js';

const apiURL = 'http://localhost:9997/v3/config/paths';

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
        const devicePort = device.ip === '127.0.0.1' ? 8553 : 8554;

        let rtspUrl = `rtsp://${device.ip}:${devicePort}/${cameraConfig.source}`;
        if (cameraConfig.type === 'WebRTC') {
          // MediaMTX path format
          rtspUrl = `rtsp://localhost:8554/${device.name}_${cameraConfig.source}`;
        }

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
    const devicePort = device.ip === '127.0.0.1' ? 8553 : 8554;
    for (let i = 0; i < device.camera.length; i = i + 1) {
      logger.info(`Device IP: rtsp://${device.ip}:${devicePort}/${device.camera[i].source}`);
      if (device.camera[i]['type'] == 'WebRTC') {
        try {
          let response = await fetch(
            `http://localhost:9997/v3/config/paths/add/${device.name}_${device.camera[i].source}`,
            {
              method: 'POST',
              body: JSON.stringify({
                source: `rtsp://${device.ip}:${devicePort}/${device.camera[i].source}`,
              }),
              headers: {
                'Content-Type': 'application/json',
              },
            }
          );
          if (response.status == 200) {
            logger.info(`Camera added: ${device.camera[i].source}`);
          } else {
            logger.error(`Error adding camera ${device.camera[i].source}: HTTP ${response.status}`);
            return false;
          }
        } catch (e) {
          logger.error(`Error adding camera ${device.camera[i].source}: ${e.message}`);
          return false;
        }
      }
    }
  }

  static async removeCameraWebRTC(device) {
    if (device.hasOwnProperty('camera')) {
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

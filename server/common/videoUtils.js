import { spawn } from 'child_process';
import logger from './logger.js';

export class VideoUtils {
  static async captureFrame(rtspUrl) {
    logger.info(`Capturing frame from RTSP: ${rtspUrl}`);

    return new Promise((resolve) => {
      const args = [
        '-rtsp_transport', 'tcp',
        '-i', rtspUrl,
        '-vf', "select='eq(pict_type,I)',scale='min(1280,iw)':-2",
        '-frames:v', '1',
        '-q:v', '3',
        '-update', '1',
        '-f', 'image2',
        'pipe:1',
      ];

      const ffmpeg = spawn('ffmpeg', args);
      const chunks = [];

      ffmpeg.stdout.on('data', (chunk) => chunks.push(chunk));

      ffmpeg.on('close', (code) => {
        if (chunks.length === 0) {
          logger.error(`ffmpeg exited with code ${code} and no output for ${rtspUrl}`);
          return resolve(null);
        }
        resolve(Buffer.concat(chunks));
      });

      ffmpeg.on('error', (err) => {
        logger.error(`ffmpeg spawn error for ${rtspUrl}: ${err.message}`);
        resolve(null);
      });
    });
  }
}

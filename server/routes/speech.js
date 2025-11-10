import { Router } from 'express';
import multer from 'multer';
import path from 'path';
import os from 'os';
import fs from 'fs';
import { SpeechController } from '../controllers/speech.js';

export const speechRouter = Router();

// Crear directorio temporal si no existe
const tmpDir = path.join(os.tmpdir(), 'multiuav-audio');
if (!fs.existsSync(tmpDir)) {
  fs.mkdirSync(tmpDir, { recursive: true });
}

// Configurar multer para guardar archivos temporalmente con extensión
const storage = multer.diskStorage({
  destination: (req, file, cb) => {
    cb(null, tmpDir);
  },
  filename: (req, file, cb) => {
    // Generar nombre único con extensión correcta
    const uniqueSuffix = Date.now() + '-' + Math.round(Math.random() * 1e9);
    // Determinar extensión basada en el mimetype
    let ext = '.webm';
    if (file.mimetype === 'audio/wav') ext = '.wav';
    else if (file.mimetype === 'audio/mp3' || file.mimetype === 'audio/mpeg') ext = '.mp3';
    else if (file.mimetype === 'audio/mp4') ext = '.mp4';
    else if (file.mimetype === 'audio/ogg') ext = '.ogg';
    else if (file.mimetype === 'audio/flac') ext = '.flac';
    else if (file.mimetype === 'audio/webm') ext = '.webm';

    cb(null, 'audio-' + uniqueSuffix + ext);
  },
});

const upload = multer({
  storage: storage,
  limits: {
    fileSize: 25 * 1024 * 1024, // Límite de 25MB (límite de OpenAI Whisper)
  },
  fileFilter: (req, file, cb) => {
    // Aceptar archivos de audio comunes
    const allowedMimeTypes = [
      'audio/webm',
      'audio/wav',
      'audio/mp3',
      'audio/mpeg',
      'audio/mp4',
      'audio/ogg',
      'audio/flac',
    ];

    if (allowedMimeTypes.includes(file.mimetype)) {
      cb(null, true);
    } else {
      cb(new Error('Tipo de archivo no soportado. Use audio/webm, wav, mp3, etc.'));
    }
  },
});

// Ruta para speech-to-text (transcripción)
speechRouter.post('/stt', upload.single('audio'), SpeechController.speechToText);

// Ruta para text-to-speech (síntesis de voz)
speechRouter.post('/tts', SpeechController.textToSpeech);

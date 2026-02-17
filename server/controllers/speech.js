import OpenAI from 'openai';
import { LLMApiKeys } from '../config/config.js';
import logger from '../common/logger.js';
import fs from 'fs';
import path from 'path';
import os from 'os';

let openaiClient;

// Inicializar cliente OpenAI solo si hay API key de OpenAI (speech usa siempre OpenAI Whisper/TTS)
if (LLMApiKeys.openai) {
  openaiClient = new OpenAI({ apiKey: LLMApiKeys.openai });
}

export class SpeechController {
  static async speechToText(req, res) {
    try {
      // Verificar si se recibió un archivo de audio
      if (!req.file) {
        return res.status(400).json({ error: 'No se recibió archivo de audio.' });
      }

      // Verificar si OpenAI está configurado
      if (!openaiClient) {
        logger.error('OpenAI no está configurado para speech-to-text');
        return res.status(500).json({
          error: 'El servicio de transcripción de audio no está disponible. Configure OpenAI como proveedor LLM.'
        });
      }

      logger.info('Procesando audio para transcripción', {
        filename: req.file.originalname,
        size: req.file.size,
        mimetype: req.file.mimetype,
      });

      // El archivo ya está guardado temporalmente por multer
      const audioPath = req.file.path;

      try {
        // Usar la API de Whisper de OpenAI para transcribir el audio
        const transcription = await openaiClient.audio.transcriptions.create({
          file: fs.createReadStream(audioPath),
          model: 'whisper-1',
          language: 'es', // Especificar español para mejor precisión
          prompt: 'Este es un mensaje de voz en español para un sistema de control de drones.', // Ayuda a reducir alucinaciones
        });

        // Lista de frases comunes que Whisper "alucina" cuando el audio es de baja calidad
        const hallucinationPatterns = [
          'subtítulos realizados por la comunidad de amara.org',
          'subtítulos realizados por la comunidad de amara',
          'gracias por ver',
          'suscríbete',
          'like y suscríbete',
          'dale like',
          'no olvides suscribirte',
          'Este es un mensaje de voz',
        ];

        let text = transcription.text.trim();

        // Verificar si el texto es una alucinación conocida (case insensitive)
        const isHallucination = hallucinationPatterns.some(pattern =>
          text.toLowerCase().includes(pattern)
        );

        // Si es una alucinación o el texto está vacío, devolver error
        if (isHallucination || !text) {
          logger.warn('Audio detectado como alucinación o vacío', {
            originalText: text,
            fileSize: req.file.size,
          });

          // Eliminar el archivo temporal
          fs.unlinkSync(audioPath);

          return res.status(400).json({
            error: 'No se detectó voz clara en el audio. Por favor, habla más cerca del micrófono o repite tu mensaje.',
            hallucination: true
          });
        }

        logger.info('Audio transcrito exitosamente', {
          text: text,
        });

        // Eliminar el archivo temporal
        fs.unlinkSync(audioPath);

        res.json({ text: text });
      } catch (error) {
        // Eliminar el archivo temporal en caso de error
        if (fs.existsSync(audioPath)) {
          fs.unlinkSync(audioPath);
        }
        throw error;
      }
    } catch (error) {
      logger.error('Error en speech-to-text', {
        error: error.message,
        stack: error.stack,
      });
      res.status(500).json({
        error: 'Error al procesar el audio: ' + error.message
      });
    }
  }

  static async textToSpeech(req, res) {
    try {
      const { text } = req.body;

      // Verificar que se recibió el texto
      if (!text) {
        return res.status(400).json({ error: 'No se recibió texto para convertir.' });
      }

      // Verificar si OpenAI está configurado
      if (!openaiClient) {
        logger.error('OpenAI no está configurado para text-to-speech');
        return res.status(500).json({
          error: 'El servicio de texto a voz no está disponible. Configure OpenAI como proveedor LLM.'
        });
      }

      logger.info('Generando audio desde texto', {
        textLength: text.length,
      });

      try {
        // Usar la API de TTS de OpenAI para generar audio
        const mp3Response = await openaiClient.audio.speech.create({
          model: 'tts-1',
          voice: 'nova', // Voz femenina en español (opciones: alloy, echo, fable, onyx, nova, shimmer)
          input: text,
          speed: 1.0,
        });

        // Convertir la respuesta a buffer
        const buffer = Buffer.from(await mp3Response.arrayBuffer());

        logger.info('Audio generado exitosamente', {
          size: buffer.length,
        });

        // Enviar el audio como respuesta
        res.set({
          'Content-Type': 'audio/mpeg',
          'Content-Length': buffer.length,
        });
        res.send(buffer);
      } catch (error) {
        throw error;
      }
    } catch (error) {
      logger.error('Error en text-to-speech', {
        error: error.message,
        stack: error.stack,
      });
      res.status(500).json({
        error: 'Error al generar audio: ' + error.message
      });
    }
  }
}

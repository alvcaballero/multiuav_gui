import cors from 'cors';
import { CorsEnable } from '../config/config.js';

const ACCEPTED_ORIGINS = [
  'http://localhost:3000',
  'http://localhost:4000',
  'http://localhost:5052',
  'https://mydomain.com',
  'https://mydomain.dev',
];

// Prefijos de subredes locales permitidas
const ACCEPTED_ORIGIN_PREFIXES = ['http://10.42.0.', 'http://192.168.1.'];

export const corsMiddleware = ({ acceptedOrigins = ACCEPTED_ORIGINS } = {}) =>
  cors({
    origin: (origin, callback) => {
      // CORS_ENABLE=false → desarrollo local sin restricciones
      if (!CorsEnable) {
        return callback(null, true);
      }

      // Sin origin: requests server-to-server o herramientas (curl, Postman).
      // Solo se permiten en desarrollo (CorsEnable=false ya cubierto arriba).
      if (!origin) {
        return callback(new Error('Not allowed by CORS'));
      }

      // Origin en lista explícita
      if (acceptedOrigins.includes(origin)) {
        return callback(null, true);
      }

      // Origin en subred local permitida
      if (ACCEPTED_ORIGIN_PREFIXES.some((prefix) => origin.startsWith(prefix))) {
        return callback(null, true);
      }

      return callback(new Error('Not allowed by CORS'));
    },
  });

import { OnDeviceTraining } from '@mui/icons-material';
import cors from 'cors';

const ACCEPTED_ORIGINS = [
  'http://localhost:3000',
  'http://localhost:4000',
  'https://mydomain.com',
  'https://mydomain.dev',
];

export const corsMiddleware = ({ acceptedOrigins = ACCEPTED_ORIGINS } = {}) =>
  cors({
    origin: (origin, callback) => {
      if (acceptedOrigins.includes(origin)) {
        return callback(null, true);
      }
      if (origin) {
        console.log(origin);
        if (origin.includes('http://192.168.')) {
          return callback(null, true);
        }
        if (origin.includes('http://10.42.0.')) {
          return callback(null, true);
        }
        if (origin.includes('http://10.10.2.')) {
          return callback(null, true);
        }
      }

      if (!origin) {
        return callback(null, true);
      }

      return callback(new Error('Not allowed by CORS'));
    },
  });

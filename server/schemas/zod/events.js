import { z } from 'zod';

export const EVENT_TYPE = z.enum([
  'no',
  'takeoff',
  'landing',
  'emergency',
  'battery_low',
  'connection_lost',
  'mission_start',
  'mission_end',
  'error'
]);

export const EventSchema = z.object({
  id: z.number().optional(),
  type: EVENT_TYPE.default('no'),
  eventTime: z.date().optional(),
  deviceId: z.number().nullable(),
  missionId: z.number().nullable(),
  positionId: z.array(z.number()).default([0, 0, 0]),
  attributes: z.record(z.any()).default({}),
  createdAt: z.date().optional()
});

export const EventCreateSchema = EventSchema.omit({ 
  id: true,
  createdAt: true 
}); 
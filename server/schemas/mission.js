import { z } from 'zod';

export const MISSION_STATUS = z.enum([
  'init',
  'planning',
  'running',
  'finish',
  'done',
  'cancelled',
  'error'
]);

export const ROUTE_STATUS = z.enum([
  'init',
  'loaded',
  'commanded',
  'running',
  'complete',
  'end',
  'cancelled',
  'error'
]);

export const MissionSchema = z.object({
  id: z.string().optional(),
  name: z.string(),
  uav: z.array(z.number()),
  status: MISSION_STATUS.default('init'),
  initTime: z.date().default(() => new Date()),
  endTime: z.date().nullable(),
  task: z.record(z.any()).default({}),
  mission: z.record(z.any()).default({}),
  results: z.array(z.record(z.any())).default([])
});

export const RouteSchema = z.object({
  id: z.string().optional(),
  deviceId: z.string(),
  missionId: z.string(),
  status: ROUTE_STATUS.default('init'),
  initTime: z.date().default(() => new Date()),
  endTime: z.date().nullable(),
  task: z.record(z.any()).default({}),
  mission: z.record(z.any()).default({}),
  results: z.array(z.record(z.any())).default([])
});

export const TaskSchema = z.object({
  id: z.string(),
  name: z.string().optional(),
  objetivo: z.string(),
  locations: z.array(z.record(z.any())),
  meteo: z.record(z.any()).optional()
}); 
import { z } from 'zod';
import { MISSION_STATUS as MISSION_STATUS_MAP, TASK_STATUS as TASK_STATUS_MAP } from '../../config/status.js';

export const MISSION_STATUS = z.enum(Object.values(MISSION_STATUS_MAP));
export const TASK_STATUS = z.enum(Object.values(TASK_STATUS_MAP));

export const MissionSchema = z.object({
  id: z.string().optional(),
  name: z.string(),
  uav: z.array(z.number()),
  status: MISSION_STATUS.default('init'),
  initTime: z.date().default(() => new Date()),
  endTime: z.date().nullable(),
  request: z.record(z.any()).default({}),
  mission: z.record(z.any()).default({}),
  results: z.array(z.record(z.any())).default([]),
});

export const MissionTaskSchema = z.object({
  id: z.string().optional(),
  deviceId: z.string(),
  missionId: z.string(),
  taskKey: z.string(),
  dependsOn: z.array(z.string()).default([]),
  action: z.string(),
  status: TASK_STATUS.default('init'),
  initTime: z.date().default(() => new Date()),
  endTime: z.date().nullable(),
  result: z.record(z.any()).default({}),
});

export const MissionRequestSchema = z.object({
  id: z.string(),
  name: z.string().optional(),
  objetivo: z.string(),
  locations: z.array(z.record(z.any())),
  meteo: z.record(z.any()).optional(),
});

import { z } from 'zod';

export const UAV_STATE = z.enum(['OK', 'WARNING', 'ERROR', 'CRITICAL']);
export const LANDED_STATE = z.enum(['UNDEFINED', 'ON GROUND', 'IN AIR', 'TAKEOFF', 'LANDING', 'STOPED']);
export const MISSION_STATE = z.enum(['Ready', 'Running', 'Paused', 'Completed', 'Error']);
export const ALARM_STATE = z.enum(['None', 'threat', 'confirm', 'UNDEFINED']);

export const GimbalSchema = z.object({
  x: z.number(),
  y: z.number(),
  z: z.number()
});

export const ObstacleInfoSchema = z.object({
  down: z.number(),
  front: z.number(),
  right: z.number(),
  back: z.number(),
  left: z.number(),
  up: z.number()
});

export const PositionAttributesSchema = z.object({
  batteryLevel: z.number().default(0),
  gimbal: z.array(z.number()).default([0, 0, 0]),
  obstacle_info: z.array(z.number()).default([0, 0, 0, 0, 0, 0]),
  takeoff_height: z.number().default(400),
  mission_state: MISSION_STATE.default('Ready'),
  wp_reached: z.number().default(0),
  uav_state: UAV_STATE.default('OK'),
  landed_state: LANDED_STATE.default('UNDEFINED'),
  alarm: ALARM_STATE.default('UNDEFINED'),
  home: z.array(z.number()).optional(),
  MIC_1: z.number().optional(),
  MIC_2: z.number().optional(),
  MIC_3: z.number().optional(),
  Metano: z.number().optional(),
  Alcohol: z.number().optional(),
  CO: z.number().optional()
});

export const PositionSchema = z.object({
  deviceId: z.number(),
  latitude: z.number().optional(),
  longitude: z.number().optional(),
  altitude: z.number().optional(),
  accuracy: z.number().default(0.0),
  speed: z.number().default(0.0),
  course: z.number().default(0.0),
  deviceTime: z.date().optional(),
  attributes: PositionAttributesSchema
});

export const CameraSchema = z.object({
  deviceId: z.number(),
  type: z.string(),
  source: z.string(),
  resolution: z.object({
    width: z.number(),
    height: z.number()
  }).optional(),
  fps: z.number().optional()
}); 
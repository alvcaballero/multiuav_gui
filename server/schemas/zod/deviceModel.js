import { z } from 'zod';
import { FIRMWARE, VEHICLE_TYPE } from '../../config/status.js';

// Shape of ONE device model file (config/devices/device_models/<key>.yaml): WHAT a vehicle is, apart
// from how the GCS talks to it (its interface). Strict like the interfaces: a typo is an error.

const key = z.string().regex(/^[A-Za-z0-9_]+$/);
const positive = z.number().positive();

// Field names are Smart-Flies-MUAVPlanner's (files/devices.yaml, modules/uav.py): it is the planner
// that consumes them (energy model of a rotorcraft).
const physics = z
  .object({
    mass: positive.optional(), // kg
    number_of_rotors: z.number().int().positive().optional(),
    rotor_blades: z.number().int().positive().optional(),
    rotor_radius: positive.optional(), // m
    blade_chord: positive.optional(), // m, at 0.7 R
    lift_coefficient: z.number().optional(),
    draft_coefficient: z.number().optional(), // (sic) drag coefficient, Smart-Flies' name
    induced_power_factor: z.number().optional(),
    energy_efficiency: z.number().optional(),
    P0_numerical_constant: z.number().optional(),
    equivalent_flat_plate_area: positive.optional(), // m²
  })
  .strict();

const battery = z
  .object({
    number: z.number().int().positive().optional(),
    capacity: positive.optional(), // mAh
    number_of_cells: z.number().int().positive().optional(),
    voltage_per_cell: positive.optional(), // V
  })
  .strict();

const speeds = z.object({ mission: positive.optional(), navigation: positive.optional() }).strict(); // m/s

// What a planner must respect; the GCS publishes them, it does not check the flight dynamics.
const limits = z.object({ min_turn_radius: positive.optional() /* m */ }).strict();

// Range of each obstacle sensor, [min, max] in metres.
const range = z.tuple([z.number().min(0), z.number().positive()]);
const obstacleSensor = z.record(z.enum(['down', 'front', 'right', 'back', 'left', 'up']), range);

export const deviceModelSchema = z
  .object({
    extends: key.optional(),
    abstract: z.boolean().optional(),
    vehicleType: z.enum(Object.values(VEHICLE_TYPE)).optional(),
    // The firmwares this airframe runs. Not declared = not known: compatible with any interface of
    // its vehicle type (the generic models, and a vehicle whose autopilot nobody wrote down).
    firmware: z
      .array(z.enum(Object.values(FIRMWARE)))
      .min(1)
      .refine((list) => new Set(list).size === list.length, 'repeated firmware')
      .optional(),
    // The interfaces (categories) a device of this model can use, written out: the device form offers
    // exactly these. Each one is checked at boot against the compatibility rule (vehicle type +
    // firmware), so a hand-written list cannot name one that cannot drive it. [] = none yet.
    interfaces: z
      .array(key)
      .refine((list) => new Set(list).size === list.length, 'repeated interface')
      .optional(),
    label: z.string().min(1).optional(),
    icon: key.optional(), // client map icon (preloadImages.js mapIcons)
    model3d: key.optional(), // client 3D model (ModelLoader.jsx modelPaths)
    physics: physics.optional(),
    battery: battery.optional(),
    speeds: speeds.optional(),
    limits: limits.optional(),
    obstacle_sensor: obstacleSensor.optional(),
  })
  .strict();

export function deviceModelShapeProblems(def) {
  const result = deviceModelSchema.safeParse(def ?? {});
  if (result.success) return [];
  return result.error.issues.map(({ path, message }) => ({ path, message }));
}

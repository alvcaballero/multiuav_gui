import z from 'zod';

const deviceSchema = z.object({
  name: z.string({
    invalid_type_error: 'device name must be a string',
    required_error: 'device name is required.',
  }),
  category: z.string(z.enum(['dji_M210_melodic', 'dji_M210_noetic', 'dji_M300', 'fuvex', 'catec']), {
    required_error: 'device category is required.',
    invalid_type_error: ' device category must be an array of enum Genre',
  }),
  ip: z.string().ip({ version: 'v4', message: 'Invalid IP address' }),
  camera: z.array(z.object({ type: z.string(), source: z.string() })),
  files: z.array(z.object({ url: z.string(), type: z.string() })),
  protocol: z.string(z.enum(['ros', 'robofleet']), {
    required_error: 'device protocol is required.',
    invalid_type_error: ' device protocol must be an array of enum Genre',
  }),
});

export function validateDevice(input) {
  console.log('check validate device');
  console.log(input);
  return deviceSchema.safeParse(input);
}

export function validatePartialDevice(input) {
  return deviceSchema.partial().safeParse(input);
}

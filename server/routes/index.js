import { createDevicesRouter } from './devices.js';
import { categoryRouter } from './category.js';
import { positionsRouter } from './positions.js';
import { eventsRouter } from './events.js';
import { commandsRouter } from './commands.js';
import { mapRouter } from './map.js';
import { createMissionRouter } from './mission.js';
import { createFilesRouter } from './files.js';
import { ExtAppRouter } from './ExtApp.js';
import { serverRouter } from './server.js';
import { planningRouter } from './planning.js';
import { geofenceRouter } from './geofence.js';
import { llmRouter } from './llm.js';
import { rosRouter } from './ros.js';
import { speechRouter } from './speech.js';

export function setupRoutes(app) {
  app.use('/api/devices', createDevicesRouter());
  app.use('/api/category', categoryRouter);
  app.use('/api/positions', positionsRouter);
  app.use('/api/events', eventsRouter);
  app.use('/api/commands', commandsRouter);
  app.use('/api/map', mapRouter);
  app.use('/api/missions', createMissionRouter());
  app.use('/api/files', createFilesRouter());
  app.use('/api/planning', planningRouter);
  app.use('/api/ExtApp', ExtAppRouter);
  app.use('/api/server', serverRouter);
  app.use('/api/geofences', geofenceRouter);
  app.use('/api/chat', llmRouter);
  app.use('/api/ros', rosRouter);
  app.use('/api/speech-tools', speechRouter);
}

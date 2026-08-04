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
import { markersRouter } from './markers/markers.js';
import { geofenceRouter } from './geofence.js';
import { chatRouter } from './chat.js';
import { rosRouter } from './ros.js';
import { elementTypesRouter } from './markers/elementTypes.js';
import { elementGroupsRouter } from './markers/elementGroups.js';
import { elementItemsRouter } from './markers/elementItems.js';
import { basesRouter } from './markers/bases.js';
import { assignmentsRouter } from './markers/assignments.js';

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
  app.use('/api/ros', rosRouter);
  app.use('/api/chat', chatRouter);
  app.use('/api/markers/types', elementTypesRouter);
  app.use('/api/markers/groups', elementGroupsRouter);
  app.use('/api/markers/items', elementItemsRouter);
  app.use('/api/markers/bases', basesRouter);
  app.use('/api/markers/assignments', assignmentsRouter);
  // Root instances ({markersbase, elements}) — consumed by get_registered_objects
  // in mcp_server. Mounted last so it never shadows the more specific
  // /api/markers/{types,groups,items,bases,assignments} prefixes above.
  app.use('/api/markers', markersRouter);
}

import { Router } from 'express';
import { geofenceController } from '../controllers/geofence.js';

export const geofenceRouter = Router();

// Obtener todas las geocercas
geofenceRouter.get('/', geofenceController.getAll);

// Obtener una geocerca por ID
geofenceRouter.get('/:id', geofenceController.getById);

// Crear una nueva geocerca
geofenceRouter.post('/', geofenceController.create);

// Actualizar una geocerca existente
geofenceRouter.put('/:id', geofenceController.update);

// Eliminar una geocerca
geofenceRouter.delete('/:id', geofenceController.delete);

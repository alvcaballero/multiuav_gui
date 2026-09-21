import '@mapbox/mapbox-gl-draw/dist/mapbox-gl-draw.css';
import * as maplibregl from 'maplibre-gl';
import MapboxDraw from '@mapbox/mapbox-gl-draw';
import { useEffect, useMemo } from 'react';

import { useDispatch, useSelector } from 'react-redux';
import { useNavigate } from 'react-router-dom';
import { useTheme } from '@mui/material/styles';
import { map } from '../core/mapInstance';
import { findFonts, geofenceToFeature, geometryToArea } from '../core/mapUtil';
import { errorsActions, geofencesActions } from '../../store';
import { useCatchCallback } from '../../reactHelper';
import drawTheme from './theme';

// mapbox-gl-draw looks its DOM up by Mapbox's class names; these remap them onto
// MapLibre's. CANVAS matters most - draw resolves the drawing surface through it.
// Same set the official MapLibre v6 + draw example applies.
MapboxDraw.constants.classes.CANVAS = 'maplibregl-canvas';
MapboxDraw.constants.classes.ATTRIBUTION = 'maplibregl-ctrl-attrib';
MapboxDraw.constants.classes.CONTROL_BASE = 'maplibregl-ctrl';
MapboxDraw.constants.classes.CONTROL_PREFIX = 'maplibregl-ctrl-';
MapboxDraw.constants.classes.CONTROL_GROUP = 'maplibregl-ctrl-group';

const MapGeofenceEdit = ({ selectedGeofenceId }) => {
  const theme = useTheme();
  const dispatch = useDispatch();
  const navigate = useNavigate();

  const draw = useMemo(
    () =>
      new MapboxDraw({
        displayControlsDefault: false,
        controls: {
          polygon: true,
          line_string: true,
          trash: true,
        },
        userProperties: true,
        styles: [
          ...drawTheme,
          {
            id: 'gl-draw-title',
            type: 'symbol',
            filter: ['all'],
            layout: {
              'text-field': '{user_name}',
              'text-font': findFonts(map),
              'text-size': 12,
            },
            paint: {
              'text-halo-color': 'white',
              'text-halo-width': 1,
            },
          },
        ],
      }),
    [],
  );

  const geofences = useSelector((state) => state.geofences.items);

  const refreshGeofences = useCatchCallback(async () => {
    const response = await fetch('/api/geofences');
    if (response.ok) {
      dispatch(geofencesActions.refresh(await response.json()));
    } else {
      throw Error(await response.text());
    }
  }, [dispatch]);

  useEffect(() => {
    refreshGeofences();

    map.addControl(draw, 'top-left');
    return () => map.removeControl(draw);
  }, [refreshGeofences, draw]);

  useEffect(() => {
    const listener = async (event) => {
      const feature = event.features[0];
      const newItem = { name: 'sharedGeofence', area: geometryToArea(feature.geometry) };
      draw.delete(feature.id);
      try {
        const response = await fetch('/api/geofences', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(newItem),
        });
        if (response.ok) {
          const item = await response.json();
          navigate(`/settings/geofence/${item.id}`);
        } else {
          throw Error(await response.text());
        }
      } catch (error) {
        dispatch(errorsActions.push(error.message));
      }
    };

    map.on('draw.create', listener);
    return () => map.off('draw.create', listener);
  }, [dispatch, navigate, draw]);

  useEffect(() => {
    const listener = async (event) => {
      const feature = event.features[0];
      try {
        const response = await fetch(`/api/geofences/${feature.id}`, { method: 'DELETE' });
        if (response.ok) {
          refreshGeofences();
        } else {
          throw Error(await response.text());
        }
      } catch (error) {
        dispatch(errorsActions.push(error.message));
      }
    };

    map.on('draw.delete', listener);
    return () => map.off('draw.delete', listener);
  }, [dispatch, refreshGeofences]);

  useEffect(() => {
    const listener = async (event) => {
      const feature = event.features[0];
      const item = Object.values(geofences).find((i) => i.id === feature.id);
      if (item) {
        const updatedItem = { ...item, area: geometryToArea(feature.geometry) };
        try {
          const response = await fetch(`/api/geofences/${feature.id}`, {
            method: 'PUT',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(updatedItem),
          });
          if (response.ok) {
            refreshGeofences();
          } else {
            throw Error(await response.text());
          }
        } catch (error) {
          dispatch(errorsActions.push(error.message));
        }
      }
    };

    map.on('draw.update', listener);
    return () => map.off('draw.update', listener);
  }, [dispatch, geofences, refreshGeofences]);

  useEffect(() => {
    const loadIntoDraw = () => {
      draw.deleteAll();
      Object.values(geofences).forEach((geofence) => {
        draw.add(geofenceToFeature(theme, geofence));
      });
    };

    loadIntoDraw();

    // Under maplibre-gl v6 the features draw pushes before the style has settled are
    // accepted by the source and then never turned into tiles: draw holds them, the
    // source reports them, and nothing is ever drawn. Re-applying once the map goes
    // idle re-sends the same data through a source that is ready to parse it.
    if (map.loaded() && map.isStyleLoaded()) return undefined;
    map.once('idle', loadIntoDraw);
    return () => map.off('idle', loadIntoDraw);
  }, [geofences, draw, theme]);

  useEffect(() => {
    if (selectedGeofenceId) {
      const feature = draw.get(selectedGeofenceId);
      let { coordinates } = feature.geometry;
      if (Array.isArray(coordinates[0][0])) {
        [coordinates] = coordinates;
      }
      const bounds = coordinates.reduce(
        (bounds, coordinate) => bounds.extend(coordinate),
        new maplibregl.LngLatBounds(coordinates[0], coordinates[1]),
      );
      const canvas = map.getCanvas();
      map.fitBounds(bounds, { padding: Math.min(canvas.width, canvas.height) * 0.1 });
    }
  }, [selectedGeofenceId, draw]);

  return null;
};

export default MapGeofenceEdit;

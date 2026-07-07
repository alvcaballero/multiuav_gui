import 'maplibre-gl/dist/maplibre-gl.css';
import maplibregl from 'maplibre-gl';

import { MaplibreExportControl, Size, PageOrientation, Format } from '@watergis/maplibre-gl-export';
import '@watergis/maplibre-gl-export/dist/maplibre-gl-export.css';

import { SwitcherControl } from '../switcher/switcher';
import { savePersistedState } from '../../shared/usePersistedState';

import { mapImages, imagesReady } from './preloadImages';

const element = document.createElement('div');
element.style.width = '100%';
element.style.height = '100%';
element.style.boxSizing = 'initial';

export const map = new maplibregl.Map({
  container: element,
  attributionControl: false,
  zoom: 14,
  centerClampedToGround: false,
  maxPitch: 85,
  canvasContextAttributes: { antialias: true }, // create the gl context with MSAA antialiasing, so custom layers are antialiased
});

export { element };

let ready = false;
const readyListeners = new Set();

export const addReadyListener = (listener) => {
  readyListeners.add(listener);
  listener(ready);
};

export const removeReadyListener = (listener) => {
  readyListeners.delete(listener);
};

const updateReadyValue = (value) => {
  ready = value;
  readyListeners.forEach((listener) => listener(value));
};

const initMap = async () => {
  if (ready) return;
  await imagesReady;
  if (!map.hasImage('background')) {
    Object.entries(mapImages).forEach(([key, value]) => {
      map.addImage(key, value, {
        pixelRatio: window.devicePixelRatio,
      });
    });
  }
  updateReadyValue(true);
};

map.on('styleimagemissing', (e) => {
  const missingId = e.id;
  if (mapImages[missingId]) {
    map.addImage(missingId, mapImages[missingId], { pixelRatio: window.devicePixelRatio });
  }
});

map.addControl(new maplibregl.NavigationControl());
map.addControl(
  new MaplibreExportControl({
    PageSize: Size.A3,
    PageOrientation: PageOrientation.Portrait,
    Format: Format.SVG,
    Crosshair: true,
    PrintableArea: true,
    Local: 'en',
  }),
  'top-right',
);

export const switcher = new SwitcherControl(
  () => updateReadyValue(false),
  (styleId) => savePersistedState('selectedMapStyle', styleId),
  () => {
    map.once('styledata', () => {
      const waiting = () => {
        if (!map.loaded()) {
          setTimeout(waiting, 33);
        } else {
          initMap();
        }
      };
      waiting();
    });
  },
);

map.addControl(switcher);

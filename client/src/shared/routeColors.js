// Single source of a route's color, for every view that draws routes (2D/3D map,
// route list, stats, elevation, KML export) so the same route has the same color
// everywhere.
//
// A route's key is its stable editor id (`route.id`, never renumbered when another
// route is deleted), or its position in the plan when it has none (a plan read
// straight from the server). Never key by the position in a *filtered* list: the
// route would change color when the filter changes.
//
// The first keys take the curated palette. Past it, hues are spaced by the golden
// angle: each new hue lands in the widest gap left by the previous ones, so
// consecutive routes never look alike and there is no upper limit on routes. The
// sequence starts in the widest gap of the curated hues (green→cyan) and alternates
// lightness, so a generated hue close to a curated one still reads as different.

const CURATED = ['#F34C28', '#F39A28', '#1EC910', '#1012C9', '#C310C9', '#1FDBF1', '#F6FD04'];

const GOLDEN_ANGLE = 137.508;
const GENERATED_HUE_START = 150;
const GENERATED_SATURATION = 0.8;
const GENERATED_LIGHTNESS = [0.4, 0.62];

// A route with no key (not part of any plan): neutral, so it never passes for a route.
export const NO_ROUTE_COLOR = '#808080';

const hslToHex = (hue, saturation, lightness) => {
  const a = saturation * Math.min(lightness, 1 - lightness);
  const channel = (n) => {
    const k = (n + hue / 30) % 12;
    const value = lightness - a * Math.max(-1, Math.min(k - 3, 9 - k, 1));
    return Math.round(value * 255)
      .toString(16)
      .padStart(2, '0');
  };
  return `#${channel(0)}${channel(8)}${channel(4)}`.toUpperCase();
};

const generated = new Map();

export const routeColor = (key) => {
  const index = Number(key);
  if (key == null || key === '' || !Number.isInteger(index) || index < 0) return NO_ROUTE_COLOR;
  if (index < CURATED.length) return CURATED[index];
  if (!generated.has(index)) {
    const step = index - CURATED.length;
    const hue = (GENERATED_HUE_START + step * GOLDEN_ANGLE) % 360;
    const lightness = GENERATED_LIGHTNESS[step % GENERATED_LIGHTNESS.length];
    generated.set(index, hslToHex(hue, GENERATED_SATURATION, lightness));
  }
  return generated.get(index);
};

export const routeColorKey = (route, index) => route?.id ?? index;

// A device takes the color of the first route assigned to it (a device can run
// several tasks of one plan). null when the device has no route.
export const deviceRouteColorKey = (routes, deviceName) => {
  const index = (routes ?? []).findIndex((route) => route.uav === deviceName);
  return index < 0 ? null : routeColorKey(routes[index], index);
};

// KML colors are aabbggrr, not rrggbb.
export const toKmlColor = (hex) => `ff${hex.slice(5, 7)}${hex.slice(3, 5)}${hex.slice(1, 3)}`;

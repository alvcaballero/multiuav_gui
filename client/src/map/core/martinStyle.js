// Background layers using low-zoom tiles (global coverage from Planetiler)
// martin_bg always overzooms from zoom 3, giving a persistent world view
const backgroundWater = {
  id: 'bg-water',
  type: 'fill',
  source: 'martin_bg',
  'source-layer': 'water',
  //maxzoom: 6, // hide before the low-res coastline becomes visible
  paint: { 'fill-color': '#a0c8f0' },
};
const backgroundLand = { id: 'background', type: 'background', paint: { 'background-color': '#f8f4f0' } };

const water = [
  { id: 'water', type: 'fill', source: 'martin', 'source-layer': 'water', paint: { 'fill-color': '#a0c8f0' } },
  {
    id: 'waterway',
    type: 'line',
    source: 'martin',
    'source-layer': 'waterway',
    paint: { 'line-color': '#a0c8f0', 'line-width': 1.5 },
  },
  {
    id: 'water-name',
    type: 'symbol',
    source: 'martin',
    'source-layer': 'water_name',
    layout: {
      'text-field': ['coalesce', ['get', 'name_en'], ['get', 'name']],
      'text-font': ['Noto Sans Italic'],
      'text-size': 12,
    },
    paint: { 'text-color': '#5588aa', 'text-halo-color': '#f8f4f0', 'text-halo-width': 1 },
  },
];

const landcover = [
  {
    id: 'landcover-wood',
    type: 'fill',
    source: 'martin',
    'source-layer': 'landcover',
    filter: ['==', 'class', 'wood'],
    paint: { 'fill-color': '#d0e8c0', 'fill-opacity': 0.8 },
  },
  {
    id: 'landcover-grass',
    type: 'fill',
    source: 'martin',
    'source-layer': 'landcover',
    filter: ['in', 'class', 'grass', 'farmland', 'farmyard'],
    paint: { 'fill-color': '#e8f0d8', 'fill-opacity': 0.8 },
  },
];

const landuse = [
  {
    id: 'landuse-residential',
    type: 'fill',
    source: 'martin',
    'source-layer': 'landuse',
    filter: ['==', 'class', 'residential'],
    paint: { 'fill-color': '#f0ece4', 'fill-opacity': 0.8 },
  },
  {
    id: 'landuse-commercial',
    type: 'fill',
    source: 'martin',
    'source-layer': 'landuse',
    filter: ['==', 'class', 'commercial'],
    paint: { 'fill-color': '#f0e8e0', 'fill-opacity': 0.8 },
  },
  {
    id: 'park',
    type: 'fill',
    source: 'martin',
    'source-layer': 'park',
    paint: { 'fill-color': '#d8ecbc', 'fill-opacity': 0.8 },
  },
];

const boundaries = [
  {
    id: 'boundary-country',
    type: 'line',
    source: 'martin',
    'source-layer': 'boundary',
    filter: ['==', 'admin_level', 2],
    paint: { 'line-color': '#8888aa', 'line-width': 1.5, 'line-dasharray': [4, 2] },
  },
];

const buildings = [
  {
    id: 'building',
    type: 'fill',
    source: 'martin',
    'source-layer': 'building',
    minzoom: 13,
    paint: { 'fill-color': '#e0d8d0', 'fill-outline-color': '#c8c0b8' },
  },
];

const roads = [
  {
    id: 'road-motorway',
    type: 'line',
    source: 'martin',
    'source-layer': 'transportation',
    filter: ['==', 'class', 'motorway'],
    paint: { 'line-color': '#fc8f3f', 'line-width': ['interpolate', ['linear'], ['zoom'], 5, 1.5, 14, 6] },
  },
  {
    id: 'road-trunk',
    type: 'line',
    source: 'martin',
    'source-layer': 'transportation',
    filter: ['==', 'class', 'trunk'],
    paint: { 'line-color': '#fdc874', 'line-width': ['interpolate', ['linear'], ['zoom'], 7, 1, 14, 4] },
  },
  {
    id: 'road-primary',
    type: 'line',
    source: 'martin',
    'source-layer': 'transportation',
    filter: ['==', 'class', 'primary'],
    paint: { 'line-color': '#fdc874', 'line-width': ['interpolate', ['linear'], ['zoom'], 8, 0.8, 14, 3] },
  },
  {
    id: 'road-secondary',
    type: 'line',
    source: 'martin',
    'source-layer': 'transportation',
    filter: ['==', 'class', 'secondary'],
    paint: { 'line-color': '#f0d080', 'line-width': ['interpolate', ['linear'], ['zoom'], 10, 0.5, 14, 2.5] },
  },
  {
    id: 'road-street',
    type: 'line',
    source: 'martin',
    'source-layer': 'transportation',
    filter: ['in', 'class', 'minor', 'service', 'tertiary'],
    paint: { 'line-color': '#ffffff', 'line-width': ['interpolate', ['linear'], ['zoom'], 11, 0.4, 14, 1.5] },
  },
  {
    id: 'road-label',
    type: 'symbol',
    source: 'martin',
    'source-layer': 'transportation_name',
    minzoom: 13,
    layout: {
      'text-field': ['get', 'name'],
      'text-font': ['Noto Sans Regular'],
      'symbol-placement': 'line',
      'text-size': 11,
    },
    paint: { 'text-color': '#444444', 'text-halo-color': '#ffffff', 'text-halo-width': 1 },
  },
];

const places = [
  {
    id: 'place-country',
    type: 'symbol',
    source: 'martin',
    'source-layer': 'place',
    filter: ['==', 'class', 'country'],
    layout: {
      'text-field': ['coalesce', ['get', 'name_en'], ['get', 'name']],
      'text-font': ['Noto Sans Bold'],
      'text-size': ['interpolate', ['linear'], ['zoom'], 2, 10, 6, 16],
    },
    paint: { 'text-color': '#333344', 'text-halo-color': '#ffffff', 'text-halo-width': 1.5 },
  },
  {
    id: 'place-city',
    type: 'symbol',
    source: 'martin',
    'source-layer': 'place',
    filter: ['in', 'class', 'city', 'town'],
    layout: {
      'text-field': ['coalesce', ['get', 'name_en'], ['get', 'name']],
      'text-font': ['Noto Sans Regular'],
      'text-size': ['interpolate', ['linear'], ['zoom'], 6, 10, 14, 16],
    },
    paint: { 'text-color': '#333344', 'text-halo-color': '#ffffff', 'text-halo-width': 1.5 },
  },
  {
    id: 'place-village',
    type: 'symbol',
    source: 'martin',
    'source-layer': 'place',
    filter: ['in', 'class', 'village', 'hamlet', 'suburb', 'neighbourhood'],
    minzoom: 10,
    layout: {
      'text-field': ['coalesce', ['get', 'name_en'], ['get', 'name']],
      'text-font': ['Noto Sans Regular'],
      'text-size': 12,
    },
    paint: { 'text-color': '#555566', 'text-halo-color': '#ffffff', 'text-halo-width': 1 },
  },
];

// TODO:  add sprite for icons, e.g. poi, transportation, etc.
// sprite: 'https://demotiles.maplibre.org/styles/osm-bright-gl-style/sprite',
export const buildMartinStyle = (glyphs, hostname) => ({
  version: 8,
  glyphs,
  sources: {
    martin_bg: {
      type: 'vector',
      url: `http://${hostname}:8080/tiles`,
      maxzoom: 0, // always uses zoom ≤3 tiles → global water/boundary coverage
    },
    martin: {
      type: 'vector',
      url: `http://${hostname}:8080/tiles`,
      maxzoom: 14,
    },
  },
  layers: [
    backgroundLand,
    backgroundWater,
    ...water,
    ...landcover,
    ...landuse,
    ...boundaries,
    ...buildings,
    ...roads,
    ...places,
  ],
});

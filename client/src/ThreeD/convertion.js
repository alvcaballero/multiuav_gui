import maplibregl from 'maplibre-gl';

const latLonToXYZ = (lat, lon, alt) => {
  const radius = 6371; // Earth radius in kilometers
  const phi = (90 - lat) * (Math.PI / 180);
  const theta = (lon + 180) * (Math.PI / 180);

  const x = -(radius + alt) * Math.sin(phi) * Math.cos(theta);
  const y = (radius + alt) * Math.cos(phi);
  const z = (radius + alt) * Math.sin(phi) * Math.sin(theta);

  return [x, y, z];
};

export const calculateDistanceMercatorToMeters = (from, to) => {
  const mercatorPerMeter = from.meterInMercatorCoordinateUnits();
  // mercator x: 0=west, 1=east
  const dEast = to.x - from.x;
  const dEastMeter = dEast / mercatorPerMeter;
  // mercator y: 0=north, 1=south
  const dNorth = from.y - to.y;
  const dNorthMeter = dNorth / mercatorPerMeter;
  return { x:dEastMeter,y: dNorthMeter };
};

export const LatLon2XYZ = (from, to) => {
  const origen = maplibregl.MercatorCoordinate.fromLngLat({ lng: from.lng, lat: from.lat}, from.alt);
  if(Array.isArray(to))
  {
    return to.map((wp)=>{
    const destino = maplibregl.MercatorCoordinate.fromLngLat({ lng: wp.lng, lat: wp.lat}, wp.alt);
    const distance = calculateDistanceMercatorToMeters(origen, destino);
    return [distance.x, distance.y, wp.alt];
    })
  }
 
  const destino = maplibregl.MercatorCoordinate.fromLngLat({ lng: to.lng, lat: to.lat}, to.alt);
  const distance = calculateDistanceMercatorToMeters(origen, destino);

  return [distance.x, distance.y, to.alt];;
};

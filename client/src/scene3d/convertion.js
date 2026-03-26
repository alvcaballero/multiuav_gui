import maplibregl from 'maplibre-gl';

const round = (number, decimals = 0) => {
  const factor = Math.pow(10, decimals);
  return Math.round(number * factor) / factor;
};

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

export const LatLon2XYZObj = (origin, items,maxDistance=1000) => {
  const origen = maplibregl.MercatorCoordinate.fromLngLat({ lng: origin.lng, lat: origin.lat}, origin.alt);
  if(Array.isArray(items))
  { 
    const result = [];
    for (let i = 0; i < items.length; i++) {
        const item = items[i];
        const destino = maplibregl.MercatorCoordinate.fromLngLat({ lng: item.lng, lat: item.lat}, item.alt);
        const distance = calculateDistanceMercatorToMeters(origen, destino);
        if(distance.x > -maxDistance && distance.x < maxDistance && distance.y > -maxDistance && distance.y < maxDistance){
            result.push({...item, pos:[round(distance.x,1), round(distance.y,1), round(item.alt,1)]});
        }
    }
    return result;
  }
 
  const destino = maplibregl.MercatorCoordinate.fromLngLat({ lng: items.lng, lat: items.lat}, items.alt);
  const distance = calculateDistanceMercatorToMeters(origen, destino);

  return {...items,pos:[distance.x, distance.y, items.alt]};
};

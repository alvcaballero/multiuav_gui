// Centroide (lat/lng/alt promedio) de todos los waypoints de todas las rutas de una misión.
// Usado para centrar el origen de la escena 3D en la zona de la misión actual.
export const getMissionCentroid = (route) => {
  let sumLat = 0;
  let sumLng = 0;
  let sumAlt = 0;
  let count = 0;

  route.forEach((rt) => {
    rt.wp.forEach((wp) => {
      sumLat += wp.pos[0];
      sumLng += wp.pos[1];
      sumAlt += wp.pos[2] || 0;
      count++;
    });
  });

  if (count === 0) return null;

  return { lat: sumLat / count, lng: sumLng / count, alt: sumAlt / count };
};

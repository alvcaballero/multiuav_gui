import { useEffect, useRef } from 'react';
import { useSelector } from 'react-redux';
import { map } from '../core/mapInstance';
import { usePreference } from '../../shared/preferences';

// Centra el mapa en el home de la misión cuando se carga una nueva (state.mission.home
// cambia). No debe disparar en el montaje: este componente vive dentro de MainMap, que
// se remonta en cada cambio de página, y el home ya cargado de antes no debe volver a
// pisar la posición donde el usuario dejó el mapa.
const MapMissionHome = () => {
  const missionHome = useSelector((state) => state.mission.home);
  const defaultZoom = usePreference('zoom', 10);

  const previousMissionHomeRef = useRef(missionHome);
  useEffect(() => {
    const changed = previousMissionHomeRef.current !== missionHome;
    previousMissionHomeRef.current = missionHome;
    if (!changed || !missionHome) return;
    map.easeTo({
      center: [missionHome[1], missionHome[0]],
      zoom: Math.max(map.getZoom(), defaultZoom),
      offset: [0, -1 / 2],
    });
  }, [missionHome, defaultZoom]);

  return null;
};

export default MapMissionHome;

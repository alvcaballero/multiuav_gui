import { useRef, useEffect } from 'react';
import { useSelector } from 'react-redux';
import { map } from '../core/MapView';
import { usePrevious } from '../../reactHelper';


const MapSelectedDevice = () => {
  const selectedDeviceId = useSelector((state) => state.devices.selectedId);
  const previousDeviceId = usePrevious(selectedDeviceId);
  const mapFollow  =  useSelector((state) =>state.devices.follow)
  const selectZoom = 10;

  const position = useSelector((state) => state.session.positions[selectedDeviceId]);
  const previousPosition = usePrevious(position);

  useEffect(() => {
    const positionChanged = position && (!previousPosition || position.latitude !== previousPosition.latitude || position.longitude !== previousPosition.longitude);


    if ((selectedDeviceId !== previousDeviceId ||  (mapFollow && positionChanged)) && position) {
      if (position.hasOwnProperty('latitude')) {
        map.easeTo({
          center: [position.longitude, position.latitude],
          zoom: Math.max(map.getZoom(), selectZoom),
          offset: [0, -300 / 2],
        });
      }
    }
  }, [selectedDeviceId, previousDeviceId, mapFollow, position, selectZoom]);

  return null;
};

export default MapSelectedDevice;

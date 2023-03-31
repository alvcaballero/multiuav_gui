import { useRef,useEffect } from 'react';

import { useSelector } from 'react-redux';
import { map } from './Mapview';

const usePrevious = (value) => {
    const ref = useRef();
    useEffect(() => {
      ref.current = value;
    });
    return ref.current;
  };

const MapSelectedDevice = () => {
  const selectedDeviceId = useSelector((state) => state.devices.selectedId);
  const previousDeviceId = usePrevious(selectedDeviceId);

  const selectZoom = 10;
  const mapFollow = false;

  const position = useSelector((state) => state.data.positions[selectedDeviceId]);

  useEffect(() => {
    if ((selectedDeviceId !== previousDeviceId || mapFollow) && position) {
      map.easeTo({
        center: [position.longitude, position.latitude],
        zoom: Math.max(map.getZoom(), selectZoom),
        offset: [0, -300/ 2],
      });
    }
  });

  return null;
};

export default MapSelectedDevice;

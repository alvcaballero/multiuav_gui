import { useEffect, useRef } from 'react';

import { useSelector ,useDispatch} from 'react-redux';
import { sessionActions } from '../../store';

const SelectDevice3D = () => {
  const selectedDeviceId = useSelector((state) => state.devices.selectedId);
  // Arranca en el id ya seleccionado al montar, para no disparar un recentrado
  // (y pisar el origen recién fijado por el Pegman/misión) en el primer render.
  const previousDeviceId = useRef(selectedDeviceId);

  const dispatch = useDispatch();

  const position = useSelector((state) => state.session.positions[selectedDeviceId]);

  useEffect(() => {
    if (selectedDeviceId !== previousDeviceId.current && position?.latitude !== undefined) {
      dispatch(sessionActions.updateScene3dOrigin({ lng: position.longitude, lat: position.latitude, alt: 400 }));
    }
    previousDeviceId.current = selectedDeviceId;
  }, [selectedDeviceId, position, dispatch]);

  return null;
};

export default SelectDevice3D;

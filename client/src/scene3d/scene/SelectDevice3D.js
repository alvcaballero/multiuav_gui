import { useEffect } from 'react';

import { useSelector ,useDispatch} from 'react-redux';
import { sessionActions } from '../../store';
import { usePrevious } from '../../reactHelper';


const SelectDevice3D = () => {
  const selectedDeviceId = useSelector((state) => state.devices.selectedId);
  const previousDeviceId = usePrevious(selectedDeviceId);

  const dispatch = useDispatch();

  const position = useSelector((state) => state.session.positions[selectedDeviceId]);

  useEffect(() => {
    if (selectedDeviceId !== previousDeviceId && position?.latitude !== undefined) {
      dispatch(sessionActions.updateScene3dOrigin({ lng: position.longitude, lat: position.latitude, alt: 400 }));
    }
  }, [selectedDeviceId, previousDeviceId]);

  return null;
};

export default SelectDevice3D;

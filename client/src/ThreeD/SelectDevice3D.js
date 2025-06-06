import { useRef, useEffect } from 'react';

import { useSelector ,useDispatch} from 'react-redux';
import { sessionActions } from '../store';
import { LatLon2XYZ , LatLon2XYZObj} from './convertion';
import { usePrevious } from '../reactHelper';


const SelectDevice3D = () => {
  const selectedDeviceId = useSelector((state) => state.devices.selectedId);
  const previousDeviceId = usePrevious(selectedDeviceId);
  const mapFollow  =  useSelector((state) =>state.devices.follow)

  const dispatch = useDispatch();



  const position = useSelector((state) => state.session.positions[selectedDeviceId]);
  const previousPosition = usePrevious(position);


  useEffect(() => {
    const positionChanged = position && (!previousPosition || position.latitude !== previousPosition.latitude || position.longitude !== previousPosition.longitude);


    if ((selectedDeviceId !== previousDeviceId ||  (mapFollow && positionChanged)) && position) {

      if (position.hasOwnProperty('latitude')) {
        dispatch(sessionActions.updateScene3dOrigin({lng:position.longitude,lat: position.latitude,alt:400}))
      }
    }

  },[[selectedDeviceId, previousDeviceId, mapFollow, position]]);

  return null;
};

export default SelectDevice3D;

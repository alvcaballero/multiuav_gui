import React, { useState, useRef, useEffect, useMemo } from 'react';
import { useDispatch, useSelector, connect } from 'react-redux';

export const RosContext = React.createContext();

export const RosControl = ({ children }) => {
  const [rosState, setrosState] = useState(false);
  const [confirmMission, setconfirmMission] = useState(false);
  const serverState = useSelector((state) => state.session.serverROS);

  useEffect(() => {
    console.log('RosControl mounted');
    return () => {
      console.log('RosControl unmounted');
    };
  }, []);

  useEffect(() => {
    setrosState(serverState);
  }, [serverState]);

  const contextValue = useMemo(
    () => ({
      rosState,
      confirmMission,
      setconfirmMission,
    }),
    [rosState, confirmMission]
  );

  return <RosContext.Provider value={contextValue}>{children}</RosContext.Provider>;
};

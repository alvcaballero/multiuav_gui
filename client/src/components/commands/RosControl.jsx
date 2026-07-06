import React, { useState, useEffect, useMemo } from 'react';
import { useSelector } from 'react-redux';

export const RosContext = React.createContext();

export const RosControl = ({ children }) => {
  const [rosState, setRosState] = useState(false);
  const [confirmMission, setConfirmMission] = useState(false);
  const serverState = useSelector((state) => state.session.serverROS);

  useEffect(() => {
    console.log('RosControl mounted');
    return () => {
      console.log('RosControl unmounted');
    };
  }, []);

  useEffect(() => {
    setRosState(serverState);
  }, [serverState]);

  const contextValue = useMemo(
    () => ({
      rosState,
      confirmMission,
      setconfirmMission: setConfirmMission,
    }),
    [rosState, confirmMission],
  );

  return <RosContext value={contextValue}>{children}</RosContext>;
};

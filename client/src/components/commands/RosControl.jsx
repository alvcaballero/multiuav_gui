import React, { useState, useEffect, useMemo } from 'react';
import { useSelector } from 'react-redux';

export const RosContext = React.createContext();

export const RosControl = ({ children }) => {
  const [confirmMission, setConfirmMission] = useState(false);
  const serverState = useSelector((state) => state.session.serverROS);
  const rosState = serverState;

  useEffect(() => {
    console.log('RosControl mounted');
    return () => {
      console.log('RosControl unmounted');
    };
  }, []);

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

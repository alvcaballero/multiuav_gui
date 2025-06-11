import React, { useState, useRef, useEffect } from 'react';
import { useDispatch, useSelector, connect } from 'react-redux';

export const RosContext = React.createContext();

export const RosControl = ({ children }) => {
  const [rosState, setrosState] = useState(false);
  const [confirmMission, setconfirmMission] = useState(false);
  const serverState = useSelector((state) => state.session.server.rosState);

  useEffect(() => {
    console.log('RosControl mounted');
    return () => {
      console.log('RosControl unmounted');
    };
  }, []);

  useEffect(() => {
    setrosState(serverState);
  }, [serverState]);

  return (
    <div style={{ width: '100%', height: '100%' }}>
      <RosContext.Provider
        value={{
          rosState,
          confirmMission,
          setconfirmMission,
        }}
      >
        {children}
      </RosContext.Provider>
    </div>
  );
};

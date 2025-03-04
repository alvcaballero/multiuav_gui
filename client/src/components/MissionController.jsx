import React, { useState } from 'react';

export const MissionContext = React.createContext();

export const missionController = ({ children }) => {
  const [changeWp, setChangeWp] = useState({ route_id: -1, wp_id: -1 });

  return (
    <div style={{ width: '100%', height: '100%' }}>
      <MissionContext.Provider
        value={{
          changeWp,
          setChangeWp,
        }}
      >
        {children}
      </MissionContext.Provider>
    </div>
  );
};

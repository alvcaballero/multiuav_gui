import React, { useState, useMemo } from 'react';

export const MissionContext = React.createContext();

export const MissionController = ({ children }) => {
  const [changeWp, setChangeWp] = useState({ route_id: -1, wp_id: -1 });

  const contextValue = useMemo(
    () => ({
      changeWp,
      setChangeWp,
    }),
    [changeWp]
  );

  return <MissionContext.Provider value={contextValue}>{children}</MissionContext.Provider>;
};

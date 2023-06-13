import React, { useState } from 'react';
import {
  Route, Routes, useLocation, useNavigate,Link
} from 'react-router-dom';
import { useEffectAsync } from './reactHelper';
import App from './App';
import useQuery from './common/useQuery';
import MainPage from './MainPage'
import MissionPage from './MissionPage';
import EventPage from './EventPage';
import ReplayPage from './ReplayPage';
import DevicePage from './DevicePage';

const padding = {
    padding: 5
  }

const Navigation = () => {
  const navigate = useNavigate();
  
  
  
  const { pathname } = useLocation();
  const query = useQuery();

  useEffectAsync(async () => {
      navigate('/');
  }, [query]);
  return (
    <Routes>
        <Route path="/" element={<App/>}>
            <Route index element={<MainPage />} />

            <Route path="mission" element={<MissionPage />} />
            <Route path="device/:id" element={<DevicePage />} />
            <Route path="event/:id" element={<EventPage />} />
            <Route path="replay" element={<ReplayPage />} />

        </Route>

    </Routes>
  )
}

export default Navigation;
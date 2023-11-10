import React, { useState } from 'react';
import { Route, Routes, useLocation, useNavigate, Link } from 'react-router-dom';
import { useEffectAsync } from './reactHelper';
import App from './App';
import useQuery from './common/useQuery';
import MainPage from './other/MainPage';
import MissionPage from './other/MissionPage';
import EventPage from './other/EventPage';
import ReplayPage from './other/ReplayPage';
import DevicePage from './other/DevicePage';
import CameraPage from './other/CameraPage';
import EventsPage from './other/EventsPage';
import TopicsPage from './other/TopicsPage';

const padding = {
  padding: 5,
};

const Navigation = () => {
  const navigate = useNavigate();

  const { pathname } = useLocation();
  const query = useQuery();

  useEffectAsync(async () => {
    navigate('/');
  }, [query]);
  return (
    <Routes>
      <Route path='/' element={<App />}>
        <Route index element={<MainPage />} />

        <Route path='mission' element={<MissionPage />} />
        <Route path='camera' element={<CameraPage />} />
        <Route path='device/:id' element={<DevicePage />} />
        <Route path='event/:id' element={<EventPage />} />
        <Route path='events' element={<EventsPage />} />

        <Route path='replay' element={<ReplayPage />} />
        <Route path='topics' element={<TopicsPage />} />
      </Route>
    </Routes>
  );
};

export default Navigation;

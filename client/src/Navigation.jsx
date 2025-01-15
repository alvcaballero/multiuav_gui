import React, { useState } from 'react';
import { Route, Routes, useLocation, useNavigate, Link } from 'react-router-dom';
import { useEffectAsync } from './reactHelper';
import App from './App';
import useQuery from './common/useQuery';
import MainPage from './other/MainPage';
import MissionPage from './other/MissionPage';
import MissionPageTest from './other/MissionPageTest';
import MissionPage3D from './other/MissionPage3D';
import ReplayPage from './other/ReplayPage';
import DevicePage from './other/DevicePage';
import CameraPage from './other/CameraPage';
import EventsPage from './other/EventsPage';
import TopicsPage from './other/TopicsPage';
import PlanningPage from './other/PlanningPage';
import MissionReportPage from './other/MissionReportPage';
import MissionReportRoutePage from './other/MissionReportRoutePage';
import MissionDetailReportPage from './other/MissionDetailReportPage';
import SettingsCategoryPage from './settings/SettingsCategoryPage';
import SettingsCategoryPageEdit from './settings/SettingsCategoryPageEdit';
import SettingsDevicesPage from './settings/SettingsDevicesPage';
import SettingsDevicesPageEdit from './settings/SettingsDevicesPageEdit';

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
      <Route path="/" element={<App />}>
        <Route index element={<MainPage />} />

        <Route path="mission" element={<MissionPage />} />
        <Route path="missiontest" element={<MissionPageTest />} />
        <Route path="3Dmission" element={<MissionPage3D />} />
        <Route path="planning" element={<PlanningPage />} />
        <Route path="camera" element={<CameraPage />} />
        <Route path="device/:id" element={<DevicePage />} />

        <Route path="replay" element={<ReplayPage />} />
        <Route path="topics" element={<TopicsPage />} />

        <Route path="event/:id" element={<EventsPage />} />

        <Route path="settings">
          <Route path="devices" element={<SettingsDevicesPage />} />
          <Route path="devices/:id" element={<SettingsDevicesPageEdit />} />
          <Route path="category" element={<SettingsCategoryPage />} />
          <Route path="category/:id" element={<SettingsCategoryPageEdit />} />

        </Route>

        <Route path="reports">
          <Route path="events" element={<EventsPage />} />
          <Route path="mission" element={<MissionReportPage />} />
          <Route path="mission/:id" element={<MissionDetailReportPage />} />
          <Route path="route" element={<MissionReportRoutePage />} />
        </Route>
      </Route>
    </Routes>
  );
};

export default Navigation;

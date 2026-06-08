import React, { useState, Suspense, lazy } from 'react';
import { Route, Routes, useLocation, useNavigate, Link } from 'react-router-dom';
import { CircularProgress, Box } from '@mui/material';
import { useEffectAsync } from './reactHelper';
import App from './App';
import useQuery from './shared/useQuery';
import MainPage from './pages/MainPage';
import MissionPage from './pages/MissionPage';
import MissionPageTest from './pages/MissionPageTest';
import ReplayPage from './pages/ReplayPage';
import DevicePage from './pages/DevicePage';
import CameraPage from './pages/CameraPage';
import EventsPage from './pages/EventsPage';
import TopicsPage from './pages/TopicsPage';
import PlanningPage from './pages/PlanningPage';
import MissionReportPage from './pages/MissionReportPage';
import MissionReportRoutePage from './pages/MissionReportRoutePage';
import MissionDetailReportPage from './pages/MissionDetailReportPage';
import SettingsCategoryPage from './settings/SettingsCategoryPage';
import SettingsCategoryPageEdit from './settings/SettingsCategoryPageEdit';
import SettingsDevicesPage from './settings/SettingsDevicesPage';
import SettingsDevicesPageEdit from './settings/SettingsDevicesPageEdit';
import GeofencesPage from './pages/GeofencesPage';
import GeofencePage from './settings/GeofencePage';
import ChatPage from './pages/ChatPage';

const MainPage3D = lazy(() => import('./pages/MainPage3D'));
const MissionPage3D = lazy(() => import('./pages/MissionPage3D'));
const Scene3DEditorPage = lazy(() => import('./pages/Scene3DEditorPage'));
const DevicePage3D = lazy(() => import('./pages/DevicePage3D'));

const Loader3D = () => (
  <Box display="flex" justifyContent="center" alignItems="center" height="100%">
    <CircularProgress />
  </Box>
);
const padding = {
  padding: 5,
};

const Navigation = () => {
  const navigate = useNavigate();

  const { pathname } = useLocation();
  const query = useQuery();

  useEffectAsync(async () => {
    if (!query.get('redirect')) return;
    navigate('/');
  }, [query]);
  return (
    <Routes>
      <Route path="/" element={<App />}>
        <Route index element={<MainPage />} />

        {/* This is a temporary route for testing purposes, can be removed later
         */}
        <Route path="chat" element={<ChatPage />} />
        <Route path="3Dview" element={<Suspense fallback={<Loader3D />}><MainPage3D /></Suspense>} />
        <Route path="3Deditor" element={<Suspense fallback={<Loader3D />}><Scene3DEditorPage /></Suspense>} />
        <Route path="3Dmission" element={<Suspense fallback={<Loader3D />}><MissionPage3D /></Suspense>} />

        <Route path="mission" element={<MissionPage />} />
        <Route path="missiontest" element={<MissionPageTest />} />
        <Route path="planning" element={<PlanningPage />} />
        <Route path="camera" element={<CameraPage />} />
        <Route path="device/:id" element={<DevicePage />} />
        <Route path="device3d/:id" element={<Suspense fallback={<Loader3D />}><DevicePage3D /></Suspense>} />

        <Route path="replay" element={<ReplayPage />} />
        <Route path="topics" element={<TopicsPage />} />
        <Route path="geofences" element={<GeofencesPage />} />

        <Route path="event/:id" element={<EventsPage />} />

        <Route path="settings">
          <Route path="devices" element={<SettingsDevicesPage />} />
          <Route path="devices/:id" element={<SettingsDevicesPageEdit />} />
          <Route path="category" element={<SettingsCategoryPage />} />
          <Route path="category/:id" element={<SettingsCategoryPageEdit />} />
          <Route path="geofence/:id" element={<GeofencePage />} />
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

import React, { Suspense, lazy } from 'react';
import { Route, Routes, useNavigate } from 'react-router-dom';
import { useAsyncTask } from './reactHelper';
import App from './App';
import useQuery from './shared/useQuery';
import MainPage from './pages/MainPage';

const MissionPage = lazy(() => import('./pages/MissionPage'));
const MissionPageTest = lazy(() => import('./pages/MissionPageTest'));
const ReplayPage = lazy(() => import('./pages/ReplayPage'));
const DevicePage = lazy(() => import('./pages/DevicePage'));
const CameraPage = lazy(() => import('./pages/CameraPage'));
const EventsPage = lazy(() => import('./pages/EventsPage'));
const TopicsPage = lazy(() => import('./pages/TopicsPage'));
const PlanningPage = lazy(() => import('./pages/PlanningPage'));
const MissionReportPage = lazy(() => import('./pages/MissionReportPage'));
const MissionReportRoutePage = lazy(() => import('./pages/MissionReportRoutePage'));
const MissionDetailReportPage = lazy(() => import('./pages/MissionDetailReportPage'));
const SettingsCategoryPage = lazy(() => import('./settings/SettingsCategoryPage'));
const SettingsCategoryPageEdit = lazy(() => import('./settings/SettingsCategoryPageEdit'));
const SettingsDevicesPage = lazy(() => import('./settings/SettingsDevicesPage'));
const SettingsDevicesPageEdit = lazy(() => import('./settings/SettingsDevicesPageEdit'));
const GeofencesPage = lazy(() => import('./pages/GeofencesPage'));
const GeofencePage = lazy(() => import('./settings/GeofencePage'));
const ChatPage = lazy(() => import('./pages/ChatPage'));
const MainPage3D = lazy(() => import('./pages/MainPage3D'));
const MissionPage3D = lazy(() => import('./pages/MissionPage3D'));
const Scene3DEditorPage = lazy(() => import('./pages/Scene3DEditorPage'));
const DevicePage3D = lazy(() => import('./pages/DevicePage3D'));

const PageLoader = () => (
  <div
    style={{
      position: 'fixed',
      inset: 0,
      display: 'flex',
      alignItems: 'center',
      justifyContent: 'center',
      zIndex: 9999,
    }}
  >
    <div
      style={{
        width: 48,
        height: 48,
        border: '4px solid #e0e0e0',
        borderTopColor: '#1976d2',
        borderRadius: '50%',
        animation: 'page-spin 0.8s linear infinite',
      }}
    />
    <style>{`@keyframes page-spin { to { transform: rotate(360deg); } }`}</style>
  </div>
);

const Navigation = () => {
  const navigate = useNavigate();

  const query = useQuery();

  useAsyncTask(async () => {
    if (!query.get('redirect')) return;
    navigate('/');
  }, [query, navigate]);

  return (
    <Suspense fallback={<PageLoader />}>
      <Routes>
        <Route path="/" element={<App />}>
          <Route index element={<MainPage />} />

          <Route path="chat" element={<ChatPage />} />
          <Route path="3Dview" element={<MainPage3D />} />
          <Route path="3Deditor" element={<Scene3DEditorPage />} />
          <Route path="3Dmission" element={<MissionPage3D />} />

          <Route path="mission" element={<MissionPage />} />
          <Route path="missiontest" element={<MissionPageTest />} />
          <Route path="planning" element={<PlanningPage />} />
          <Route path="camera" element={<CameraPage />} />
          <Route path="device/:id" element={<DevicePage />} />
          <Route path="device3d/:id" element={<DevicePage3D />} />

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
    </Suspense>
  );
};

export default Navigation;

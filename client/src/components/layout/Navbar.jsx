import React, { useRef, Fragment, useEffect } from 'react';
import { useDispatch, useSelector } from 'react-redux';
import { useNavigate } from 'react-router-dom';
import { AppBar, Toolbar, Container, Typography, Button } from '@mui/material';

import { missionActions, sessionActions, activeMissionsActions } from '../../store';
import { map } from '../../map/core/MapView';
import MenuItems from './MenuItems';
import { connectRos, commandLoadMission } from '../../shared/fetchs';
import { useCatch } from '../../reactHelper';
import { usePreference } from '../../shared/preferences';
import { readTextFile, parseKmlElements } from '../../services/fileService';
import { useMissionFile } from '../../services/useMissionFile';
import { getMissionCentroid } from '../../shared/util/missionGeo';

const Navbar = React.memo(
  ({ SetAddUAVOpen, setconfirmMission = (item) => item, setChatOpen = () => null }) => {
    const dispatch = useDispatch();
    const navigate = useNavigate();
    const mission = useSelector((state) => state.mission);
    const llmEnabled = useSelector((state) => state.session.server?.llmEnabled ?? false);

    const handleConnectRos = useCatch(connectRos);
    // Guards against firing loadMission twice in a row (menu item has no disabled
    // state to show) — would create duplicate Mission/Plan/Route rows server-side.
    const loadingMissionRef = useRef(false);
    const handleCommandLoadMission = useCatch(async () => {
      if (loadingMissionRef.current) return;
      loadingMissionRef.current = true;
      try {
        const res = await commandLoadMission(mission);
        // Select the created mission so it becomes the one commandMission will command.
        if (res?.missionId != null) dispatch(activeMissionsActions.selectMission(res.missionId));
        return res;
      } finally {
        loadingMissionRef.current = false;
      }
    });
    const handleMissionFile = useMissionFile();
    const defaultLatitude = usePreference('latitude', 0);
    const defaultLongitude = usePreference('longitude', 0);
    const defaultZoom = usePreference('zoom', 10);
    useEffect(() => {
      console.log('render navbar');
    }, []);

    const menuItemsData = [
      {
        title: 'ROS',
        submenu: [
          { title: 'Connect ROS', action: () => handleConnectRos() },
          { title: 'Show Topics', action: () => navigate('/topics') },
          { title: 'Show Services' },
        ],
      },
      {
        title: 'Devices',
        submenu: [
          { title: 'Connect Devices', action: () => openAddUav() },
          { title: 'Load Mission all', action: () => handleCommandLoadMission() },
          { title: 'Command Mission All', action: () => setconfirmMission(true) },
        ],
      },
      {
        title: 'Mission',
        submenu: [
          { title: ' Open Mission', input: (e) => readFile(e) },
          { title: 'Clear mission', action: () => clearmission() },
          { title: 'Edit mission', action: () => navigate('/mission') },
          { title: 'Planning', action: () => navigate('/planning') },
        ],
      },
      {
        title: 'View',
        submenu: [
          { title: 'Geofrences', action: () => navigate('/geofences') },
          { title: 'add elements', input: (e) => loadElements(e), type: '.kml' },
          { title: 'Camera view', action: () => navigate('/camera') },
          { title: '3D view', action: () => navigate('/3Dview') },
          { title: '3D Missionview', action: () => goto3DMission() },
          { title: '3D Editor', action: () => navigate('/3Deditor') },
          { title: 'Mission test', action: () => navigate('/missiontest') },
        ],
      },
      {
        title: 'Report',
        submenu: [
          { title: 'Missions', action: () => navigate('/reports/mission') },
          { title: 'Routes', action: () => navigate('/reports/route') },
          { title: 'events', action: () => navigate('/reports/events') },
        ],
      },
      {
        title: 'Settings',
        action: () => navigate('/settings/devices'),
      },
      ...(llmEnabled ? [{ title: 'Chat', action: () => setChatOpen(true) }] : []),
    ];

    const clearmission = () => {
      dispatch(missionActions.clearMission({}));
    };

    const readFile = (e) => handleMissionFile(e.target.files[0]);

    const loadElements = (e) => {
      const file = e.target.files[0];
      readTextFile(file, ({ data }) => {
        const result = parseKmlElements(data);
        if (!result) return;
        if (result.kind === 'bases') {
          dispatch(sessionActions.addMarkerBase(result.markers));
        } else {
          dispatch(sessionActions.addMarkerElement(result.markers));
        }
      });
    };

    function moveToDefaultHome() {
      map.easeTo({
        center: [defaultLongitude, defaultLatitude],
        zoom: Math.max(map.getZoom(), defaultZoom),
        offset: [0, -1 / 2],
      });
    }

    // Centra el origen 3D en el centroide de la misión actual antes de abrir la vista 3D,
    // igual que hace el Pegman al soltarse sobre un punto del mapa 2D.
    function goto3DMission() {
      const centroid = getMissionCentroid(mission.route);
      if (centroid) {
        dispatch(sessionActions.updateScene3dOrigin(centroid));
      }
      navigate('/3Dmission');
    }

    function openAddUav() {
      SetAddUAVOpen(true);
    }

    return (
      <AppBar position="static" style={{ backgroundColor: '#333', height: '52px' }}>
        <Container maxWidth="x">
          <Toolbar disableGutters>
            <Button
              onClick={() => {
                moveToDefaultHome();
                navigate('/');
              }}
            >
              <Typography
                variant="h6"
                noWrap
                component="a"
                sx={{
                  mr: 2,
                  display: { xs: 'none', md: 'flex' },
                  fontFamily: 'monospace',
                  fontWeight: 700,
                  letterSpacing: '.3rem',
                  color: '#FFFFFF',
                  textDecoration: 'none',
                }}
              >
                Management Tool
              </Typography>
            </Button>
            {menuItemsData.map((menu, index) => (
              <Fragment key={'s-' + index}>
                <MenuItems items={menu} depthLevel={0} />
              </Fragment>
            ))}
          </Toolbar>
        </Container>
      </AppBar>
    );
  },
);
export default Navbar;

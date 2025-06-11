import React, { useContext, Fragment, useEffect } from 'react';
import { useDispatch } from 'react-redux';
import { useNavigate } from 'react-router-dom';
import { AppBar, Toolbar, Container, Typography, Button } from '@mui/material';

import { missionActions, sessionActions } from '../store';
import { map } from '../Mapview/MapView';
import MenuItems from './MenuItems';
import { FiletoMission } from '../Mapview/MissionConvert';
import { connectRos, commandLoadMission, commandMission } from '../common/fetchs';
import { usePreference } from '../common/preferences';

const Navbar = React.memo(({ SetAddUAVOpen, setconfirmMission = (item) => item }) => {
  const dispatch = useDispatch();
  const navigate = useNavigate();
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
        { title: 'Connect ROS', action: () => connectRos() },
        { title: 'Show Topics', action: () => navigate('/topics') },
        { title: 'Show Services' },
      ],
    },
    {
      title: 'Devices',
      submenu: [
        { title: 'Connect Devices', action: () => openAddUav() },
        { title: 'Load Mission all', action: () => commandLoadMission() },
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
        { title: '3D Missionview', action: () => navigate('/3Dmission') },
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
  ];

  const clearmission = () => {
    dispatch(missionActions.clearMission({}));
  };

  const readFile = (e) => {
    //https://www.youtube.com/watch?v=K3SshoCXC2g
    const file = e.target.files[0];
    if (!file) return;
    const fileReader = new FileReader();
    fileReader.readAsText(file);
    fileReader.onload = () => {
      FiletoMission({ name: file.name, data: fileReader.result });
    };
    fileReader.onerror = () => {
      console.log('error');
      console.log(fileReader.error);
    };
  };

  // https://www.youtube.com/watch?v=K3SshoCXC2g
  const loadElements = (e) => {
    const file = e.target.files[0];
    if (!file) return;
    const fileReader = new FileReader();
    fileReader.readAsText(file);
    fileReader.onload = () => {
      console.log(fileReader.result);
      let xmlDocument = new DOMParser().parseFromString(fileReader.result, 'text/xml');
      console.log(xmlDocument);
      let mission_line = xmlDocument.getElementsByTagName('Point');
      console.log(mission_line);
      let mission_array = Object.values(mission_line).map((x) =>
        x.textContent
          .replace('\t1', '')
          .replace(/(\r\n|\n|\r|\t)/gm, '')
          .split(',')
      );

      console.log(mission_array);

      let mission_line1 = xmlDocument.getElementsByTagName('coordinates');
      console.log(mission_line1);
      const mission_array1 = Object.values(mission_line1).map((x) =>
        x.textContent
          .replace('\t1', '')
          .replace(/(\r\n|\n|\r|\t)/gm, '')
          .split(' ')
      );
      const missionArray2 = mission_array1.map((x) => x.map((y) => y.split(',')));
      console.log(mission_array1);
      console.log(missionArray2);
      let markers = [];
      if (mission_array.length) {
        console.log('add markers');
        markers = mission_array.map((x, index, list) => {
          return { latitude: Number(x[1]), longitude: Number(x[0]), image: 'base' };
        });
        console.log(markers);
        dispatch(sessionActions.addMarkerBase(markers));
        return null;
      }
      if (missionArray2.length) {
        console.log('add towers markers');
        missionArray2.map((x) => {
          console.log(x);
          const myList = [];
          x.map((y) => {
            console.log(y);
            if (y.length > 1) {
              myList.push({ latitude: Number(y[1]), longitude: Number(y[0]) });
            }
            return null;
          });
          markers.push({
            type: 'powerTower',
            items: myList,
          });
        });
        console.log(markers);
        dispatch(sessionActions.addMarkerElement(markers));
      }
    };
    fileReader.onerror = () => {
      console.log('error');
      console.log(fileReader.error);
    };
  };

  function sethome() {
    map.easeTo({
      center: [defaultLongitude, defaultLatitude],
      zoom: Math.max(map.getZoom(), defaultZoom),
      offset: [0, -1 / 2],
    });
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
              sethome();
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
          {React.Children.toArray(
            menuItemsData.map((menu, index) => (
              <Fragment key={'s-' + index}>
                <MenuItems items={menu} depthLevel={0} />
              </Fragment>
            ))
          )}
        </Toolbar>
      </Container>
    </AppBar>
  );
});
export default Navbar;

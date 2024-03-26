import React, { useState, useContext, Fragment } from 'react';
import { useDispatch } from 'react-redux';
import { useNavigate } from 'react-router-dom';
import {
  AppBar,
  Toolbar,
  Container,
  Typography,
  Button,
  Menu,
  MenuItem,
  Fade,
} from '@mui/material';

import { missionActions, sessionActions } from '../store';
import UploadButtons from './uploadButton';
import { map } from '../Mapview/MapView';
import { RosContext } from './RosControl';

const MenuItems = ({ items, depthLevel }) => {
  const [anchorEl, setAnchorEl] = React.useState(null);
  const open = Boolean(anchorEl);

  const handleClick = (event) => {
    setAnchorEl(event.currentTarget);
  };
  const handleClose = () => {
    setAnchorEl(null);
  };

  const handleHover = (event) => {
    if (items.submenu) {
      setAnchorEl(event.currentTarget);
    }
  };

  return (
    <div>
      <Button
        id="fade-button"
        aria-controls={open ? 'fade-menu' : undefined}
        aria-haspopup="true"
        aria-expanded={open ? 'true' : undefined}
        onClick={handleClick}
        sx={{ color: '#FFFFFF', fontSize: 16 }}
        style={{ textTransform: 'none' }}
      >
        {items.title}
      </Button>
      {items.submenu && (
        <Menu
          id="fade-menu"
          MenuListProps={{
            'aria-labelledby': 'fade-button',
          }}
          anchorEl={anchorEl}
          open={open}
          onClose={handleClose}
          TransitionComponent={Fade}
        >
          {items.submenu.map((element, index) => {
            if (element.input) {
              return (
                <UploadButtons
                  key={'men' + index}
                  title={element.title}
                  readFile={(e) => {
                    element.input(e);
                  }}
                  typefiles={element.type}
                />
              );
            }
            return (
              <MenuItem
                key={'men' + index}
                onClick={() => {
                  handleClose();
                  if (element.action) element.action();
                }}
              >
                {element.title}
              </MenuItem>
            );
          })}
        </Menu>
      )}
    </div>
  );
};
export const Navbar = ({ SetAddUAVOpen }) => {
  const dispatch = useDispatch();
  const navigate = useNavigate();
  const rosContex = useContext(RosContext);

  const menuItemsData = [
    {
      title: 'ROS',
      submenu: [
        {
          title: 'Connect ROS',
          action: () => rosContex.rosConnect(),
        },
        {
          title: 'Show Topics',
          action: () => navigate('/topics'),
        },
        {
          title: 'Show Services',
        },
      ],
    },
    {
      title: 'Devices',
      submenu: [
        {
          title: 'Connect Devices',
          action: () => openAddUav(),
        },
        {
          title: 'Load Mission all',
          action: () => rosContex.loadMission(),
        },
        {
          title: 'Command Mission All',
          action: () => rosContex.setconfirmMission(true),
        },
      ],
    },
    {
      title: 'Mission',
      submenu: [
        {
          title: ' Open Mission',
          input: (e) => readFile(e),
          type: '.yaml, .plan, .waypoint, .kml',
        },
        {
          title: 'Clear mission',
          action: () => clearmission(),
        },
        {
          title: 'Edit mission',
          action: () => navigate('/mission'),
        },
        {
          title: 'Planning',
          action: () => navigate('/planning'),
        },
      ],
    },
    {
      title: 'View',
      submenu: [
        {
          title: 'Enviroment Features',
        },
        {
          title: 'add elements',
          input: (e) => loadElements(e),
          type: '.kml',
        },
        {
          title: 'Camera view',
          action: () => navigate('/camera'),
        },
        {
          title: 'Camera view',
          action: () => navigate('/3Dmission'),
        },
      ],
    },
    {
      title: 'Report',
      submenu: [
        {
          title: 'Missions',
        },
        {
          title: 'events',
        },
      ],
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
      rosContex.openMision(file.name, fileReader.result);
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
      center: [-6.0025, 37.412],
      zoom: Math.max(map.getZoom(), 5),
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
};

import React, { useState, useContext } from 'react';
import { useDispatch } from 'react-redux';
import { useNavigate } from 'react-router-dom';
import './Navbar.css';
import { map } from '../Mapview/MapView';
import { RosContext } from './RosControl';
import { AppBar, Toolbar, Container, Typography, Button } from '@mui/material';

import { missionActions, sessionActions } from '../store';

export const Navbar = ({ SetAddUAVOpen }) => {
  const [isActive, setIsActive] = useState(false);
  const dispatch = useDispatch();
  const navigate = useNavigate();

  const clearmission = () => {
    dispatch(missionActions.clearMission({}));
  };

  const handleClick = () => {
    // 👇️ toggle
    const element = document.getElementsByName('otrotest');
    //console.log(element);
    for (let i = 0; i < element.length; i++) {
      element[i].setAttribute('class', 'mytest');
    }

    setTimeout(() => {
      console.log('1 Segundo esperado');
      for (let i = 0; i < element.length; i++) {
        element[i].setAttribute('class', 'dropdown-content');
      }
    }, 500);
    setIsActive((current) => !current);

    // 👇️ or set to true
    // setIsActive(true);
  };
  const rosContex = useContext(RosContext);

  const readFile = (e) => {
    //https://www.youtube.com/watch?v=K3SshoCXC2g
    const file = e.target.files[0];
    if (!file) return;

    const fileReader = new FileReader();
    fileReader.readAsText(file);
    fileReader.onload = () => {
      //console.log(fileReader.result);
      //console.log(file.name);

      rosContex.openMision(file.name, fileReader.result);
    };
    fileReader.onerror = () => {
      console.log('error');
      console.log(fileReader.error);
    };
  };

  const loadElements = (e) => {
    //https://www.youtube.com/watch?v=K3SshoCXC2g
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
      let mission_array1 = Object.values(mission_line1).map((x) =>
        x.textContent
          .replace('\t1', '')
          .replace(/(\r\n|\n|\r|\t)/gm, '')
          .split(' ')
      );
      let mission_array2 = mission_array1.map((x) => x.map((y) => y.split(',')));
      console.log(mission_array1);
      console.log(mission_array2);
      let markers = [];
      if (mission_array.length) {
        console.log('add markers');
        markers = mission_array.map((x, index, list) => {
          return { latitude: Number(x[1]), longitude: Number(x[0]), image: 'base' };
        });
        console.log(markers);
        dispatch(sessionActions.addMarker(markers));
        return null;
      }
      if (mission_array2.length) {
        console.log('add towers markers');
        mission_array2.map((x, index, list) => {
          console.log(x);
          x.map((y) => {
            console.log(y);
            if (y.length > 1) {
              markers.push({
                latitude: Number(y[1]),
                longitude: Number(y[0]),
                image: 'powerTower',
              });
            }
          });
        });
        console.log(markers);
        dispatch(sessionActions.addMarker(markers));
      }

      //rosContex.openMision(file.name, fileReader.result);
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
    <AppBar position='static' style={{ backgroundColor: '#333', height: '52px' }}>
      <Container maxWidth='x'>
        <Toolbar disableGutters>
          <Button
            onClick={() => {
              sethome();
              navigate('/');
            }}
          >
            <Typography
              variant='h6'
              noWrap
              component='a'
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
          <div className='dropdown'>
            <button className='dropbtn'>ROS </button>
            <div name='otrotest' className='dropdown-content'>
              <a
                id='rosConnectNavbar'
                onClick={() => {
                  rosContex.rosConnect();
                  handleClick();
                }}
              >
                Connect Ros
              </a>
              <a
                id='rosConnectNavbar'
                onClick={() => {
                  rosContex.rosConnect();
                  handleClick();
                }}
              >
                Show Topics
              </a>
              <a
                id='rosConnectNavbar'
                onClick={() => {
                  rosContex.rosConnect();
                  handleClick();
                }}
              >
                Show servies
              </a>
            </div>
          </div>
          <div className='dropdown'>
            <button className='dropbtn'>
              UAV
              <i className='fa fa-caret-down'></i>
            </button>
            <div name='otrotest' className='dropdown-content'>
              <a
                id='openAddUavNavbar'
                onClick={() => {
                  openAddUav();
                  handleClick();
                }}
              >
                Connect UAV
              </a>
              <a
                id='loadMissionNavbar'
                onClick={() => {
                  rosContex.loadMission();
                  handleClick();
                }}
              >
                Load Mission UAV's
              </a>
              <a
                id='commandMissionNavbar'
                onClick={() => {
                  rosContex.setconfirmMission(true);
                  handleClick();
                }}
              >
                Command Mission All
              </a>
            </div>
          </div>
          <div className='dropdown'>
            <button className='dropbtn'>
              Mission
              <i className='fa fa-caret-down'></i>
            </button>
            <div name='otrotest' className='dropdown-content'>
              <label id='menuopenmission' htmlFor='openMissionNavbar'>
                Open Mision
              </label>
              <input
                accept='.yaml, .plan, .waypoint, .kml'
                type='file'
                multiple={false}
                style={{ display: 'none' }}
                id='openMissionNavbar'
                onChange={readFile}
              />
              <a
                id='Clear mission'
                onClick={() => {
                  clearmission();
                  handleClick();
                }}
              >
                Clear Mission
              </a>
              <a
                id='editmission'
                onClick={() => {
                  navigate('/mission');
                  handleClick();
                }}
              >
                Edit mission
              </a>
            </div>
          </div>
          <div className='dropdown'>
            <button className='dropbtn'>
              View
              <i className='fa fa-caret-down'></i>
            </button>
            <div name='otrotest' className='dropdown-content'>
              <a id='hideRosterNavbar' onClick={handleClick}>
                Enviroment Features
              </a>
              <label id='menuopenelements' htmlFor='openElementsNavbar'>
                add elements
              </label>
              <input
                accept='.kml'
                type='file'
                multiple={false}
                style={{ display: 'none' }}
                id='openElementsNavbar'
                onChange={loadElements}
              />
              <a
                id='cameraView'
                onClick={() => {
                  navigate('/camera');
                  handleClick();
                }}
              >
                Camera view
              </a>
            </div>
          </div>
          <div className='dropdown'>
            <button
              className='dropbtn'
              onClick={() => {
                navigate('/events');
              }}
            >
              Report
              <i className='fa fa-caret-down'></i>
            </button>
          </div>
        </Toolbar>
      </Container>
    </AppBar>
  );
};

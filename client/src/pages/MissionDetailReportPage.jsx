import React, { useState, useMemo } from 'react';
import { useSelector } from 'react-redux';
import { useNavigate, useParams } from 'react-router-dom';

import { Typography, Container, Paper, AppBar, Toolbar, IconButton, Chip } from '@mui/material';

import { makeStyles } from 'tss-react/mui';

import ArrowBackIcon from '@mui/icons-material/ArrowBack';

import { formatTime } from '../shared/formatter';
import { missionStyle, routeStyle } from '../shared/missionStatus';

import { useAsyncTask } from '../reactHelper';
import ImageFull from './missionDetailReport/ImageFull';
import MissionSummarySection from './missionDetailReport/MissionSummarySection';
import MissionRoutesSection from './missionDetailReport/MissionRoutesSection';
import MissionMapPanel from './missionDetailReport/MissionMapPanel';

const useStyles = makeStyles()((theme) => ({
  root: {
    height: '100%',
    display: 'flex',
    flexDirection: 'column',
  },
  content: {
    overflow: 'auto',
    paddingTop: theme.spacing(2),
    paddingBottom: theme.spacing(2),
  },
  details: {
    display: 'flex',
    flexDirection: 'column',
    gap: theme.spacing(2),
    paddingBottom: theme.spacing(3),
  },
  missionMapOverlay: {
    width: '500px',
    height: '480px',
    position: 'absolute',
    top: '10px',
    left: '10px',
    flexDirection: 'column',
    display: 'flex',
    overflowY: 'auto',
  },
}));

const formatResult = (result) => {
  if (result && result.hasOwnProperty('measures') && result.measures.length > 0) {
    return result.measures.map((item, itemIndex) => (
      <Typography key={`m-${item.name}${itemIndex}`}>{`${item.name}: ${item.value}`}</Typography>
    ));
  }
  return null;
};

const MissionDetailReportPage = () => {
  const { classes } = useStyles();
  const navigate = useNavigate();
  const { id } = useParams();

  const [missions, setMissions] = useState(null);
  const [dataParam, setDataParam] = useState(null);
  const [routes, setRoutes] = useState(null);

  const [files, setFiles] = useState(null);
  const [selectFile, setSelectFile] = useState(null);
  const [events, setEvents] = useState(null);
  const [positions, setPositions] = useState(null);

  const routePath = missions?.mission?.route ?? null;

  const dataMission = useMemo(() => {
    if (!missions?.task?.devices) return null;
    return Object.values(missions.task.devices)
      .filter((deviceValue) => deviceValue?.settings?.base)
      .map((deviceValue) => ({
        baseId: deviceValue.id,
        device: { id: deviceValue.id, name: deviceValue.id },
        settings: deviceValue.settings ?? {},
      }));
  }, [missions]);

  const missionMarkers = useMemo(() => {
    const myBases = [];
    if (missions?.task?.devices) {
      Object.values(missions.task.devices).forEach((deviceValue) => {
        if (deviceValue?.settings?.base) {
          myBases.push({
            id: deviceValue.id,
            latitude: deviceValue.settings.base[0],
            longitude: deviceValue.settings.base[1],
          });
        }
      });
    }
    const myElements = missions?.task?.locations
      ? missions.task.locations.map((item) => ({ ...item, type: 'locPoint' }))
      : [];
    return { bases: myBases, elements: myElements };
  }, [missions]);

  // The actually-flown path, one track per route: recorded positions for that
  // route's device within its [initTime, endTime] window, kept with their
  // fixTime so the playback slider can scrub through them.
  const routeTracks = useMemo(() => {
    if (!routes || !positions) return [];
    return routes
      .map((route, routeIndex) => {
        const start = new Date(route.initTime).getTime();
        const end = route.endTime ? new Date(route.endTime).getTime() : Date.now();
        const trackPositions = positions
          .filter((position) => position.deviceId === route.deviceId)
          .filter((position) => position.longitude != null && position.latitude != null)
          .filter((position) => {
            const time = new Date(position.fixTime).getTime();
            return time >= start && time <= end;
          })
          .sort((a, b) => new Date(a.fixTime) - new Date(b.fixTime));
        return {
          id: route.id ?? routeIndex,
          deviceId: route.deviceId,
          startTime: start,
          endTime: end,
          positions: trackPositions,
        };
      })
      .filter((track) => track.positions.length > 1);
  }, [routes, positions]);

  const devices = useSelector((state) => state.devices.items);

  const [tabValue, setTabValue] = useState('1');

  const handleTabChange = (event, newTabValue) => {
    setTabValue(newTabValue);
  };

  // `axis` selects the status vocabulary: mission and route status share names
  // ('running', etc.) but mean different things, so each has its own color map.
  const formatValue = (item, key, axis = 'mission') => {
    const value = item[key];
    if (value === null || value === undefined) {
      return '';
    }
    switch (key) {
      case 'deviceId':
        return devices[value].name;
      case 'uav': {
        const uavsName = value.map((uav) => devices[uav].name);
        return uavsName.join(', ');
      }
      case 'initTime':
        return formatTime(value, 'minutes');
      case 'endTime':
        return formatTime(value, 'minutes');

      case 'status': {
        const style = axis === 'route' ? routeStyle(value) : missionStyle(value);
        return <Chip label={style.label} sx={{ backgroundColor: style.color, color: '#fff' }} />;
      }
      case 'result':
        return formatResult(value);
      default:
        return value;
    }
  };

  useAsyncTask(async () => {
    const response = await fetch(`/api/missions?id=${id}`);
    if (response.ok) {
      const myMissions = await response.json();
      setMissions(myMissions);
      console.log(myMissions);
    } else {
      throw Error(await response.text());
    }
    const response2 = await fetch(`/api/missions/routes?missionId=${id}`);
    let myroutes = [];
    if (response2.ok) {
      myroutes = await response2.json();
      setRoutes(myroutes);
      console.log(myroutes);
    } else {
      throw Error(await response.text());
    }
    const response3 = await fetch(`/api/files/get?missionId=${id}`);
    if (response3.ok) {
      const myfiles = await response3.json();
      setFiles(myfiles);
      console.log(myfiles);
    } else {
      throw Error(await response.text());
    }

    // Events carry no routeId of their own, only deviceId + eventTime, so we
    // fetch every event across the mission's full time span once here and let
    // MissionRoutesSection narrow it down per route (deviceId + time window).
    const initTimes = myroutes
      .map((route) => new Date(route.initTime).getTime())
      .filter((time) => !Number.isNaN(time));
    if (initTimes.length > 0) {
      const endTimes = myroutes
        .map((route) => (route.endTime ? new Date(route.endTime).getTime() : Date.now()))
        .filter((time) => !Number.isNaN(time));
      const params = new URLSearchParams({
        from: new Date(Math.min(...initTimes)).toISOString(),
        to: new Date(Math.max(...endTimes)).toISOString(),
      });
      const response4 = await fetch(`/api/events?${params}`);
      if (response4.ok) {
        setEvents(await response4.json());
      } else {
        throw Error(await response4.text());
      }

      // Positions carry no routeId either — same window, narrowed per route
      // (deviceId + time window) once here, similar to events above.
      const response5 = await fetch(`/api/positions?${params}`);
      if (response5.ok) {
        setPositions(await response5.json());
      } else {
        throw Error(await response5.text());
      }
    }
  }, [id]);

  useAsyncTask(async () => {
    if (
      missions &&
      missions.hasOwnProperty('task') &&
      missions.task &&
      missions.task.hasOwnProperty('case')
    ) {
      const response = await fetch(`/api/planning/missionparam/${missions.task.case}`);
      if (response.ok) {
        const myParamSettings = await response.json();
        console.log(myParamSettings);
        if (myParamSettings && myParamSettings.hasOwnProperty('settings')) {
          myParamSettings.devices.name = { name: 'Device', type: 'string', default: 'uav_0' };
          delete myParamSettings.devices.id;
          console.log(myParamSettings);
          setDataParam(myParamSettings);
        }
      } else {
        throw Error(await response.text());
      }
    }
  }, [missions]);

  return (
    <div className={classes.root}>
      <AppBar position="sticky" color="inherit">
        <Toolbar>
          <IconButton color="inherit" edge="start" sx={{ mr: 2 }} onClick={() => navigate(-1)}>
            <ArrowBackIcon />
          </IconButton>
          <Typography variant="h6" component="div">
            {`Mission Reports- ${id}`}
          </Typography>
        </Toolbar>
      </AppBar>
      <div className={classes.content}>
        <Container maxWidth="xl">
          <Paper style={{ padding: 30 }}>
            {missions && (
              <MissionSummarySection
                missions={missions}
                formatValue={formatValue}
                formatResult={formatResult}
              />
            )}
            {routes && (
              <MissionRoutesSection
                routes={routes}
                files={files}
                events={events}
                formatValue={formatValue}
                onSelectFile={setSelectFile}
              />
            )}
            {missions && (
              <MissionMapPanel
                classes={classes}
                missions={missions}
                routePath={routePath}
                missionMarkers={missionMarkers}
                routeTracks={routeTracks}
                dataMission={dataMission}
                dataParam={dataParam}
                tabValue={tabValue}
                onTabChange={handleTabChange}
              />
            )}
          </Paper>
        </Container>
      </div>
      <ImageFull file={selectFile} closecard={() => setSelectFile(null)} />
    </div>
  );
};

export default MissionDetailReportPage;

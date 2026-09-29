import React, { useState, useMemo } from 'react';
import { useSelector } from 'react-redux';
import { useNavigate, useParams } from 'react-router-dom';

import { Typography, Container, Paper, AppBar, Toolbar, IconButton, Chip } from '@mui/material';

import { makeStyles } from 'tss-react/mui';

import ArrowBackIcon from '@mui/icons-material/ArrowBack';

import { formatTime } from '../shared/formatter';
import { missionStyle, taskStyle } from '../shared/missionStatus';

import { useAsyncTask } from '../reactHelper';
import ImageFull from './missionDetailReport/ImageFull';
import MissionSummarySection from './missionDetailReport/MissionSummarySection';
import MissionTasksSection from './missionDetailReport/MissionTasksSection';
import MissionMapPanel from './missionDetailReport/MissionMapPanel';
import { routeColorKey } from '../shared/routeColors';

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
  const [tasks, setTasks] = useState(null);

  const [files, setFiles] = useState(null);
  const [selectFile, setSelectFile] = useState(null);
  const [events, setEvents] = useState(null);
  const [positions, setPositions] = useState(null);

  // Planned path: legacy plans carry route[], task-graph plans tasks[] — both with wp[].
  const routePath = missions?.mission?.route ?? missions?.mission?.tasks ?? null;

  const dataMission = useMemo(() => {
    if (!missions?.request?.devices) return null;
    return Object.values(missions.request.devices)
      .filter((deviceValue) => deviceValue?.settings?.base)
      .map((deviceValue) => ({
        baseId: deviceValue.id,
        device: { id: deviceValue.id, name: deviceValue.id },
        settings: deviceValue.settings ?? {},
      }));
  }, [missions]);

  const missionMarkers = useMemo(() => {
    const myBases = [];
    if (missions?.request?.devices) {
      Object.values(missions.request.devices).forEach((deviceValue) => {
        if (deviceValue?.settings?.base) {
          myBases.push({
            id: deviceValue.id,
            latitude: deviceValue.settings.base[0],
            longitude: deviceValue.settings.base[1],
          });
        }
      });
    }
    const myElements = missions?.request?.locations
      ? missions.request.locations.map((item) => ({ ...item, type: 'locPoint' }))
      : [];
    return { bases: myBases, elements: myElements };
  }, [missions]);

  // The actually-flown path, one track per task: recorded positions for that
  // task's device within its [initTime, endTime] window, kept with their
  // fixTime so the playback slider can scrub through them.
  const routeTracks = useMemo(() => {
    if (!tasks || !positions) return [];
    return tasks
      .map((task, taskIndex) => {
        const start = new Date(task.initTime).getTime();
        const end = task.endTime ? new Date(task.endTime).getTime() : Date.now();
        const trackPositions = positions
          .filter((position) => position.deviceId === task.deviceId)
          .filter((position) => position.longitude != null && position.latitude != null)
          .filter((position) => {
            const time = new Date(position.fixTime).getTime();
            return time >= start && time <= end;
          })
          .sort((a, b) => new Date(a.fixTime) - new Date(b.fixTime));
        // Drawn in its planned route's color. A legacy route[] has no task_id: the
        // server named its tasks T1..Tn in order.
        const routeIndex = (routePath ?? []).findIndex(
          (route, index) => (route.task_id ?? `T${index + 1}`) === task.taskKey,
        );
        return {
          id: task.id ?? taskIndex,
          deviceId: task.deviceId,
          routeKey: routeIndex < 0 ? null : routeColorKey(routePath[routeIndex], routeIndex),
          startTime: start,
          endTime: end,
          positions: trackPositions,
        };
      })
      .filter((track) => track.positions.length > 1);
  }, [tasks, positions, routePath]);

  const devices = useSelector((state) => state.devices.items);

  const [tabValue, setTabValue] = useState('1');

  const handleTabChange = (event, newTabValue) => {
    setTabValue(newTabValue);
  };

  // `axis` selects the status vocabulary: mission and task status share names
  // ('running', etc.) but mean different things, so each has its own color map.
  const formatValue = (item, key, axis = 'mission') => {
    const value = item[key];
    if (value === null || value === undefined) {
      return '';
    }
    switch (key) {
      case 'deviceId':
        return devices[value]?.name ?? '—';
      case 'uav': {
        const uavsName = value.map((uav) => devices[uav]?.name ?? uav);
        return uavsName.join(', ');
      }
      case 'initTime':
        return formatTime(value, 'minutes');
      case 'endTime':
        return formatTime(value, 'minutes');

      case 'status': {
        const style = axis === 'task' ? taskStyle(value) : missionStyle(value);
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
    const response2 = await fetch(`/api/missions/tasks?missionId=${id}`);
    let mytasks = [];
    if (response2.ok) {
      mytasks = await response2.json();
      setTasks(mytasks);
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

    // Events carry no taskId of their own, only deviceId + eventTime, so we
    // fetch every event across the mission's full time span once here and let
    // MissionTasksSection narrow it down per task (deviceId + time window).
    const initTimes = mytasks
      .map((task) => new Date(task.initTime).getTime())
      .filter((time) => !Number.isNaN(time));
    if (initTimes.length > 0) {
      const endTimes = mytasks
        .map((task) => (task.endTime ? new Date(task.endTime).getTime() : Date.now()))
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

      // Positions history requires a single deviceId per request (server-side
      // constraint), so fetch one call per device involved in the mission and
      // concatenate — MissionTasksSection then narrows per task (deviceId +
      // time window), similar to events above.
      const deviceIds = [...new Set(mytasks.map((task) => task.deviceId).filter((d) => d != null))];
      const positionResponses = await Promise.all(
        deviceIds.map((deviceId) =>
          fetch(
            `/api/positions?${new URLSearchParams({ ...Object.fromEntries(params), deviceId })}`,
          ),
        ),
      );
      const failedResponse = positionResponses.find((response) => !response.ok);
      if (failedResponse) {
        throw Error(await failedResponse.text());
      }
      const positionResults = await Promise.all(
        positionResponses.map((response) => response.json()),
      );
      setPositions(positionResults.flat());
    }
  }, [id]);

  useAsyncTask(async () => {
    if (
      missions &&
      missions.hasOwnProperty('task') &&
      missions.request &&
      missions.request.hasOwnProperty('case')
    ) {
      const response = await fetch(`/api/planning/missionparam/${missions.request.case}`);
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
            {tasks && (
              <MissionTasksSection
                tasks={tasks}
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

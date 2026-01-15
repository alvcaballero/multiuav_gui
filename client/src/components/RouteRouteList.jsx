import React, { Fragment, useEffect, useState } from 'react';
import { useDispatch, useSelector } from 'react-redux';
import { makeStyles } from 'tss-react/mui';

import {
  Box,
  Button,
  IconButton,
  TextField,
  Accordion,
  AccordionSummary,
  AccordionDetails,
  Typography,
} from '@mui/material';

import ExpandMore from '@mui/icons-material/ExpandMore';
import SelectField from '../common/components/SelectField';
import DeleteIcon from '@mui/icons-material/Delete';
import palette from '../common/palette';
import { map } from '../Mapview/MapView';
import WaypointRouteList from './WaypointRouteList';
import { missionActions } from '../store';

const useStyles = makeStyles()((theme) => ({
  list: {
    maxHeight: '100%',
    overflow: 'auto',
  },
  icon: {
    width: '25px',
    height: '25px',
    filter: 'brightness(0) invert(1)',
  },
  details: {
    display: 'flex',
    flexDirection: 'column',
    gap: theme.spacing(2),
    paddingBottom: theme.spacing(3),
  },
  attribute: {
    display: 'inline-block',
    width: '40%',
    textAlign: 'left',
    verticalAlign: 'middle',
  },
  attributeValue: {
    display: 'inline-block',
    width: '58%',
  },
  actionValue: {
    display: 'inline-block',
    width: '40%',
  },
}));

const RouteOptions = ({ index, route }) => {
  const { classes } = useStyles();
  const dispatch = useDispatch();
  const [expand, setExpand] = useState(false);

  const handleAttributeChange = (attribute, value) => {
    dispatch(missionActions.updateRouteAttribute({ index, attribute, value }));
  };

  return (
    <Accordion expanded={expand} onChange={() => setExpand(!expand)}>
      <AccordionSummary expandIcon={<ExpandMore />}>
        <Typography sx={{ width: '53%', flexShrink: 0 }}>Route Attributes</Typography>
      </AccordionSummary>
      <AccordionDetails className={classes.details}>
        {route.attributes && (
          <Fragment key={'fragment-route-atri-' + route.id}>
            <div>
              <Typography variant="subtitle1" className={classes.attribute}>
                Speed Mode
              </Typography>
              <div className={classes.attributeValue}>
                <SelectField
                  emptyValue={null}
                  fullWidth={true}
                  value={route.attributes.mode_speed || 0}
                  onChange={(e) => handleAttributeChange('mode_speed', e.target.value)}
                  endpoint={'/api/category/atributesparam/dji_M300/mode_speed'}
                  keyGetter={(it) => it.id}
                  titleGetter={(it) => it.name}
                />
              </div>
            </div>
            <div>
              <Typography variant="subtitle1" className={classes.attribute}>
                Speed idle (m/s):
              </Typography>
              <TextField
                fullWidth
                required
                type="number"
                className={classes.attributeValue}
                value={route.attributes.idle_vel || 1.85}
                onChange={(e) => handleAttributeChange('idle_vel', +e.target.value)}
              />
            </div>
            <div>
              <Typography variant="subtitle1" className={classes.attribute}>
                Speed MAX (m/s):
              </Typography>
              <TextField
                fullWidth
                required
                type="number"
                className={classes.attributeValue}
                value={route.attributes.max_vel || 12}
                onChange={(e) => handleAttributeChange('max_vel', +e.target.value)}
              />
            </div>
            <div>
              <Typography variant="subtitle1" className={classes.attribute}>
                landing mode
              </Typography>
              <div className={classes.attributeValue}>
                <SelectField
                  emptyValue={null}
                  fullWidth={true}
                  value={route.attributes.mode_landing || 0}
                  onChange={(e) => handleAttributeChange('mode_landing', e.target.value)}
                  endpoint={'/api/category/atributesparam/dji_M300/mode_landing'}
                  keyGetter={(it) => it.id}
                  titleGetter={(it) => it.name}
                />
              </div>
            </div>
            <div>
              <Typography variant="subtitle1" className={classes.attribute}>
                Yaw mode
              </Typography>
              <div className={classes.attributeValue}>
                <SelectField
                  emptyValue={null}
                  fullWidth={true}
                  value={route.attributes.mode_yaw || 0}
                  onChange={(e) => handleAttributeChange('mode_yaw', e.target.value)}
                  endpoint={'/api/category/atributesparam/dji_M300/mode_yaw'}
                  keyGetter={(it) => it.id}
                  titleGetter={(it) => it.name}
                />
              </div>
            </div>
            <div>
              <Typography variant="subtitle1" className={classes.attribute}>
                Gimbal mode
              </Typography>
              <div className={classes.attributeValue}>
                <SelectField
                  emptyValue={null}
                  fullWidth={true}
                  value={route.attributes.mode_gimbal || 0}
                  onChange={(e) => handleAttributeChange('mode_gimbal', e.target.value)}
                  endpoint={'/api/category/atributesparam/dji_M300/mode_gimbal'}
                  keyGetter={(it) => it.id}
                  titleGetter={(it) => it.name}
                />
              </div>
            </div>
            <div>
              <Typography variant="subtitle1" className={classes.attribute}>
                Trace mode:
              </Typography>
              <div className={classes.attributeValue}>
                <SelectField
                  emptyValue={null}
                  fullWidth={true}
                  value={route.attributes.mode_trace || 0}
                  onChange={(e) => handleAttributeChange('mode_trace', e.target.value)}
                  endpoint={'/api/category/atributesparam/dji_M300/mode_trace'}
                  keyGetter={(it) => it.id}
                  titleGetter={(it) => it.name}
                />
              </div>
            </div>
          </Fragment>
        )}
      </AccordionDetails>
    </Accordion>
  );
};

const RouteRoutesList = ({
  index,
  route,
  expanded_route,
  setExpanded_route,
  expand_wp,
  setExpand_wp,
}) => {
  const { classes } = useStyles();
  const dispatch = useDispatch();
  const devices = useSelector((state) => state.devices.items);
  const positions = useSelector((state) => state.session.positions);
  const idleVel = useSelector((state) => state.mission.route[index]?.attributes?.idle_vel);
  const [routeUAV, setRouteUAV] = useState(null);

  useEffect(() => {
    const myDevice = Object.values(devices).find((device) => device.name === route.uav);
    if (myDevice) {
      setRouteUAV(myDevice.id);
      // Update uav_type when device is found
      if (route.uav_type !== myDevice.category) {
        dispatch(missionActions.updateRoute({ index, field: 'uav_type', value: myDevice.category }));
      }
    } else {
      setRouteUAV(null);
    }
  }, [route.uav, devices, index, route.uav_type, dispatch]);

  const handleAddWaypoint = (index_route, index_wp) => {
    let center = map.getCenter();
    const mywp = { pos: [center.lat, center.lng, 5], action: {} };
    dispatch(missionActions.addWaypoint({ routeIndex: index_route, waypoint: mywp, insertAt: index_wp }));
  };

  const handleRecordWp = (index_route) => {
    if (positions[routeUAV]) {
      let altitude = positions[routeUAV].altitude;
      if (positions[routeUAV].attributes?.home) {
        altitude = altitude - positions[routeUAV].attributes.home[2];
      }

      const mywp = {
        pos: [positions[routeUAV].latitude, positions[routeUAV].longitude, altitude],
        action: { yaw: 0, gimbal: 0 },
      };

      if (positions[routeUAV].course !== undefined) {
        mywp.action.yaw =
          +Number(positions[routeUAV].course).toFixed(2) <= 180
            ? +Number(positions[routeUAV].course).toFixed(2)
            : -360 + +Number(positions[routeUAV].course).toFixed(2);
      }
      if (positions[routeUAV].attributes?.gimbal) {
        mywp.action.gimbal = Number(positions[routeUAV].attributes.gimbal[0]);
      }

      dispatch(missionActions.addWaypoint({ routeIndex: index_route, waypoint: mywp }));
    }
  };

  const handleRemoveRoute = (index_route) => {
    dispatch(missionActions.deleteRoute(index_route));
  };

  const handleRouteFieldChange = (field, value) => {
    dispatch(missionActions.updateRoute({ index, field, value }));
  };

  const handleChange_route = (panel) => (event, isExpanded) => {
    setExpanded_route(isExpanded ? panel : false);
  };

  return (
    <Accordion expanded={expanded_route === 'Rute ' + index} onChange={handleChange_route('Rute ' + index)}>
      <AccordionSummary expandIcon={<ExpandMore />}>
        <Typography
          sx={{
            width: '33%',
            flexShrink: 0,
            color: palette.colors_devices[route.id],
          }}
        >
          {'Rute ' + index}
        </Typography>
        <Typography sx={{ color: 'text.secondary' }}>{route.name + '- ' + route.uav}</Typography>
        <IconButton sx={{ py: 0, pr: 2, marginLeft: 'auto' }} onClick={() => handleRemoveRoute(index)}>
          <DeleteIcon />
        </IconButton>
      </AccordionSummary>
      <AccordionDetails className={classes.details}>
        {expanded_route === 'Rute ' + index && (
          <Fragment>
            <TextField
              required
              label="Route Name"
              variant="standard"
              value={route.name || ''}
              onChange={(e) => handleRouteFieldChange('name', e.target.value)}
            />

            <TextField
              required
              label="UAV id"
              variant="standard"
              value={route.uav || 'uav_'}
              onChange={(e) => handleRouteFieldChange('uav', e.target.value)}
            />

            <SelectField
              emptyValue={null}
              value={route.uav_type || ''}
              onChange={(e) => handleRouteFieldChange('uav_type', e.target.value)}
              endpoint="/api/category"
              keyGetter={(it) => it}
              titleGetter={(it) => it}
              label={'Type UAV mission'}
              style={{ display: 'inline', width: '200px' }}
            />

            <RouteOptions index={index} route={route} />

            <Typography variant="subtitle1">Waypoints</Typography>
            {route.wp.map((waypoint, index_wp) => (
              <WaypointRouteList
                key={`wp-${index}-${index_wp}`}
                indexWp={index_wp}
                routeIndex={index}
                waypoint={waypoint}
                idleVel={idleVel}
                expandWp={expand_wp}
                setExpandWp={setExpand_wp}
                onAddWaypoint={handleAddWaypoint}
              />
            ))}

            <Box textAlign="center">
              <Button
                variant="contained"
                size="large"
                sx={{ width: '60%', flexShrink: 0 }}
                style={{ marginTop: '15px' }}
                onClick={() => handleAddWaypoint(index, -1)}
              >
                Add Waypoint
              </Button>
              <Button
                variant="contained"
                color="secondary"
                size="large"
                disabled={routeUAV == null}
                sx={{ width: '30%', flexShrink: 0 }}
                style={{ marginTop: '15px' }}
                onClick={() => handleRecordWp(index)}
              >
                Record
              </Button>
            </Box>
          </Fragment>
        )}
      </AccordionDetails>
    </Accordion>
  );
};

export default RouteRoutesList;

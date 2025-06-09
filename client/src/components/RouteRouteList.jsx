import React, { Fragment, useEffect, useState } from 'react';
import { useDispatch, useSelector } from 'react-redux';
import { makeStyles } from 'tss-react/mui';

import {
  Divider,
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

// https://dev.to/shareef/how-to-work-with-arrays-in-reactjs-usestate-4cmi

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

const RouteOptions = ({ mission, setmission, index, route, expand, setExpand }) => {
  const { classes } = useStyles();
  return (
    <Accordion expanded={expand} onChange={setExpand(true)}>
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
                  value={route.attributes.mode_speed ? route.attributes.mode_speed : 0}
                  onChange={(e) =>
                    setmission({
                      ...mission,
                      route: mission.route.map((rt) => {
                        let copiedrt = JSON.parse(JSON.stringify(rt));
                        rt === mission.route[index]
                          ? (copiedrt.attributes.mode_speed = e.target.value)
                          : (copiedrt = rt);
                        return copiedrt;
                      }),
                    })
                  }
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
                value={route.attributes.idle_vel ? route.attributes.idle_vel : 1.85}
                onChange={(e) =>
                  setmission({
                    ...mission,
                    route: mission.route.map((rt) => {
                      let copiedrt = JSON.parse(JSON.stringify(rt));
                      rt === mission.route[index] ? (copiedrt.attributes.idle_vel = +e.target.value) : (copiedrt = rt);
                      return copiedrt;
                    }),
                  })
                }
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
                value={route.attributes.max_vel ? route.attributes.max_vel : 12}
                onChange={(e) =>
                  setmission({
                    ...mission,
                    route: mission.route.map((rt) => {
                      let copiedrt = JSON.parse(JSON.stringify(rt));
                      rt == mission.route[index] ? (copiedrt.attributes.max_vel = +e.target.value) : (copiedrt = rt);
                      return copiedrt;
                    }),
                  })
                }
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
                  value={route.attributes.mode_landing ? route.attributes.mode_landing : 0}
                  onChange={(e) =>
                    setmission({
                      ...mission,
                      route: mission.route.map((rt) => {
                        let copiedrt = JSON.parse(JSON.stringify(rt));
                        rt === mission.route[index]
                          ? (copiedrt.attributes.mode_landing = e.target.value)
                          : (copiedrt = rt);
                        return copiedrt;
                      }),
                    })
                  }
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
                  value={route.attributes.mode_yaw ? route.attributes.mode_yaw : 0}
                  onChange={(e) =>
                    setmission({
                      ...mission,
                      route: mission.route.map((rt) => {
                        let copiedrt = JSON.parse(JSON.stringify(rt));
                        rt === mission.route[index] ? (copiedrt.attributes.mode_yaw = e.target.value) : (copiedrt = rt);
                        return copiedrt;
                      }),
                    })
                  }
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
                  value={route.attributes.mode_gimbal ? route.attributes.mode_gimbal : 0}
                  onChange={(e) =>
                    setmission({
                      ...mission,
                      route: mission.route.map((rt) => {
                        let copiedrt = JSON.parse(JSON.stringify(rt));
                        rt === mission.route[index]
                          ? (copiedrt.attributes.mode_gimbal = e.target.value)
                          : (copiedrt = rt);
                        return copiedrt;
                      }),
                    })
                  }
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
                  value={route.attributes.mode_trace ? route.attributes.mode_trace : 0}
                  onChange={(e) =>
                    setmission({
                      ...mission,
                      route: mission.route.map((rt) => {
                        let copiedrt = JSON.parse(JSON.stringify(rt));
                        rt === mission.route[index]
                          ? (copiedrt.attributes.mode_trace = e.target.value)
                          : (copiedrt = rt);
                        return copiedrt;
                      }),
                    })
                  }
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
  mission,
  setmission,
  index,
  route,
  expanded_route,
  setExpanded_route,
  expand_wp,
  setExpand_wp,
}) => {
  const { classes } = useStyles();
  const devices = useSelector((state) => state.devices.items);
  const positions = useSelector((state) => state.session.positions);
  const [expandedRouteOptions, setExpandedRouteOptions] = useState(false);
  const [routeUAV, setRouteUAV] = useState(null);

  useEffect(() => {
    const myDevice = Object.values(devices).find((device) => device.name === route.uav);
    if (myDevice) {
      setRouteUAV(myDevice.id);
      setmission({
        ...mission,
        route: mission.route.map((rt, rtIndex) => (rtIndex == index ? { ...route, uav_type: myDevice.category } : rt)),
      });
    } else {
      setRouteUAV(null);
    }
  }, [route.uav, devices]);

  const AddnewWp = (index_route, index_wp) => {
    console.log('add new wp' + index_route);
    let auxroute = JSON.parse(JSON.stringify(mission.route));
    let center = map.getCenter(); // cuando index es 0 o -1
    if (index_wp > 0 && index_wp < auxroute[index_route]['wp'].length) {
      center.lat =
        (auxroute[index_route]['wp'][index_wp]['pos'][0] + auxroute[index_route]['wp'][index_wp + 1]['pos'][0]) / 2;
      center.lng =
        (auxroute[index_route]['wp'][index_wp]['pos'][1] + auxroute[index_route]['wp'][index_wp + 1]['pos'][1]) / 2;
    }

    let mywp = { pos: [center.lat, center.lng, 5], action: {} };
    index_wp < 0 ? auxroute[index_route].wp.push(mywp) : auxroute[index_route].wp.splice(index_wp + 1, 0, mywp);

    setmission({ ...mission, route: auxroute });
  };

  const recordWp = (index_route, index_wp) => {
    console.log('Record wp' + index_route);
    let auxroute = JSON.parse(JSON.stringify(mission.route));

    console.log(positions[routeUAV]);

    if (positions[routeUAV]) {
      let altitude = positions[routeUAV].altitude;
      if (positions[routeUAV].attributes.hasOwnProperty('home')) {
        altitude = altitude - positions[routeUAV].attributes.home[2];
        console.log('home');
        console.log(positions[routeUAV].attributes.home[2]);
      }
      console.log(altitude);
      let mywp = {
        pos: [positions[routeUAV].latitude, positions[routeUAV].longitude, altitude],
        action: { yaw: 0, gimbal: 0 },
      };
      if ([positions[routeUAV].hasOwnProperty('course')]) {
        mywp.action.yaw =
          +Number(positions[routeUAV].course).toFixed(2) <= 180
            ? +Number(positions[routeUAV].course).toFixed(2)
            : -360 + +Number(positions[routeUAV].course).toFixed(2);
      }
      if ([positions[routeUAV].attributes.hasOwnProperty('gimbal')]) {
        mywp.action.gimbal = Number(positions[routeUAV].attributes.gimbal[0]);
      }
      index_wp < 0 ? auxroute[index_route].wp.push(mywp) : auxroute[index_route].wp.splice(index_wp, 0, mywp);
      setmission({ ...mission, route: auxroute });
    }
  };
  const Removing_route = (index_route) => {
    console.log('remove route' + index_route);
    let auxroute = JSON.parse(JSON.stringify(mission.route));
    auxroute.splice(index_route, 1);
    //console.log(auxroute);
    setmission({ ...mission, route: auxroute });
  };

  const handleChange_route = (panel) => (event, isExpanded) => {
    setExpanded_route(isExpanded ? panel : false);
    setExpandedRouteOptions(false);
  };
  const handleChange_routeOptions = (panel) => (event, isExpanded) => {
    setExpandedRouteOptions(isExpanded ? panel : false);
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
        <IconButton sx={{ py: 0, pr: 2, marginLeft: 'auto' }} onClick={() => Removing_route(index)}>
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
              value={route.name ? route.name : ''}
              onChange={(e) =>
                setmission({
                  ...mission,
                  route: mission.route.map((rt) =>
                    rt == mission.route[index] ? { ...route, name: e.target.value } : rt
                  ),
                })
              }
            />

            <TextField
              required
              label="UAV id"
              variant="standard"
              value={route.uav ? route.uav : 'uav_'}
              onChange={(e) =>
                setmission({
                  ...mission,
                  route: mission.route.map((rt) =>
                    rt == mission.route[index] ? { ...route, uav: e.target.value } : rt
                  ),
                })
              }
            />

            <SelectField
              emptyValue={null}
              value={route.uav_type ? route.uav_type : ''}
              onChange={(e) =>
                setmission({
                  ...mission,
                  route: mission.route.map((rt) =>
                    rt == mission.route[index] ? { ...route, uav_type: e.target.value } : rt
                  ),
                })
              }
              endpoint="/api/category"
              keyGetter={(it) => it}
              titleGetter={(it) => it}
              label={'Type UAV mission'}
              style={{ display: 'inline', width: '200px' }}
            />
            <RouteOptions
              mission={mission}
              setmission={(e) => setmission(e)}
              index={index}
              route={route}
              expand={expandedRouteOptions}
              setExpand={handleChange_routeOptions}
            />

            <Typography variant="subtitle1">Waypoints</Typography>
            {React.Children.toArray(
              Object.values(route.wp).map((waypoint, index_wp, list_wp) => (
                <WaypointRouteList
                  mission={mission}
                  setmission={setmission}
                  indexWp={index_wp}
                  index={index}
                  waypoint={waypoint}
                  AddnewWp={AddnewWp}
                  expandWp={expand_wp}
                  setExpandWp={setExpand_wp}
                />
              ))
            )}

            <Box textAlign="center">
              <Button
                value={index}
                variant="contained"
                size="large"
                sx={{ width: '60%', flexShrink: 0 }}
                style={{ marginTop: '15px' }}
                onClick={(e) => AddnewWp(e.target.value, -1)}
              >
                Add Waypoint
              </Button>
              <Button
                value={index}
                variant="contained"
                color="secondary"
                size="large"
                disabled={routeUAV == null}
                sx={{ width: '30%', flexShrink: 0 }}
                style={{ marginTop: '15px' }}
                onClick={(e) => recordWp(e.target.value, -1)}
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

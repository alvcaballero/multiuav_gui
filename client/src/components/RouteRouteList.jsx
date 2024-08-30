import React, { Fragment, useEffect, useState } from 'react';
import { useDispatch, useSelector } from 'react-redux';
import makeStyles from '@mui/styles/makeStyles';
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
import { missionActions } from '../store'; // here update device action with position of uav for update in map
import palette from '../common/palette';
import { map } from '../Mapview/MapView';
import WaypointRouteList from './WaypointRouteList';

// https://dev.to/shareef/how-to-work-with-arrays-in-reactjs-usestate-4cmi

const useStyles = makeStyles((theme) => ({
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
  const classes = useStyles();
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
                      rt == mission.route[index] ? (copiedrt.attributes.idle_vel = +e.target.value) : (copiedrt = rt);
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

const RouteRoutesList = ({ mission, setmission, index, route, expanded_route, setExpanded_route, setScrool }) => {
  const classes = useStyles();
  const selectwp = useSelector((state) => state.mission.selectpoint);
  //const [open_routes, setOpen_routes] = useState(true);
  const [expandedRouteOptions, setExpandedRouteOptions] = useState(false);
  const [expanded_wp, setExpanded_wp] = useState(false);

  useEffect(() => {
    //console.log(expanded_route);
    setExpanded_route('Rute ' + selectwp.route_id);
    setExpanded_wp('wp ' + selectwp.id);
    setScrool(500 + selectwp.route_id * 50 + selectwp.id * 50);
  }, [selectwp]);

  const AddnewWp = (index_route, index_wp) => {
    console.log('add new wp' + index_route);
    let auxroute = JSON.parse(JSON.stringify(mission.route));
    let center = map.getCenter(); // cuando index es 0 o -1
    if (index_wp > 0) {
      center.lat =
        (auxroute[index_route]['wp'][index_wp]['pos'][0] + auxroute[index_route]['wp'][index_wp - 1]['pos'][0]) / 2;
      center.lng =
        (auxroute[index_route]['wp'][index_wp]['pos'][1] + auxroute[index_route]['wp'][index_wp - 1]['pos'][1]) / 2;
    } //console.log(center);

    if (index_wp < 0) {
      auxroute[index_route].wp.push({
        pos: [center.lat, center.lng, 5],
        action: {},
      });
    } else {
      auxroute[index_route].wp.splice(index_wp, 0, {
        pos: [center.lat, center.lng, 5],
        action: {},
      });
    }

    setmission({ ...mission, route: auxroute });
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
  const handleChange_wp = (panel) => (event, isExpanded) => {
    setExpanded_wp(isExpanded ? panel : false);
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
              setmission={setmission}
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
                  index_wp={index_wp}
                  index={index}
                  waypoint={waypoint}
                  AddnewWp={AddnewWp}
                  expand_wp={expanded_wp}
                />
              ))
            )}

            <Box textAlign="center">
              <Button
                value={index}
                variant="contained"
                size="large"
                sx={{ width: '80%', flexShrink: 0 }}
                style={{ marginTop: '15px' }}
                onClick={(e) => AddnewWp(e.target.value, -1)}
              >
                Add new Waypoint
              </Button>
            </Box>
          </Fragment>
        )}
      </AccordionDetails>
    </Accordion>
  );
};

export default RouteRoutesList;

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
import { colors } from '../Mapview/preloadImages';
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
  attributeName: {
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

const RoutesList = ({ mission, setmission, setScrool }) => {
  const classes = useStyles();
  const Mission_route = useSelector((state) => state.mission);
  const selectwp = useSelector((state) => state.mission.selectpoint);
  const dispatch = useDispatch();
  const [init, setinit] = useState(false);
  //const [open_routes, setOpen_routes] = useState(true);
  const [expanded_route, setExpanded_route] = useState(false);
  const [expanded_wp, setExpanded_wp] = useState(false);
  const [add_mission, setaddMission] = useState(true);

  useEffect(() => {
    setmission(Mission_route);
    //console.log("otro use effect");
    Mission_route.route.length > 0 ? setaddMission(false) : setaddMission(true);
    setinit(true);
  }, [Mission_route]);

  useEffect(() => {
    if (init) {
      console.log('update mission');
      //console.log(mission);
      dispatch(missionActions.reloadMission(mission.route));
    }
    mission.route.length > 0 ? setaddMission(false) : setaddMission(true);
  }, [mission.route]);

  useEffect(() => {
    if (init) {
      dispatch(
        missionActions.reloadName({
          name: mission.name,
          description: mission.description,
        })
      );
    }
  }, [mission.name, mission.description]);

  useEffect(() => {
    //console.log(expanded_route);
    setExpanded_route('Rute ' + selectwp.route_id);
    setExpanded_wp('wp ' + selectwp.id);
    setScrool(500 + selectwp.route_id * 50 + selectwp.id * 50);
  }, [selectwp]);

  const AddnewMission = () => {
    let auxmission = { name: 'new Mission', description: '', route: [] };
    //auxmission.route.push({ name: "", uav: "", id: 0, attributes: {}, wp: [] });
    setmission(auxmission);
    setaddMission(false);
    AddnewRoute();
    console.log('add new mission');
  };
  const AddnewRoute = () => {
    let auxroute = [...mission.route];
    let myid = auxroute.length > 0 ? +auxroute.slice(-1)[0].id + +1 : 0;
    let initAtributes = {
      max_vel: 12,
      idle_vel: 3,
      mode_yaw: 2,
      mode_gimbal: 0,
      mode_trace: 0,
      mode_landing: 2,
    };
    auxroute.push({
      name: '',
      uav: '',
      id: myid,
      attributes: initAtributes,
      wp: [],
    });
    setmission({ ...mission, route: auxroute });
  };
  const AddnewWp = (index_route, index_wp) => {
    console.log('add new wp' + index_route);
    let auxroute = JSON.parse(JSON.stringify(mission.route));
    let center = map.getCenter(); // cuando index es 0 o -1
    if (index_wp > 0) {
      center.lat =
        (auxroute[index_route]['wp'][index_wp]['pos'][0] +
          auxroute[index_route]['wp'][index_wp - 1]['pos'][0]) /
        2;
      center.lng =
        (auxroute[index_route]['wp'][index_wp]['pos'][1] +
          auxroute[index_route]['wp'][index_wp - 1]['pos'][1]) /
        2;
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
  };
  const handleChange_wp = (panel) => (event, isExpanded) => {
    setExpanded_wp(isExpanded ? panel : false);
  };

  return (
    <Fragment>
      {add_mission ? (
        <Box textAlign='center'>
          <Button
            variant='contained'
            size='large'
            sx={{ width: '80%', flexShrink: 0 }}
            style={{ marginTop: '15px' }}
            onClick={AddnewMission}
          >
            Create New Mission
          </Button>
        </Box>
      ) : (
        <Fragment>
          <Box
            style={{
              display: 'flex',
              gap: '10px',
              flexDirection: 'column',
              margin: '20px',
            }}
          >
            <TextField
              required
              label='Name Mission'
              variant='standard'
              value={mission.name ? mission.name : ' '}
              onChange={(event) => setmission({ ...mission, name: event.target.value })}
            />
            <TextField
              required
              label='Description of mission'
              variant='standard'
              value={mission.description ? mission.description : ' '}
              onChange={(event) => setmission({ ...mission, description: event.target.value })}
            />
            {React.Children.toArray(
              Object.values(mission.route).map((item_route, index, list) => (
                <Fragment key={'fragment-route-' + index}>
                  <Accordion
                    expanded={expanded_route === 'Rute ' + index}
                    onChange={handleChange_route('Rute ' + index)}
                  >
                    <AccordionSummary expandIcon={<ExpandMore />}>
                      <Typography
                        sx={{
                          width: '33%',
                          flexShrink: 0,
                          color: colors[item_route.id],
                        }}
                      >
                        {'Rute ' + index}
                      </Typography>
                      <Typography sx={{ color: 'text.secondary' }}>
                        {item_route.name + '- ' + item_route.uav}
                      </Typography>
                      <IconButton
                        sx={{ py: 0, pr: 2, marginLeft: 'auto' }}
                        onClick={() => Removing_route(index)}
                      >
                        <DeleteIcon />
                      </IconButton>
                    </AccordionSummary>
                    <AccordionDetails className={classes.details}>
                      <TextField
                        required
                        label='Route Name'
                        variant='standard'
                        value={item_route.name ? item_route.name : ''}
                        onChange={(e) =>
                          setmission({
                            ...mission,
                            route: mission.route.map((rt) =>
                              rt == mission.route[index]
                                ? { ...item_route, name: e.target.value }
                                : rt
                            ),
                          })
                        }
                      />

                      <TextField
                        required
                        label='UAV id'
                        variant='standard'
                        value={item_route.uav ? item_route.uav : 'uav_'}
                        onChange={(e) =>
                          setmission({
                            ...mission,
                            route: mission.route.map((rt) =>
                              rt == mission.route[index]
                                ? { ...item_route, uav: e.target.value }
                                : rt
                            ),
                          })
                        }
                      />

                      <SelectField
                        emptyValue={null}
                        value={item_route.uav_type ? item_route.uav_type : ''}
                        onChange={(e) =>
                          setmission({
                            ...mission,
                            route: mission.route.map((rt) =>
                              rt == mission.route[index]
                                ? { ...item_route, uav_type: e.target.value }
                                : rt
                            ),
                          })
                        }
                        endpoint='/api/category'
                        keyGetter={(it) => it}
                        titleGetter={(it) => it}
                        label={'Type UAV mission'}
                        style={{ display: 'inline', width: '200px' }}
                      />

                      <Accordion
                        expanded={expanded_wp === 'wp -' + index}
                        onChange={handleChange_wp('wp -' + index)}
                      >
                        <AccordionSummary expandIcon={<ExpandMore />}>
                          <Typography sx={{ width: '53%', flexShrink: 0 }}>
                            Route Attributes
                          </Typography>
                        </AccordionSummary>
                        <AccordionDetails className={classes.details}>
                          {item_route.attributes && (
                            <Fragment key={'fragment-route-atri-' + item_route.id}>
                              <div>
                                <Typography variant='subtitle1' className={classes.attributeName}>
                                  Speed Mode
                                </Typography>
                                <div className={classes.attributeValue}>
                                  <SelectField
                                    emptyValue={null}
                                    fullWidth={true}
                                    value={
                                      item_route.attributes.mode_speed
                                        ? item_route.attributes.mode_speed
                                        : 0
                                    }
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
                                <Typography variant='subtitle1' className={classes.attributeName}>
                                  Speed idle (m/s):
                                </Typography>
                                <TextField
                                  fullWidth
                                  required
                                  type='number'
                                  className={classes.attributeValue}
                                  value={
                                    item_route.attributes.idle_vel
                                      ? item_route.attributes.idle_vel
                                      : 1.85
                                  }
                                  onChange={(e) =>
                                    setmission({
                                      ...mission,
                                      route: mission.route.map((rt) => {
                                        let copiedrt = JSON.parse(JSON.stringify(rt));
                                        rt == mission.route[index]
                                          ? (copiedrt.attributes.idle_vel = +e.target.value)
                                          : (copiedrt = rt);
                                        return copiedrt;
                                      }),
                                    })
                                  }
                                />
                              </div>
                              <div>
                                <Typography variant='subtitle1' className={classes.attributeName}>
                                  Speed MAX (m/s):
                                </Typography>
                                <TextField
                                  fullWidth
                                  required
                                  type='number'
                                  className={classes.attributeValue}
                                  value={
                                    item_route.attributes.max_vel
                                      ? item_route.attributes.max_vel
                                      : 12
                                  }
                                  onChange={(e) =>
                                    setmission({
                                      ...mission,
                                      route: mission.route.map((rt) => {
                                        let copiedrt = JSON.parse(JSON.stringify(rt));
                                        rt == mission.route[index]
                                          ? (copiedrt.attributes.max_vel = +e.target.value)
                                          : (copiedrt = rt);
                                        return copiedrt;
                                      }),
                                    })
                                  }
                                />
                              </div>
                              <div>
                                <Typography variant='subtitle1' className={classes.attributeName}>
                                  landing mode
                                </Typography>
                                <div className={classes.attributeValue}>
                                  <SelectField
                                    emptyValue={null}
                                    fullWidth={true}
                                    value={
                                      item_route.attributes.mode_landing
                                        ? item_route.attributes.mode_landing
                                        : 0
                                    }
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
                                <Typography variant='subtitle1' className={classes.attributeName}>
                                  Yaw mode
                                </Typography>
                                <div className={classes.attributeValue}>
                                  <SelectField
                                    emptyValue={null}
                                    fullWidth={true}
                                    value={
                                      item_route.attributes.mode_yaw
                                        ? item_route.attributes.mode_yaw
                                        : 0
                                    }
                                    onChange={(e) =>
                                      setmission({
                                        ...mission,
                                        route: mission.route.map((rt) => {
                                          let copiedrt = JSON.parse(JSON.stringify(rt));
                                          rt === mission.route[index]
                                            ? (copiedrt.attributes.mode_yaw = e.target.value)
                                            : (copiedrt = rt);
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
                                <Typography variant='subtitle1' className={classes.attributeName}>
                                  {' '}
                                  Gimbal mode{' '}
                                </Typography>
                                <div className={classes.attributeValue}>
                                  <SelectField
                                    emptyValue={null}
                                    fullWidth={true}
                                    value={
                                      item_route.attributes.mode_gimbal
                                        ? item_route.attributes.mode_gimbal
                                        : 0
                                    }
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
                                <Typography variant='subtitle1' className={classes.attributeName}>
                                  {' '}
                                  Trace mode:{' '}
                                </Typography>
                                <div className={classes.attributeValue}>
                                  <SelectField
                                    emptyValue={null}
                                    fullWidth={true}
                                    value={
                                      item_route.attributes.mode_trace
                                        ? item_route.attributes.mode_trace
                                        : 0
                                    }
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

                      <Typography variant='subtitle1'>Waypoints</Typography>
                      {React.Children.toArray(
                        Object.values(item_route.wp).map((waypoint, index_wp, list_wp) => (
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

                      <Box textAlign='center'>
                        <Button
                          value={index}
                          variant='contained'
                          size='large'
                          sx={{ width: '80%', flexShrink: 0 }}
                          style={{ marginTop: '15px' }}
                          onClick={(e) => AddnewWp(e.target.value, -1)}
                        >
                          Add new Waypoint
                        </Button>
                      </Box>
                    </AccordionDetails>
                  </Accordion>
                  {index < list.length - 1 ? <Divider /> : null}
                </Fragment>
              ))
            )}
            <Box textAlign='center'>
              <Button
                variant='contained'
                size='large'
                sx={{ width: '80%', flexShrink: 0 }}
                style={{ marginTop: '15px' }}
                onClick={AddnewRoute}
              >
                Add new Route
              </Button>
            </Box>
          </Box>
        </Fragment>
      )}
    </Fragment>
  );
};

export default RoutesList;

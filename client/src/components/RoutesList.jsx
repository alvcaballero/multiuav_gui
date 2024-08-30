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
import RouteRoutesList from './RouteRouteList';

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
    console.log('render Router list');
  }, []);

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
  };
  const handleChange_wp = (panel) => (event, isExpanded) => {
    setExpanded_wp(isExpanded ? panel : false);
  };

  return (
    <Fragment>
      {add_mission ? (
        <Box textAlign="center">
          <Button
            variant="contained"
            size="large"
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
              label="Name Mission"
              variant="standard"
              value={mission.name ? mission.name : ' '}
              onChange={(event) => setmission({ ...mission, name: event.target.value })}
            />
            <TextField
              required
              label="Description of mission"
              variant="standard"
              value={mission.description ? mission.description : ' '}
              onChange={(event) => setmission({ ...mission, description: event.target.value })}
            />
            {React.Children.toArray(
              Object.values(mission.route).map((item_route, index, list) => (
                <RouteRoutesList
                  mission={mission}
                  setmission={setmission}
                  index={index}
                  route={item_route}
                  AddnewWp={AddnewWp}
                  expanded_route={expanded_route}
                  expand_wp={expanded_wp}
                />
              ))
            )}
            <Box textAlign="center">
              <Button
                variant="contained"
                size="large"
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

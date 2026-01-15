import React, { Fragment, useEffect, useState } from 'react';
import { useDispatch, useSelector } from 'react-redux';
import { makeStyles } from 'tss-react/mui';

import {
  Divider,
  Box,
  Button,
  TextField,
} from '@mui/material';

import { missionActions } from '../store';
import RouteRoutesList from './RouteRouteList';

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

const RoutesList = ({ setScrool, NoEdit = false }) => {
  const { classes } = useStyles();
  const dispatch = useDispatch();

  // Read directly from Redux - single source of truth
  const mission = useSelector((state) => state.mission);
  const selectwp = useSelector((state) => state.mission.selectpoint);

  const [expanded_route, setExpanded_route] = useState(false);
  const [expanded_wp, setExpanded_wp] = useState(false);

  const hasMission = mission.route.length > 0;

  useEffect(() => {
    // Scroll to selected waypoint
    if (selectwp.id >= 0) {
      setExpanded_route('Rute ' + selectwp.route_id);
      setExpanded_wp('wp ' + selectwp.id);
      setScrool(500 + selectwp.route_id * 50 + selectwp.id * 50);
    }
  }, [selectwp, setScrool]);

  const handleCreateNewMission = () => {
    dispatch(missionActions.createNewMission({ name: 'new Mission' }));
    dispatch(missionActions.addRoute());
  };

  const handleAddNewRoute = () => {
    dispatch(missionActions.addRoute());
  };

  const handleNameChange = (event) => {
    dispatch(missionActions.updateName(event.target.value));
  };

  const handleDescriptionChange = (event) => {
    dispatch(missionActions.updateDescription(event.target.value));
  };

  return (
    <Fragment>
      {!hasMission ? (
        <Box textAlign="center">
          <Button
            variant="contained"
            size="large"
            sx={{ width: '80%', flexShrink: 0 }}
            style={{ marginTop: '15px' }}
            onClick={handleCreateNewMission}
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
              value={mission.name || ''}
              onChange={handleNameChange}
            />
            <TextField
              required
              label="Description of mission"
              variant="standard"
              value={mission.description || ''}
              onChange={handleDescriptionChange}
            />
            {mission.route.map((item_route, index, list) => (
              <Fragment key={'fragment-route-' + item_route.id}>
                <RouteRoutesList
                  index={index}
                  route={item_route}
                  expanded_route={expanded_route}
                  setExpanded_route={setExpanded_route}
                  expand_wp={expanded_wp}
                  setExpand_wp={setExpanded_wp}
                />
                {index < list.length - 1 ? <Divider /> : null}
              </Fragment>
            ))}
            {!NoEdit && (
              <Box textAlign="center">
                <Button
                  variant="contained"
                  size="large"
                  sx={{ width: '80%', flexShrink: 0 }}
                  style={{ marginTop: '15px' }}
                  onClick={handleAddNewRoute}
                >
                  Add new Route
                </Button>
              </Box>
            )}
          </Box>
        </Fragment>
      )}
    </Fragment>
  );
};

export default RoutesList;

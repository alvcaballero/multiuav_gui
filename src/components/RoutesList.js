import React, { Fragment } from 'react';
import { useDispatch, useSelector } from 'react-redux';
import makeStyles from '@mui/styles/makeStyles';
import {
  Divider, List, ListItemButton, ListItemText,
} from '@mui/material';

//import { geofencesActions } from '../store';
//import CollectionActions from '../settings/components/CollectionActions';
//import { useCatchCallback } from '../reactHelper';


const useStyles = makeStyles(() => ({
    list: {
      maxHeight: '100%',
      overflow: 'auto',
    },
    icon: {
      width: '25px',
      height: '25px',
      filter: 'brightness(0) invert(1)',
    },
  }));

const RoutesList = ({ onGeofenceSelected }) => {
    const classes = useStyles();
    const items = useSelector((state) => state.mission.route);

  return (
    <List className={classes.list}>
    {Object.values(items).map((item, index, list) => (
      <Fragment key={item.id}>
        <ListItemButton key={item.id} onClick={() => onGeofenceSelected(item.id)}>
          <ListItemText primary={item.name} />
        </ListItemButton>
        {index < list.length - 1 ? <Divider /> : null}
      </Fragment>
    ))}
  </List>
  )
}

export default RoutesList
import React, { Fragment, useEffect } from 'react';
import { useDispatch, useSelector } from 'react-redux';
import makeStyles from '@mui/styles/makeStyles';
import {
  Divider, List, ListItemButton, ListItemText,
} from '@mui/material';
import ArrowRight from '@mui/icons-material/ArrowRight';
import KeyboardArrowDown from '@mui/icons-material/KeyboardArrowDown';

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
  const [open, setOpen] = React.useState(true);
    const classes = useStyles();
    const items = useSelector((state) => state.mission.route);
    useEffect(()=>{

    },items)

  return (
    <List className={classes.list}>
    {Object.values(items).map((item, index, list) => (
      <Fragment key={item.id}>
        <ListItemButton key={item.id} onClick={() => setOpen(!open)}>
          <ListItemText primary={"Ruta "+item.name+"         "+"uav: "+item.uav} />
          <KeyboardArrowDown
                  sx={{
                    mr: -1,
                    opacity: 0,
                    transform: open ? 'rotate(-180deg)' : 'rotate(0)',
                    transition: '0.2s',
                  }}
                />
        </ListItemButton>
        {open &&
        Object.values(item.wp).map((waypoint, index_wp, list_wp) => (
            <ListItemButton
            key={index_wp}
            sx={{ py: 0, minHeight: 32, color: 'rgba(0,0,0,.8)'}}
          >
                    <ListItemText
                      primary={"WP"+index_wp+"-"+waypoint.pos[0]}
                      primaryTypographyProps={{ fontSize: 14, fontWeight: 'medium' }}
                    />
                    </ListItemButton>
        ))}
        {index < list.length - 1 ? <Divider /> : null}
      </Fragment>
    ))}
  </List>
  )
}

export default RoutesList
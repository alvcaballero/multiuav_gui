import React, { Fragment, useEffect } from 'react';
import { useDispatch, useSelector } from 'react-redux';
import makeStyles from '@mui/styles/makeStyles';
import {
  Divider, 
  List, 
  ListItemButton, 
  ListItemText,
  Collapse,
  TextField,
  Accordion,
  AccordionSummary,
  AccordionDetails,
} from '@mui/material';
import ArrowRight from '@mui/icons-material/ArrowRight';
import KeyboardArrowDown from '@mui/icons-material/KeyboardArrowDown';
import ExpandLess from '@mui/icons-material/ExpandLess';
import ExpandMore from '@mui/icons-material/ExpandMore';
//import { geofencesActions } from '../store';
//import CollectionActions from '../settings/components/CollectionActions';
//import { useCatchCallback } from '../reactHelper';
// https://dev.to/shareef/how-to-work-with-arrays-in-reactjs-usestate-4cmi

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
  const [open_routes, setOpen_routes] = React.useState(true);
    const classes = useStyles();
    const items = useSelector((state) => state.mission.route);
    useEffect(()=>{

    },[items])

  return (
    <div>
    <div >
      <a>Mission Name</a>
    <TextField id="outlined-basic" label="Outlined" variant="outlined" />
    </div>
    <List className={classes.list}>
    {Object.values(items).map((item_route, index, list) => (
      <Fragment key={item_route.id}>
        <ListItemButton key={item_route.id} onClick={() => setOpen_routes(!open_routes)}>
          <ListItemText primary={"Ruta "+item_route.name+"         "+"uav: "+item_route.uav} />
          {open_routes ? <ExpandLess /> : <ExpandMore />}
        </ListItemButton>
        {open_routes &&
          Object.values(item.wp).map((waypoint, index_wp, list_wp) => (
            <Collapse in={open_routes} timeout="auto" unmountOnExit>
              <List component="div" disablePadding>
              <ListItemButton
              key={index_wp}
              sx={{ py: 0, minHeight: 32, color: 'rgba(0,0,0,.8)'}}
            >
                      <ListItemText
                        primary={"WP"+index_wp+"-"+waypoint.pos[0]}
                        primaryTypographyProps={{ fontSize: 14, fontWeight: 'medium' }}
                      />
                      {open_routes ? <ExpandLess /> : <ExpandMore />}
              </ListItemButton>
              </List>
            </Collapse>
        ))}
        {index < list.length - 1 ? <Divider /> : null}
      </Fragment>
    ))}
  </List>
  </div>
  )
}

export default RoutesList
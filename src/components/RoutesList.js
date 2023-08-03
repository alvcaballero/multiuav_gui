import React, { Fragment, useEffect,useState } from 'react';
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
import Typography from '@mui/material/Typography';
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

    const classes = useStyles();
    const Mission_route = useSelector((state) => state.mission.route);

    const [items, setItems] = useState({});
    const [open_routes, setOpen_routes] = useState(true);
    const [expanded_route, setExpanded_route] = useState(false);
    const [expanded_wp, setExpanded_wp] = useState(false);

    useEffect(()=>{
      setItems(Mission_route)
    },[Mission_route])

    const handleChange_route = (panel) => (event, isExpanded) => {
      setExpanded_route(isExpanded ? panel : false);
    };
    const handleChange_wp = (panel) => (event, isExpanded) => {
      setExpanded_wp(isExpanded ? panel : false);
    };

  return (
    <div>
    <div >
      <a>Mission Name</a>
    <TextField id="outlined-basic" label="Outlined" variant="outlined" />
    </div>
    <List className={classes.list}>
    {Object.values(items).map((item_route, index, list) => (
      <Fragment key={item_route.id}>

      <Accordion expanded={expanded_route === ('Rute '+ index)} onChange={handleChange_route('Rute '+ index)} >
        <AccordionSummary
          expandIcon={<ExpandMore />}
          aria-controls="panel1bh-content"
          id="panel1bh-header"
        >
          <Typography sx={{ width: '33%', flexShrink: 0 }}> {'Rute '+ index}</Typography>
          <Typography sx={{ color: 'text.secondary' }}> {item_route.name+"- "+item_route.uav} </Typography>
        </AccordionSummary>
        <AccordionDetails>
        {Object.values(item_route.wp).map((waypoint, index_wp, list_wp) => (
              <Accordion expanded={expanded_wp === ('Rute '+ index_wp)} onChange={handleChange_wp('Rute '+ index_wp)}>
              <AccordionSummary
                expandIcon={<ExpandMore />}
                aria-controls="panel1bh-content"
                id="panel1bh-header"
              >
                <Typography sx={{ width: '33%', flexShrink: 0 }}> {'wp '+ index_wp}</Typography>
                <Typography sx={{ color: 'text.secondary' }}>I am an waypoint</Typography>
              </AccordionSummary>
              <AccordionDetails>
                <Typography>
                  Nulla facilisi. Phasellus sollicitudin nulla et quam mattis feugiat.
                  Aliquam eget maximus est, id dignissim quam.
                </Typography>
              </AccordionDetails>
            </Accordion>
        ))}

        </AccordionDetails>
      </Accordion>
        {index < list.length - 1 ? <Divider /> : null}
      </Fragment>
    ))}
  </List>
  </div>
  )
}

export default RoutesList
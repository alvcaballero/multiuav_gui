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
import BaseList from './BaseList';

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

const ElementList = ({ markers, setMarkers }) => {
  const classes = useStyles();

  const dispatch = useDispatch();
  const [init, setinit] = useState(false);
  //const [open_routes, setOpen_routes] = useState(true);
  const [expanded_route, setExpanded_route] = useState(false);
  const [expanded, setExpanded] = useState(false);
  const [BasesExist, setBasesExist] = useState(true);

  const setElement = (index, value) => {
    let auxMarkers = JSON.parse(JSON.stringify(markers));
    auxMarkers[index].items = value;
    setMarkers(auxMarkers);
  };
  const DeleteList = (index) => {
    let auxMarkers = JSON.parse(JSON.stringify(markers));
    auxMarkers.splice(index, 1);
    setMarkers(auxMarkers);
  };
  const AddList = () => {
    let auxMarkers = JSON.parse(JSON.stringify(markers));
    auxMarkers.push({ type: 'powerTower', name: 'Elements', linea: true, items: [] });
    setMarkers(auxMarkers);
  };
  const handleChange = (panel) => (event, isExpanded) => {
    setExpanded(isExpanded ? panel : false);
  };

  useEffect(() => {
    if (markers) {
      markers.length > 0 ? setBasesExist(false) : setBasesExist(true);
    }
    console.log(markers);
  }, [markers]);

  return (
    <Fragment>
      {BasesExist ? (
        <Box textAlign="center">
          <Button
            variant="contained"
            size="large"
            sx={{ width: '80%', flexShrink: 0 }}
            style={{ marginTop: '15px' }}
            onClick={AddList}
          >
            Create New Group
          </Button>
        </Box>
      ) : (
        <div className={classes.details}>
          {React.Children.toArray(
            Object.values(markers).map((base, index, list) => (
              <Accordion expanded={expanded === 'Elements ' + index} onChange={handleChange('Elements ' + index)}>
                <AccordionSummary expandIcon={<ExpandMore />}>
                  <Typography sx={{ width: '33%', flexShrink: 0 }}>{'Group ' + index}</Typography>
                  <IconButton sx={{ py: 0, pr: 2, marginLeft: 'auto' }} onClick={() => DeleteList(index)}>
                    <DeleteIcon />
                  </IconButton>
                </AccordionSummary>
                <AccordionDetails className={classes.details}>
                  {expanded === 'Elements ' + index && (
                    <Fragment>
                      <TextField required label="Name" variant="standard" value={base.name ? base.name : ''} />
                      <SelectField
                        emptyValue={null}
                        value={0}
                        data={[
                          { id: 0, name: 'Power Tower' },
                          { id: 1, name: 'wind turbine' },
                          { id: 2, name: 'Solar Panel' },
                        ]}
                        label="Type"
                        style={{ display: 'inline', width: '200px' }}
                      />
                      <BaseList markers={base.items} setMarkers={(value) => setElement(index, value)} type="Element" />
                    </Fragment>
                  )}
                </AccordionDetails>
              </Accordion>
            ))
          )}
          <Box textAlign="center">
            <Button
              variant="contained"
              size="large"
              sx={{ width: '80%', flexShrink: 0 }}
              style={{ marginTop: '15px' }}
              onClick={AddList}
            >
              Create List of Elements
            </Button>
          </Box>
        </div>
      )}
    </Fragment>
  );
};

export default ElementList;

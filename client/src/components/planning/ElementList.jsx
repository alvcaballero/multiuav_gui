import React, { Fragment, useEffect, useState } from 'react';
import { useDispatch } from 'react-redux';
import { useMarkerTypes } from '../../hooks/useMarkerTypes';
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
import SelectField from '../../shared/components/SelectField';
import DeleteIcon from '@mui/icons-material/Delete';
import BaseList from './BaseList';

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
  const { classes } = useStyles();

  const dispatch = useDispatch();
  const { types: markerTypes } = useMarkerTypes();
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
  const setElementName = (index, value) => {
    let auxMarkers = JSON.parse(JSON.stringify(markers));
    auxMarkers[index].name = value;
    setMarkers(auxMarkers);
  };
  const setElementDescription = (index, value) => {
    let auxMarkers = JSON.parse(JSON.stringify(markers));
    auxMarkers[index].description = value;
    setMarkers(auxMarkers);
  };
  const setElementType = (index, value) => {
    let auxMarkers = JSON.parse(JSON.stringify(markers));
    auxMarkers[index].type = value;
    setMarkers(auxMarkers);
  };
  const DeleteList = (index) => {
    let auxMarkers = JSON.parse(JSON.stringify(markers));
    auxMarkers.splice(index, 1);
    setMarkers(auxMarkers);
  };
  const AddList = () => {
    const defaultType = markerTypes[0]?.id || 'powerTower';
    let auxMarkers = JSON.parse(JSON.stringify(markers));
    auxMarkers.push({ type: defaultType, name: 'Elements', linea: true, items: [] });
    setMarkers(auxMarkers);
  };
  const handleChange = (panel) => (event, isExpanded) => {
    setExpanded(isExpanded ? panel : false);
  };

  useEffect(() => {
    if (markers) {
      markers.length > 0 ? setBasesExist(false) : setBasesExist(true);
    }
    //console.log(markers);
  }, [markers]);

  return (
    <Fragment>
      {BasesExist ? (
        <Box sx={{ textAlign: 'center' }}>
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
          {Object.values(markers).map((base, index) => (
            <Accordion
              key={index}
              expanded={expanded === 'Elements ' + index}
              onChange={handleChange('Elements ' + index)}
            >
              <AccordionSummary component="div" expandIcon={<ExpandMore />}>
                <Typography sx={{ flexGrow: 1, flexShrink: 1, minWidth: 0 }} noWrap>
                  {base.name || 'Group ' + index} {base.type && `(${base.type})`}
                </Typography>
                <IconButton sx={{ py: 0, pr: 0, flexShrink: 0 }} onClick={() => DeleteList(index)}>
                  <DeleteIcon />
                </IconButton>
              </AccordionSummary>
              <AccordionDetails className={classes.details}>
                {expanded === 'Elements ' + index && (
                  <Fragment>
                    <TextField
                      required
                      label="Name"
                      variant="standard"
                      value={base.name ? base.name : ''}
                      onChange={(e) => setElementName(index, e.target.value)}
                    />
                    <TextField
                      label="Description"
                      variant="standard"
                      multiline
                      rows={2}
                      fullWidth
                      value={base.description ? base.description : ''}
                      onChange={(e) => setElementDescription(index, e.target.value)}
                      placeholder="Add a description for this group..."
                    />
                    <SelectField
                      emptyValue={null}
                      label="Type"
                      value={base.type}
                      data={markerTypes}
                      onChange={(e) => setElementType(index, e.target.value)}
                      keyGetter={(item) => item.id}
                      titleGetter={(item) => item.name}
                      style={{ display: 'inline', width: '200px' }}
                    />
                    <BaseList
                      markers={base.items}
                      setMarkers={(value) => setElement(index, value)}
                      type="Element"
                      hasMapImage={markerTypes.find((t) => t.id === base.type)?.mapImage === true}
                    />
                  </Fragment>
                )}
              </AccordionDetails>
            </Accordion>
          ))}
          <Box sx={{ textAlign: 'center' }}>
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

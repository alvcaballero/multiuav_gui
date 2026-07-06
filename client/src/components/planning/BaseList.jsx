import React, { Fragment, useState } from 'react';
import { makeStyles } from 'tss-react/mui';

import {
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
import DeleteIcon from '@mui/icons-material/Delete';
import MyLocationIcon from '@mui/icons-material/MyLocation';
import { map } from '../../map/core/MapView';

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
  attribute: {
    display: 'inline-block',
    width: '58%',
  },
  actionValue: {
    display: 'inline-block',
    width: '40%',
  },
}));

const CORNERS = ['SW', 'SE', 'NE', 'NW'];

const BaseList = ({ markers, setMarkers, type = 'Base', hasMapImage = false }) => {
  const { classes } = useStyles();

  const [expanded, setExpanded] = useState(false);
  const basesExist = !markers || markers.length === 0;

  const handleChange = (panel) => (event, isExpanded) => {
    setExpanded(isExpanded ? panel : false);
  };
  const addNewElement = () => {
    let center = map.getCenter();
    let auxMarkers = JSON.parse(JSON.stringify(markers));
    auxMarkers.push({ latitude: center.lat, longitude: center.lng });
    setMarkers(auxMarkers, { meth: 'add', index: -1 });
  };
  const goToBase = (index) => {
    let base = markers[index];
    map.flyTo({ center: [base.longitude, base.latitude], zoom: Math.max(map.getZoom(), 18) });
  };
  const DeleteElement = (index) => {
    let auxMarkers = JSON.parse(JSON.stringify(markers));
    auxMarkers.splice(index, 1);
    setMarkers(auxMarkers, { meth: 'del', index: index });
  };

  const changeLat = (index, value) => {
    let auxMarkers = JSON.parse(JSON.stringify(markers));
    auxMarkers[index].latitude = value;
    setMarkers(auxMarkers, { meth: 'mod', index: index });
  };
  const changeLng = (index, value) => {
    let auxMarkers = JSON.parse(JSON.stringify(markers));
    auxMarkers[index].longitude = value;
    setMarkers(auxMarkers, { meth: 'mod', index: index });
  };
  const setName = (index, value) => {
    let auxMarkers = JSON.parse(JSON.stringify(markers));
    auxMarkers[index].name = value;
    setMarkers(auxMarkers, { meth: 'mod', index: index });
  };

  const setHeading = (index, value) => {
    let auxMarkers = JSON.parse(JSON.stringify(markers));
    auxMarkers[index].heading = Math.min(360, Math.max(0, +value));
    setMarkers(auxMarkers, { meth: 'mod', index: index });
  };

  const setCorner = (index, cornerIdx, axis, value) => {
    let auxMarkers = JSON.parse(JSON.stringify(markers));
    if (!auxMarkers[index].corners) {
      auxMarkers[index].corners = [
        [0, 0],
        [0, 0],
        [0, 0],
        [0, 0],
      ];
    }
    auxMarkers[index].corners[cornerIdx][axis === 'lng' ? 0 : 1] = +value;
    setMarkers(auxMarkers, { meth: 'mod', index: index });
  };

  return (
    <Fragment>
      {basesExist ? (
        <Box sx={{ textAlign: 'center' }}>
          <Button
            variant="contained"
            size="large"
            sx={{ width: '80%', flexShrink: 0 }}
            style={{ marginTop: '15px' }}
            onClick={addNewElement}
          >
            Create New Base
          </Button>
        </Box>
      ) : (
        <div className={classes.details}>
          {Object.values(markers).map((base, index) => (
            <Accordion
              key={index}
              expanded={expanded === 'wp ' + index}
              onChange={handleChange('wp ' + index)}
            >
              <AccordionSummary component="div" expandIcon={<ExpandMore />}>
                <Typography sx={{ flexGrow: 1, flexShrink: 1, minWidth: 0 }} noWrap>
                  {base.name || type + ' ' + index}
                </Typography>
                <IconButton
                  sx={{ py: 0, pr: 0, flexShrink: 0 }}
                  onClick={(e) => {
                    e.stopPropagation();
                    goToBase(index);
                  }}
                >
                  <MyLocationIcon />
                </IconButton>

                <IconButton
                  sx={{ py: 0, pr: 0, flexShrink: 0 }}
                  onClick={() => DeleteElement(index)}
                >
                  <DeleteIcon />
                </IconButton>
              </AccordionSummary>
              <AccordionDetails className={classes.details}>
                {expanded === 'wp ' + index && (
                  <Fragment>
                    <Box
                      component="form"
                      sx={{
                        '& .MuiTextField-root': { m: 1 },
                      }}
                    >
                      {type === 'Element' && (
                        <TextField
                          required
                          label="Name"
                          variant="standard"
                          value={base.name ? base.name : ''}
                          onChange={(e) => setName(index, e.target.value)}
                        />
                      )}
                      <div>
                        <Typography variant="subtitle1" style={{ display: 'inline' }}>
                          Position
                        </Typography>
                      </div>
                      <TextField
                        required
                        label="Latitude "
                        type="number"
                        sx={{ width: '15ch' }}
                        variant="standard"
                        slotProps={{ htmlInput: { maxLength: 8, step: 0.0001 } }}
                        defaultValue={base.latitude}
                        onBlur={(e) => {
                          changeLat(index, +e.target.value);
                        }}
                      />
                      <TextField
                        required
                        label="Longitud "
                        type="number"
                        variant="standard"
                        sx={{ width: '15ch' }}
                        slotProps={{ htmlInput: { maxLength: 8, step: 0.0001 } }}
                        defaultValue={base.longitude}
                        onBlur={(e) => {
                          changeLng(index, +e.target.value);
                        }}
                      />
                      {type === 'Element' && (
                        <TextField
                          label="Heading (° from N)"
                          type="number"
                          variant="standard"
                          sx={{ width: '18ch' }}
                          slotProps={{ htmlInput: { min: 0, max: 360, step: 1 } }}
                          defaultValue={base.heading ?? 0}
                          onBlur={(e) => setHeading(index, e.target.value)}
                        />
                      )}
                      {hasMapImage && (
                        <>
                          <div style={{ marginTop: '8px' }}>
                            <Typography variant="subtitle1">
                              Image corners (SW → SE → NE → NW)
                            </Typography>
                          </div>
                          {CORNERS.map((label, cornerIdx) => (
                            <div key={label}>
                              <Typography variant="caption">{label}</Typography>
                              <TextField
                                label="Lat"
                                type="number"
                                variant="standard"
                                sx={{ width: '15ch' }}
                                slotProps={{ htmlInput: { step: 0.0001 } }}
                                defaultValue={base.corners?.[cornerIdx]?.[1] ?? 0}
                                onBlur={(e) => setCorner(index, cornerIdx, 'lat', e.target.value)}
                              />
                              <TextField
                                label="Lng"
                                type="number"
                                variant="standard"
                                sx={{ width: '15ch' }}
                                slotProps={{ htmlInput: { step: 0.0001 } }}
                                defaultValue={base.corners?.[cornerIdx]?.[0] ?? 0}
                                onBlur={(e) => setCorner(index, cornerIdx, 'lng', e.target.value)}
                              />
                            </div>
                          ))}
                        </>
                      )}
                    </Box>
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
              onClick={addNewElement}
            >
              Create new {type}
            </Button>
          </Box>
        </div>
      )}
    </Fragment>
  );
};

export default BaseList;

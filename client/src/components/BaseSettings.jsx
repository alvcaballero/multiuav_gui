import React, { Fragment, useEffect, useState } from 'react';
import makeStyles from '@mui/styles/makeStyles';
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
import palette from '../common/palette';
import { map } from '../Mapview/MapView';

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
  attribute: {
    display: 'inline-block',
    width: '58%',
  },
  actionValue: {
    display: 'inline-block',
    width: '40%',
  },
}));

const BaseSettings = ({ markers, setMarkers, type = 'Base' }) => {
  const classes = useStyles();

  const [expanded, setExpanded] = useState(false);
  const [BasesExist, setBasesExist] = useState(true);

  const handleChange = (panel) => (event, isExpanded) => {
    setExpanded(isExpanded ? panel : false);
  };
  const AddNewElement = () => {
    let center = map.getCenter();
    let auxMarkers = JSON.parse(JSON.stringify(markers));
    auxMarkers.push({ latitude: center.lat, longitude: center.lng });
    setMarkers(auxMarkers);
  };
  const DeleteElement = (index) => {
    let auxMarkers = JSON.parse(JSON.stringify(markers));
    auxMarkers.splice(index, 1);
    setMarkers(auxMarkers);
  };

  const changeLat = (index, value) => {
    let auxMarkers = JSON.parse(JSON.stringify(markers));
    auxMarkers[index].latitude = value;
    setMarkers(auxMarkers);
  };
  const changeLng = (index, value) => {
    let auxMarkers = JSON.parse(JSON.stringify(markers));
    auxMarkers[index].longitude = value;
    setMarkers(auxMarkers);
  };
  useEffect(() => {
    if (markers) {
      markers.length > 0 ? setBasesExist(false) : setBasesExist(true);
    }
  }, [markers]);

  return (
    <Fragment>
      {BasesExist ? (
        <Box textAlign='center'>
          <Typography>dont exits base</Typography>
        </Box>
      ) : (
        <div className={classes.details}>
          {React.Children.toArray(
            Object.values(markers).map((base, index, list) => (
              <Accordion
                expanded={expanded === 'wp ' + index}
                onChange={handleChange('wp ' + index)}
              >
                <AccordionSummary expandIcon={<ExpandMore />}>
                  <Typography sx={{ width: '33%', flexShrink: 0 }}>{type + ' ' + index}</Typography>
                  <IconButton
                    sx={{ py: 0, pr: 2, marginLeft: 'auto' }}
                    onClick={() => DeleteElement(index)}
                  >
                    <DeleteIcon />
                  </IconButton>
                </AccordionSummary>
                <AccordionDetails className={classes.details}>
                  {expanded === 'wp ' + index && (
                    <Fragment>
                      <Box
                        component='form'
                        sx={{
                          '& .MuiTextField-root': { m: 1 },
                        }}
                      >
                        <div>
                          <Typography variant='subtitle1' style={{ display: 'inline' }}>
                            Device
                          </Typography>
                        </div>
                        <div>
                          {base.devices &&
                            React.Children.toArray(
                              Object.keys(base.devices).map((action_key, index_ac, list_ac) => (
                                <div>
                                  <Typography variant='subtitle1' className={classes.attributeName}>
                                    {action_key}
                                  </Typography>
                                  <div className={classes.actionValue}>
                                    <TextField
                                      required
                                      fullWidth={true}
                                      value={
                                        base.devices[action_key] ? base.devices[action_key] : 0
                                      }
                                    />
                                  </div>
                                </div>
                              ))
                            )}
                        </div>
                        <div>
                          <Typography variant='subtitle1' style={{ display: 'inline' }}>
                            mission
                          </Typography>
                        </div>
                        <div>
                          {base.mission &&
                            React.Children.toArray(
                              Object.keys(base.mission).map((action_key, index_ac, list_ac) => (
                                <div>
                                  <Typography variant='subtitle1' className={classes.attributeName}>
                                    {action_key}
                                  </Typography>
                                  <div className={classes.actionValue}>
                                    <TextField
                                      required
                                      fullWidth={true}
                                      value={
                                        base.mission[action_key] ? base.mission[action_key] : 0
                                      }
                                    />
                                  </div>
                                </div>
                              ))
                            )}
                        </div>
                      </Box>
                    </Fragment>
                  )}
                </AccordionDetails>
              </Accordion>
            ))
          )}
        </div>
      )}
    </Fragment>
  );
};

export default BaseSettings;

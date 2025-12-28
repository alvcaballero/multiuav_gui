import React, { Fragment, useEffect, useState } from 'react';
import { makeStyles } from 'tss-react/mui';

import { Box, IconButton, TextField, Accordion, AccordionSummary, AccordionDetails, Typography } from '@mui/material';

import ExpandMore from '@mui/icons-material/ExpandMore';
import MyLocationIcon from '@mui/icons-material/MyLocation';
import SelectField from '../common/components/SelectField';

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

const BaseSettings = ({
  data,
  markers,
  param,
  defaultSettings = {},
  setData,
  type = 'Base',
  goToBase = () => null
}) => {
  const { classes } = useStyles();

  const [expanded, setExpanded] = useState(false);
  const [dataExist, setDataExist] = useState(false);

  const handleChange = (panel) => (event, isExpanded) => {
    setExpanded(isExpanded ? panel : false);
  };

  const modifyData = (assignmentIndex, field, objvalue) => {
    const auxData = JSON.parse(JSON.stringify(data));
    console.log(auxData);

    if (field === 'device') {
      // Actualizar el device
      auxData[assignmentIndex].device = { ...auxData[assignmentIndex].device, ...objvalue };
    } else if (field === 'settings') {
      // Actualizar settings
      auxData[assignmentIndex].settings = { ...auxData[assignmentIndex].settings, ...objvalue };
    }

    setData(auxData);
  };

  useEffect(() => {
    if (markers && markers.bases) {
      markers.bases.length > 0 ? setDataExist(false) : setDataExist(true);
    }
  }, [markers]);

  return (
    <div>
      {dataExist ? (
        <Box textAlign="center">
          <Typography>No existen bases</Typography>
        </Box>
      ) : (
        <div className={classes.details}>
          {React.Children.toArray(
            markers.bases.map((base, baseIndex) => {
              // Buscar si existe una asignación para esta base
              const assignmentIndex = data.findIndex((a) => a.baseId === base.id);
              const assignment = assignmentIndex >= 0 ? data[assignmentIndex] : null;

              return (
                <Accordion
                  expanded={expanded === `wp ${base.id}`}
                  onChange={handleChange(`wp ${base.id}`)}
                >
                  <AccordionSummary expandIcon={<ExpandMore />}>
                    <Typography sx={{ width: '33%', flexShrink: 0 }}>
                      {`${type} ${baseIndex} - ${assignment?.device?.name || 'Sin asignar'}`}
                    </Typography>
                    <IconButton
                      sx={{ py: 0, pr: 2, marginLeft: 'auto' }}
                      onClick={(e) => {
                        e.stopPropagation();
                        goToBase(base.id);
                      }}
                    >
                      <MyLocationIcon />
                    </IconButton>
                  </AccordionSummary>
                  <AccordionDetails className={classes.details}>
                    {expanded === `wp ${base.id}` && (
                      <Box
                        component="form"
                        sx={{
                          '& .MuiTextField-root': { m: 1 },
                        }}
                      >
                        <div>
                          <Typography variant="subtitle1" style={{ display: 'inline' }}>
                            Device params
                          </Typography>
                        </div>
                        <div>
                          {param.devices &&
                            React.Children.toArray(
                              Object.keys(param.devices).map((actionKey) => (
                                <div>
                                  {actionKey === 'id' && (
                                    <SelectField
                                      emptyValue={''}
                                      fullWidth
                                      label="device"
                                      value={assignment?.device?.id || ''}
                                      endpoint="/api/devices"
                                      keyGetter={(it) => String(it.id)}
                                      titleGetter={(it) => `${it.name} - ${it.category}`}
                                      onChange={(e, items) => {
                                        console.log(items);
                                        console.log(e.target.value);

                                        // Crear o actualizar asignación
                                        const auxData = JSON.parse(JSON.stringify(data));
                                        const deviceId = e.target.value;
                                        const selectedDevice = items.find((item) => +item.id === +deviceId);

                                        if (assignmentIndex >= 0) {
                                          // Actualizar asignación existente
                                          auxData[assignmentIndex].device = {
                                            id: deviceId,
                                            name: deviceId === '' ? '' : selectedDevice?.name || '',
                                          };
                                        } else {
                                          // Crear nueva asignación
                                          auxData.push({
                                            baseId: base.id,
                                            device: {
                                              id: deviceId,
                                              name: deviceId === '' ? '' : selectedDevice?.name || '',
                                            },
                                            settings: defaultSettings,
                                          });
                                        }

                                        setData(auxData);
                                      }}
                                    />
                                  )}
                                </div>
                              ))
                            )}
                        </div>
                        <div>
                          <Typography variant="subtitle1" style={{ display: 'inline' }}>
                            Settings
                          </Typography>
                        </div>
                        <div>
                          {param.settings &&
                            React.Children.toArray(
                              Object.keys(param.settings).map((actionKey) => (
                                <div>
                                  {param.settings[actionKey].name && (
                                    <>
                                      <Typography variant="subtitle1" className={classes.attributeName}>
                                        {param.settings[actionKey].name}
                                      </Typography>
                                      <div className={classes.actionValue}>
                                        <TextField
                                          required
                                          fullWidth
                                          type="number"
                                          value={
                                            assignment?.settings?.[actionKey] !== undefined
                                              ? assignment.settings[actionKey]
                                              : param.settings[actionKey].default
                                          }
                                          onChange={(e) => {
                                            if (assignmentIndex >= 0) {
                                              modifyData(assignmentIndex, 'settings', {
                                                [actionKey]: +e.target.value,
                                              });
                                            }
                                          }}
                                        />
                                      </div>
                                    </>
                                  )}
                                </div>
                              ))
                            )}
                        </div>
                      </Box>
                    )}
                  </AccordionDetails>
                </Accordion>
              );
            })
          )}
        </div>
      )}
    </div>
  );
};

export default BaseSettings;

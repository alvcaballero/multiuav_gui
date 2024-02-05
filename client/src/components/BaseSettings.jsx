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
import MyLocationIcon from '@mui/icons-material/MyLocation';
import palette from '../common/palette';
import { map } from '../Mapview/MapView';
import SelectField from '../common/components/SelectField';

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

const BaseSettings = ({ data, param, setData, type = 'Base' }) => {
  const classes = useStyles();

  const [expanded, setExpanded] = useState(false);
  const [dataExist, setDataExist] = useState(false);

  const handleChange = (panel) => (event, isExpanded) => {
    setExpanded(isExpanded ? panel : false);
  };
  const modifyData = (index = 0, type = 'mission', key, value) => {
    let auxData = JSON.parse(JSON.stringify(data));
    auxData[index][type][key] = value;
    setData(auxData);
  };

  useEffect(() => {
    if (data) {
      data.length > 0 ? setDataExist(false) : setDataExist(true);
    }
  }, [data]);

  return (
    <div>
      {dataExist ? (
        <Box textAlign="center">
          <Typography>dont exits base</Typography>
        </Box>
      ) : (
        <div className={classes.details}>
          {React.Children.toArray(
            Object.values(data).map((base, index, list) => (
              <Accordion
                expanded={expanded === `wp ${index}`}
                onChange={handleChange(`wp ${index}`)}
              >
                <AccordionSummary expandIcon={<ExpandMore />}>
                  <Typography sx={{ width: '33%', flexShrink: 0 }}>{`${type} ${index}`}</Typography>
                  <IconButton
                    sx={{ py: 0, pr: 2, marginLeft: 'auto' }}
                    onClick={() => console.log('button')}
                  >
                    <MyLocationIcon />
                  </IconButton>
                </AccordionSummary>
                <AccordionDetails className={classes.details}>
                  {expanded === `wp ${index}` && (
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
                            Object.keys(param.devices).map((actionKey, index_ac, list_ac) => (
                              <div>
                                {actionKey == 'deviceId' ? (
                                  <SelectField
                                    emptyValue={null}
                                    fullWidth
                                    label="device"
                                    value={base.devices[actionKey]}
                                    endpoint="/api/devices"
                                    keyGetter={(it) => it.id}
                                    titleGetter={(it) => `${it.name} - ${it.category}`}
                                    onChange={(e) => {
                                      modifyData(index, 'devices', actionKey, e.target.value);
                                    }}
                                  />
                                ) : (
                                  <>
                                    <Typography
                                      variant="subtitle1"
                                      className={classes.attributeName}
                                    >
                                      {param.devices[actionKey].name}
                                    </Typography>
                                    <div className={classes.actionValue}>
                                      <TextField
                                        required
                                        fullWidth
                                        type="number"
                                        value={
                                          base.devices[actionKey]
                                            ? base.devices[actionKey]
                                            : param.devices[actionKey].default
                                        }
                                        onChange={(e) => {
                                          modifyData(index, 'devices', actionKey, e.target.value);
                                        }}
                                      />
                                    </div>
                                  </>
                                )}
                              </div>
                            ))
                          )}
                      </div>
                      <div>
                        <Typography variant="subtitle1" style={{ display: 'inline' }}>
                          Mission Params
                        </Typography>
                      </div>
                      <div>
                        {param.mission &&
                          React.Children.toArray(
                            Object.keys(param.mission).map((actionKey) => (
                              <div>
                                <Typography variant="subtitle1" className={classes.attributeName}>
                                  {actionKey}
                                </Typography>
                                <div className={classes.actionValue}>
                                  <TextField
                                    required
                                    fullWidth
                                    type="number"
                                    value={
                                      base.mission[actionKey]
                                        ? base.mission[actionKey]
                                        : param.devices[actionKey].default
                                    }
                                    onChange={(e) => {
                                      modifyData(index, 'mission', actionKey, e.target.value);
                                    }}
                                  />
                                </div>
                              </div>
                            ))
                          )}
                      </div>
                    </Box>
                  )}
                </AccordionDetails>
              </Accordion>
            ))
          )}
        </div>
      )}
    </div>
  );
};

export default BaseSettings;

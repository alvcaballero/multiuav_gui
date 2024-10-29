import React, { Fragment, useEffect, useState } from 'react';
import makeStyles from '@mui/styles/makeStyles';
import { Box, IconButton, TextField, Accordion, AccordionSummary, AccordionDetails, Typography } from '@mui/material';

import ExpandMore from '@mui/icons-material/ExpandMore';
import MyLocationIcon from '@mui/icons-material/MyLocation';
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

const BaseSettings = ({ data, param, setData, type = 'Base', goToBase = () => null }) => {
  const classes = useStyles();

  const [expanded, setExpanded] = useState(false);
  const [dataExist, setDataExist] = useState(false);

  const handleChange = (panel) => (event, isExpanded) => {
    setExpanded(isExpanded ? panel : false);
  };
  const modifyData = (index = 0, type = 'mission', objvalue) => {
    const auxData = JSON.parse(JSON.stringify(data));
    console.log(auxData);
    auxData[index][type] = { ...auxData[index][type], ...objvalue };
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
              <Accordion expanded={expanded === `wp ${index}`} onChange={handleChange(`wp ${index}`)}>
                <AccordionSummary expandIcon={<ExpandMore />}>
                  <Typography
                    sx={{ width: '33%', flexShrink: 0 }}
                  >{`${type} ${index} - ${base.devices.name} `}</Typography>
                  <IconButton sx={{ py: 0, pr: 2, marginLeft: 'auto' }} onClick={() => goToBase(index)}>
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
                                {actionKey == 'id' && (
                                  <SelectField
                                    emptyValue={null}
                                    fullWidth
                                    label="device"
                                    value={base.devices[actionKey]}
                                    endpoint="/api/devices"
                                    keyGetter={(it) => it.id}
                                    titleGetter={(it) => `${it.name} - ${it.category}`}
                                    onChange={(e, items) => {
                                      console.log(items);
                                      console.log(e.target.value);
                                      modifyData(index, 'devices', {
                                        id: e.target.value,
                                        name: items.find((item) => +item.id == +e.target.value).name,
                                      });
                                    }}
                                  />
                                )}
                                {actionKey == 'name' && (
                                  <>
                                    <Typography variant="subtitle1" className={classes.attributeName}>
                                      {param.devices[actionKey].name}
                                    </Typography>
                                    <div className={classes.attribute}>
                                      <TextField
                                        required
                                        fullWidth
                                        value={`${base.devices.name} - ${base.devices.category}`}
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
                                          base.settings[actionKey]
                                            ? base.settings[actionKey]
                                            : param.settings[actionKey].default
                                        }
                                        onChange={(e) => {
                                          modifyData(index, 'settings', { [actionKey]: +e.target.value });
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
            ))
          )}
        </div>
      )}
    </div>
  );
};

export default BaseSettings;

import React, { Fragment, useEffect, useState } from 'react';
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
import DeleteIcon from '@mui/icons-material/Delete';

import ExpandMore from '@mui/icons-material/ExpandMore';
import SelectField from '../common/components/SelectField';
import { map } from '../Mapview/MapView';

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

const SelectList = ({ Data, setData = () => null }) => {
  const { classes } = useStyles();

  const [expandedGroup, setExpandedGroup] = useState(false);
  const [expanded, setExpanded] = useState(false);
  const [ElementsExist, setElementsExist] = useState(false);

  const DeleteGroup = (index) => {
    let auxData = JSON.parse(JSON.stringify(Data));
    auxData.splice(index, 1);
    setData(auxData);
  };
  const DeleteElement = (index, IndexElement) => {
    let auxData = JSON.parse(JSON.stringify(Data));
    auxData[index].items.splice(IndexElement, 1);
    setData(auxData);
  };
  const handleChange = (panel) => (event, isExpanded) => {
    setExpanded(isExpanded ? panel : false);
  };
  const handleChangeGroup = (panel) => (event, isExpanded) => {
    setExpandedGroup(isExpanded ? panel : false);
  };

  useEffect(() => {
    if (Data) {
      Data.length > 0 ? setElementsExist(true) : setElementsExist(false);
    }
  }, [Data]);

  return (
    <div>
      {ElementsExist && (
        <div className={classes.details}>
          {React.Children.toArray(
            Object.values(Data).map((group, indexGroup, list) => (
              <Accordion
                expanded={expandedGroup === `Elements ${indexGroup}`}
                onChange={handleChangeGroup(`Elements ${indexGroup}`)}
              >
                <AccordionSummary expandIcon={<ExpandMore />}>
                  <Typography sx={{ width: '33%', flexShrink: 0 }}>{`Group ${indexGroup}`}</Typography>
                  <IconButton
                    sx={{ py: 0, pr: 2, marginLeft: 'auto' }}
                    onClick={(e) => {
                      e.stopPropagation();
                      DeleteGroup(indexGroup);
                    }}
                  >
                    <DeleteIcon />
                  </IconButton>
                </AccordionSummary>
                <AccordionDetails className={classes.details}>
                  {expandedGroup === `Elements ${indexGroup}` && (
                    <>
                      <TextField required label="Name" variant="standard" value={group.name ? group.name : ''} />
                      <div className={classes.details}>
                        {React.Children.toArray(
                          Object.values(group.items).map((element, index, list) => (
                            <Accordion
                              expanded={expanded === `Elements ${index}`}
                              onChange={handleChange(`Elements ${index}`)}
                            >
                              <AccordionSummary expandIcon={<ExpandMore />}>
                                {element.title ? (
                                  <Typography sx={{ width: '33%', flexShrink: 0 }}>
                                    {`Element ${element.title}`}
                                  </Typography>
                                ) : (
                                  <Typography sx={{ width: '33%', flexShrink: 0 }}>{`Element ${index}`}</Typography>
                                )}

                                <IconButton
                                  sx={{ py: 0, pr: 2, marginLeft: 'auto' }}
                                  onClick={(e) => {
                                    e.stopPropagation();
                                    DeleteElement(indexGroup, index);
                                  }}
                                >
                                  <DeleteIcon />
                                </IconButton>
                              </AccordionSummary>
                              <AccordionDetails className={classes.details}>
                                {expanded === `Elements ${index}` && (
                                  <Box
                                    component="form"
                                    sx={{
                                      '& .MuiTextField-root': { m: 1 },
                                    }}
                                  >
                                    <TextField
                                      disabled
                                      label="Latitude "
                                      type="number"
                                      sx={{ width: '15ch' }}
                                      variant="standard"
                                      inputProps={{
                                        maxLength: 8,
                                        step: 0.0001,
                                      }}
                                      value={element.latitude}
                                    />
                                    <TextField
                                      disabled
                                      label="Longitud "
                                      type="number"
                                      variant="standard"
                                      sx={{ width: '15ch' }}
                                      inputProps={{
                                        maxLength: 8,
                                        step: 0.0001,
                                      }}
                                      value={element.longitude}
                                    />
                                  </Box>
                                )}
                              </AccordionDetails>
                            </Accordion>
                          ))
                        )}
                      </div>
                    </>
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

export default SelectList;

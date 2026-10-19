import { useState } from 'react';
import { makeStyles } from 'tss-react/mui';

import {
  Box,
  IconButton,
  TextField,
  Accordion,
  AccordionSummary,
  AccordionDetails,
  Typography,
} from '@mui/material';
import DeleteIcon from '@mui/icons-material/Delete';

import ExpandMore from '@mui/icons-material/ExpandMore';

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
  const elementsExist = !!Data && Data.length > 0;

  const DeleteGroup = (index) => {
    let auxData = structuredClone(Data);
    auxData.splice(index, 1);
    setData(auxData);
  };
  const DeleteElement = (index, IndexElement) => {
    let auxData = structuredClone(Data);
    auxData[index].items.splice(IndexElement, 1);
    setData(auxData);
  };
  const handleChange = (panel) => (event, isExpanded) => {
    setExpanded(isExpanded ? panel : false);
  };
  const handleChangeGroup = (panel) => (event, isExpanded) => {
    setExpandedGroup(isExpanded ? panel : false);
  };

  return (
    <div>
      {elementsExist && (
        <div className={classes.details}>
          {Object.values(Data).map((group, indexGroup) => (
            <Accordion
              key={indexGroup}
              expanded={expandedGroup === `Elements ${indexGroup}`}
              onChange={handleChangeGroup(`Elements ${indexGroup}`)}
            >
              <Box sx={{ display: 'flex', alignItems: 'center' }}>
                <AccordionSummary expandIcon={<ExpandMore />} sx={{ flexGrow: 1, minWidth: 0 }}>
                  <Typography
                    sx={{ width: '33%', flexShrink: 0 }}
                  >{`Group ${indexGroup}`}</Typography>
                </AccordionSummary>
                <IconButton
                  sx={{ mr: 1 }}
                  onClick={(e) => {
                    e.stopPropagation();
                    DeleteGroup(indexGroup);
                  }}
                >
                  <DeleteIcon />
                </IconButton>
              </Box>
              <AccordionDetails className={classes.details}>
                {expandedGroup === `Elements ${indexGroup}` && (
                  <>
                    <TextField
                      required
                      label="Name"
                      variant="standard"
                      value={group.name ? group.name : ''}
                    />
                    <div className={classes.details}>
                      {Object.values(group.items).map((element, index) => (
                        <Accordion
                          key={`${element.title ?? 'el'}-${element.latitude}-${element.longitude}-${index}`}
                          expanded={expanded === `Elements ${index}`}
                          onChange={handleChange(`Elements ${index}`)}
                        >
                          <Box sx={{ display: 'flex', alignItems: 'center' }}>
                            <AccordionSummary
                              expandIcon={<ExpandMore />}
                              sx={{ flexGrow: 1, minWidth: 0 }}
                            >
                              {element.title ? (
                                <Typography sx={{ width: '33%', flexShrink: 0 }}>
                                  {`Element ${element.title}`}
                                </Typography>
                              ) : (
                                <Typography
                                  sx={{ width: '33%', flexShrink: 0 }}
                                >{`Element ${index}`}</Typography>
                              )}
                            </AccordionSummary>
                            <IconButton
                              sx={{ mr: 1 }}
                              onClick={(e) => {
                                e.stopPropagation();
                                DeleteElement(indexGroup, index);
                              }}
                            >
                              <DeleteIcon />
                            </IconButton>
                          </Box>
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
                                  slotProps={{ htmlInput: { maxLength: 8, step: 0.0001 } }}
                                  value={element.latitude}
                                />
                                <TextField
                                  disabled
                                  label="Longitud "
                                  type="number"
                                  variant="standard"
                                  sx={{ width: '15ch' }}
                                  slotProps={{ htmlInput: { maxLength: 8, step: 0.0001 } }}
                                  value={element.longitude}
                                />
                              </Box>
                            )}
                          </AccordionDetails>
                        </Accordion>
                      ))}
                    </div>
                  </>
                )}
              </AccordionDetails>
            </Accordion>
          ))}
        </div>
      )}
    </div>
  );
};

export default SelectList;

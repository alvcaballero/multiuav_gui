import React from 'react';
import { Accordion, AccordionSummary, AccordionDetails, Box, Button, Divider, Typography } from '@mui/material';
import ExpandMore from '@mui/icons-material/ExpandMore';
import { makeStyles } from 'tss-react/mui';
import BaseList from '../map/BaseList';
import ElementList from '../map/ElementList';

const useStyles = makeStyles()((theme) => ({
  details: {
    display: 'flex',
    flexDirection: 'column',
    gap: theme.spacing(2),
    paddingBottom: theme.spacing(3),
  },
  tabPanelContent: {
    maxHeight: 'calc(100vh - 240px)',
    overflowY: 'auto',
    paddingRight: '8px',
  },
  panelButton: {
    width: '80%',
    flexShrink: 0,
    marginTop: '15px',
  },
}));

const ElementsTab = ({ markers, setMarkersBase, setMarkersElements, onSaveGlobalMarkers }) => {
  const { classes } = useStyles();

  return (
    <div className={`${classes.details} ${classes.tabPanelContent}`}>
      <Accordion>
        <AccordionSummary expandIcon={<ExpandMore />}>
          <Typography>Base elements</Typography>
        </AccordionSummary>
        <AccordionDetails className={classes.details}>
          <BaseList markers={markers.bases} setMarkers={setMarkersBase} />
        </AccordionDetails>
      </Accordion>
      <Divider />
      <Accordion>
        <AccordionSummary expandIcon={<ExpandMore />}>
          <Typography>Interest Elements</Typography>
        </AccordionSummary>
        <AccordionDetails className={classes.details}>
          <ElementList markers={markers.elements} setMarkers={setMarkersElements} />
        </AccordionDetails>
      </Accordion>
      <Box textAlign="center">
        <Button variant="contained" size="large" className={classes.panelButton} onClick={onSaveGlobalMarkers}>
          Save Global Markers
        </Button>
      </Box>
    </div>
  );
};

export default ElementsTab;

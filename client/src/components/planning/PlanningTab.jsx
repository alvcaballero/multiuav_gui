import React from 'react';
import {
  Accordion,
  AccordionSummary,
  AccordionDetails,
  Box,
  Divider,
  TextField,
  Typography,
} from '@mui/material';
import ExpandMore from '@mui/icons-material/ExpandMore';
import { makeStyles } from 'tss-react/mui';
import SelectField from '../../shared/components/SelectField';
import SelectList from '../ui/SelectList';

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
}));

const PlanningTab = ({
  missionRequest,
  onUpdateId,
  onUpdateName,
  onUpdateObjective,
  onGetItems,
  setLocations,
}) => {
  const { classes } = useStyles();

  return (
    <Box className={`${classes.details} ${classes.tabPanelContent}`}>
      <TextField
        required
        fullWidth
        label="id"
        type="number"
        variant="standard"
        defaultValue={missionRequest.id ? missionRequest.id : 123}
        onBlur={onUpdateId}
      />
      <TextField
        required
        fullWidth
        label="Name Mission"
        variant="standard"
        value={missionRequest.name ? missionRequest.name : ' '}
        onChange={onUpdateName}
      />
      <SelectField
        emptyValue={null}
        fullWidth
        label="objetive"
        value={missionRequest.objetivo.hasOwnProperty('id') ? missionRequest.objetivo.id : 1}
        endpoint="/api/planning/missionstype"
        keyGetter={(it) => it.id}
        titleGetter={(it) => it.name}
        onChange={onUpdateObjective}
        getItems={onGetItems}
      />

      <Accordion>
        <AccordionSummary expandIcon={<ExpandMore />}>
          <Typography>Points to inspect</Typography>
        </AccordionSummary>
        <AccordionDetails className={classes.details}>
          <div className={classes.details}>
            {missionRequest.objetivo.hasOwnProperty('description') ? (
              <Typography>{missionRequest.objetivo.description}</Typography>
            ) : (
              <Typography>select doing click in the map</Typography>
            )}
            <SelectList Data={missionRequest.loc} setData={setLocations} />
          </div>
        </AccordionDetails>
      </Accordion>
      <Divider />
      <Accordion>
        <AccordionSummary expandIcon={<ExpandMore />}>
          <Typography>Meteo</Typography>
        </AccordionSummary>
        <AccordionDetails className={classes.details}>
          <TextField
            required
            fullWidth
            label="wind speed"
            type="number"
            variant="standard"
            value={12}
          />
          <TextField
            required
            fullWidth
            label="Wind direction"
            type="number"
            variant="standard"
            value={12}
          />
          <TextField
            required
            fullWidth
            label="Temperature"
            type="number"
            variant="standard"
            value={12}
          />
          <TextField
            required
            fullWidth
            label="Humidity"
            type="number"
            variant="standard"
            value={12}
          />
          <TextField
            required
            fullWidth
            label="Pressure"
            type="number"
            variant="standard"
            value={12}
          />
        </AccordionDetails>
      </Accordion>
    </Box>
  );
};

export default PlanningTab;

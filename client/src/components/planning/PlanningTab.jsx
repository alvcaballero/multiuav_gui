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
  sendTask,
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
        defaultValue={sendTask.id ? sendTask.id : 123}
        onBlur={onUpdateId}
      />
      <TextField
        required
        fullWidth
        label="Name Mission"
        variant="standard"
        value={sendTask.name ? sendTask.name : ' '}
        onChange={onUpdateName}
      />
      <SelectField
        emptyValue={null}
        fullWidth
        label="objetive"
        value={sendTask.objetivo.hasOwnProperty('id') ? sendTask.objetivo.id : 1}
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
            {sendTask.objetivo.hasOwnProperty('description') ? (
              <Typography>{sendTask.objetivo.description}</Typography>
            ) : (
              <Typography>select doing click in the map</Typography>
            )}
            <SelectList Data={sendTask.loc} setData={setLocations} />
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

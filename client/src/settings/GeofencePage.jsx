import { useState } from 'react';
import { useDispatch } from 'react-redux';
import {
  Accordion,
  AccordionSummary,
  AccordionDetails,
  Typography,
  TextField,
  FormControlLabel,
  Checkbox,
} from '@mui/material';
import ExpandMoreIcon from '@mui/icons-material/ExpandMore';
import EditItemView from './components/EditItemView';
import SettingsMenu from './components/SettingsMenu';
import { geofencesActions } from '../store';
import useSettingsStyles from './common/useSettingsStyles';

const GeofencePage = () => {
  const { classes } = useSettingsStyles();
  const dispatch = useDispatch();

  const [item, setItem] = useState();

  const onItemSaved = (result) => {
    dispatch(geofencesActions.update([result]));
  };

  const validate = () => item && item.name;

  return (
    <EditItemView
      endpoint="geofences"
      item={item}
      setItem={setItem}
      validate={validate}
      onItemSaved={onItemSaved}
      menu={<SettingsMenu />}
      breadcrumbs={['Settings', 'Geofence']}
    >
      {item && (
        <>
          <Accordion defaultExpanded>
            <AccordionSummary expandIcon={<ExpandMoreIcon />}>
              <Typography variant="subtitle1">Obligatory data</Typography>
            </AccordionSummary>
            <AccordionDetails className={classes.details}>
              <TextField
                value={item.name || ''}
                onChange={(event) => setItem({ ...item, name: event.target.value })}
                label="Name"
              />
            </AccordionDetails>
          </Accordion>
          <Accordion>
            <AccordionSummary expandIcon={<ExpandMoreIcon />}>
              <Typography variant="subtitle1">Extra</Typography>
            </AccordionSummary>
            <AccordionDetails className={classes.details}>
              <TextField
                value={item.description || ''}
                onChange={(event) => setItem({ ...item, description: event.target.value })}
                label="Description"
              />

              <FormControlLabel
                control={
                  <Checkbox
                    checked={!!item.attributes?.hide}
                    onChange={(e) =>
                      setItem({
                        ...item,
                        attributes: { ...item.attributes, hide: e.target.checked },
                      })
                    }
                  />
                }
                label="Hide on map"
              />
            </AccordionDetails>
          </Accordion>
        </>
      )}
    </EditItemView>
  );
};

export default GeofencePage;

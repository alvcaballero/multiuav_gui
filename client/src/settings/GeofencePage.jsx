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
//import EditAttributesAccordion from './components/EditAttributesAccordion';
//import useGeofenceAttributes from '../common/attributes/useGeofenceAttributes';
import SettingsMenu from './components/SettingsMenu';
import SelectField from '../common/components/SelectField';
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
            breadcrumbs={['settingsTitle', 'sharedGeofence']}
        >
            {item && (
                <>
                    <Accordion defaultExpanded>
                        <AccordionSummary expandIcon={<ExpandMoreIcon />}>
                            <Typography variant="subtitle1">
                                {'sharedRequired'}
                            </Typography>
                        </AccordionSummary>
                        <AccordionDetails className={classes.details}>
                            <TextField
                                value={item.name || ''}
                                onChange={(event) => setItem({ ...item, name: event.target.value })}
                                label={'sharedName'}
                            />
                        </AccordionDetails>
                    </Accordion>
                    <Accordion>
                        <AccordionSummary expandIcon={<ExpandMoreIcon />}>
                            <Typography variant="subtitle1">
                                {'sharedExtra'}
                            </Typography>
                        </AccordionSummary>
                        <AccordionDetails className={classes.details}>
                            <TextField
                                value={item.description || ''}
                                onChange={(event) => setItem({ ...item, description: event.target.value })}
                                label={'sharedDescription'}
                            />
                            <SelectField
                                value={item.calendarId}
                                onChange={(event) => setItem({ ...item, calendarId: Number(event.target.value) })}
                                endpoint="/api/calendars"
                                label={'sharedCalendar'}
                            />
                            <FormControlLabel
                                control={<Checkbox checked={item.attributes.hide} onChange={(e) => setItem({ ...item, attributes: { ...item.attributes, hide: e.target.checked } })} />}
                                label={'sharedFilterMap'}
                            />
                        </AccordionDetails>
                    </Accordion>

                </>
            )}
        </EditItemView>
    );
};

export default GeofencePage;

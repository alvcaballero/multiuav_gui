import {
  TextField,
  Accordion,
  AccordionSummary,
  AccordionDetails,
  Typography,
} from '@mui/material';
import ExpandMoreIcon from '@mui/icons-material/ExpandMore';
import { makeStyles } from 'tss-react/mui';
import SelectField from '../../common/components/SelectField';
import { isMavlink, messageFormat, systemIdError, withProtocol } from './deviceIdentity';

const useStyles = makeStyles()((theme) => ({
  details: {
    display: 'flex',
    flexDirection: 'column',
    gap: theme.spacing(2),
    paddingBottom: theme.spacing(3),
  },
}));

// Controlled editor for the identity of a device: name, protocol, category (only those the
// protocol can drive, from GET /api/category?protocol=…), the MAVLink system id and the IP.
// Shared by the add dialog (Adduav) and the settings page, like DeviceCameraEditor.
const DeviceIdentityEditor = ({ value, onChange }) => {
  const { classes } = useStyles();
  const set = (patch) => onChange({ ...value, ...patch });
  const mavlink = isMavlink(value.protocol);
  const sysidProblem = mavlink && value.systemId != null ? systemIdError(value.systemId) : null;

  return (
    <Accordion defaultExpanded>
      <AccordionSummary expandIcon={<ExpandMoreIcon />}>
        <Typography variant="subtitle1">Obligatory data</Typography>
      </AccordionSummary>
      <AccordionDetails className={classes.details}>
        <TextField
          required
          label="Name"
          name="uavid"
          value={value.name ?? ''}
          onChange={(event) => set({ name: event.target.value })}
          helperText="The name must be unique, it will be used to identify the device, must to be the same of that device's name_space or device's identificator ,it is recommended to use the format uav_XXXX in case of using a UAV"
        />
        {/* GET /api/server/protocol lists the four protocols with `enabled`: the ones this server
            does not run are shown greyed out. No auto-selection (emptyValue ''): it would pass the
            whole item, and could land on a disabled protocol. */}
        <SelectField
          emptyValue=""
          value={value.protocol ?? ''}
          onChange={(e) => onChange(withProtocol(value, e.target.value || null))}
          endpoint="/api/server/protocol"
          keyGetter={(it) => it.name}
          titleGetter={(it) => (it.enabled ? it.name : `${it.name} (disabled on the server)`)}
          itemDisabled={(it) => !it.enabled}
          label="Protocol "
        />
        {value.protocol && (
          // Remounted per message format: SelectField keeps its previous list until the new one
          // arrives, and would auto-select a category the new protocol cannot carry meanwhile.
          <SelectField
            key={messageFormat(value.protocol)}
            emptyValue={null}
            value={value.category ?? null}
            onChange={(e) => set({ category: e.target.value })}
            endpoint={`/api/category?protocol=${encodeURIComponent(value.protocol)}`}
            keyGetter={(it) => it}
            titleGetter={(it) => it}
            label="Type "
          />
        )}
        {mavlink && (
          <TextField
            required
            type="number"
            label="MAVLink system id"
            value={value.systemId ?? ''}
            onChange={(event) =>
              set({ systemId: event.target.value === '' ? null : Number(event.target.value) })
            }
            slotProps={{ htmlInput: { min: 1, max: 254, step: 1 } }}
            error={Boolean(sysidProblem)}
            helperText={
              sysidProblem ??
              'The vehicle is found by this id, on any MAVLink link of the server. PX4: MAV_SYS_ID. ArduPilot: MAV_SYSID (SYSID_THISMAV before 4.7). Unique per vehicle.'
            }
          />
        )}
        <TextField
          label="ip"
          value={value.ip ?? ''}
          onChange={(event) => set({ ip: event.target.value })}
          helperText="IP for camera stream"
        />
      </AccordionDetails>
    </Accordion>
  );
};

export default DeviceIdentityEditor;

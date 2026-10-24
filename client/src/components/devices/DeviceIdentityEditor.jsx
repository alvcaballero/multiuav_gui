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
import {
  isMavlink,
  messageFormat,
  systemIdError,
  withCategory,
  withProtocol,
} from './deviceIdentity';

const useStyles = makeStyles()((theme) => ({
  details: {
    display: 'flex',
    flexDirection: 'column',
    gap: theme.spacing(2),
    paddingBottom: theme.spacing(3),
  },
}));

// Controlled editor for the identity of a device: name, protocol, category (only those the
// protocol can drive, from GET /api/category?protocol=…), device model (only those the category can
// drive, GET /api/device-models?category=…), the protocol's own attributes (MAVLink: system id).
// Shared by the add dialog (AddDevice) and the settings page, like DeviceCameraEditor.
const DeviceIdentityEditor = ({ value, onChange }) => {
  const { classes } = useStyles();
  const set = (patch) => onChange({ ...value, ...patch });
  const mavlink = isMavlink(value.protocol);
  const setAttribute = (patch) => set({ attributes: { ...value.attributes, ...patch } });
  const systemId = value.attributes?.systemId;
  const sysidProblem = mavlink && systemId != null ? systemIdError(systemId) : null;

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
            // The auto-selection (emptyValue={null}) hands over the whole ITEM, a pick in the list
            // its key: keep only the name either way.
            onChange={(e) => onChange(withCategory(value, e.target.value?.name ?? e.target.value))}
            // details: each category with its `root` (top of its `extends` chain), sorted so a
            // variant (dji_M210_melodic) follows the category it extends (dji_M210_noetic).
            endpoint={`/api/category?protocol=${encodeURIComponent(value.protocol)}&details=true`}
            keyGetter={(it) => it.name}
            titleGetter={(it) =>
              it.root === it.name ? it.name : `${it.name} (variant of ${it.root})`
            }
            label="Type "
          />
        )}
        {value.category && (
          // What the device IS. Only the models this category can drive, its default first, so the
          // auto-selection (emptyValue={null}, again the whole item) lands on the default. Remounted
          // per category for the same reason as the list above.
          <SelectField
            key={value.category}
            emptyValue={null}
            value={value.deviceModel ?? null}
            onChange={(e) => set({ deviceModel: e.target.value?.name ?? e.target.value })}
            endpoint={`/api/device-models?category=${encodeURIComponent(value.category)}`}
            keyGetter={(it) => it.name}
            titleGetter={(it) =>
              `${it.label ?? it.name} · ${it.vehicleType}${it.isDefault ? ' (default)' : ''}`
            }
            label="Model "
          />
        )}
        {mavlink && (
          <TextField
            required
            type="number"
            label="MAVLink system id"
            value={systemId ?? ''}
            onChange={(event) =>
              setAttribute({
                systemId: event.target.value === '' ? null : Number(event.target.value),
              })
            }
            slotProps={{ htmlInput: { min: 1, max: 254, step: 1 } }}
            error={Boolean(sysidProblem)}
            helperText={
              sysidProblem ??
              'The vehicle is found by this id, on any MAVLink link of the server. PX4: MAV_SYS_ID. ArduPilot: MAV_SYSID (SYSID_THISMAV before 4.7). Unique per vehicle.'
            }
          />
        )}
      </AccordionDetails>
    </Accordion>
  );
};

export default DeviceIdentityEditor;

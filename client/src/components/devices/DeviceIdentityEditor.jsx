import { useEffect, useState } from 'react';
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
import { useAsyncTask } from '../../reactHelper';
import {
  isMavlink,
  protocolsFor,
  systemIdError,
  withCategory,
  withModel,
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

const fetchJson = async (url, signal) => {
  const response = await fetch(url, { signal });
  if (!response.ok) throw Error(await response.text());
  return response.json();
};

// Controlled editor for the identity of a device, in the order its parts depend on each other:
// name → model (what it IS, GET /api/device-models) → interface (one the model lists,
// GET /api/category?deviceModel=…) → protocol (one that carries the interface's message format;
// picked by itself when only one does, as for MAVLink) → the protocol's own attributes (MAVLink:
// system id). Changing one clears what depended on it (deviceIdentity.js). No list auto-selects:
// SelectField's auto-selection hands over the whole item, and picking for the user hides a choice.
// Shared by the add dialog (AddDevice) and the settings page, like DeviceCameraEditor.
const DeviceIdentityEditor = ({ value, onChange }) => {
  const { classes } = useStyles();
  const set = (patch) => onChange({ ...value, ...patch });
  const mavlink = isMavlink(value.protocol);
  const setAttribute = (patch) => set({ attributes: { ...value.attributes, ...patch } });
  const systemId = value.attributes?.systemId;
  const sysidProblem = mavlink && systemId != null ? systemIdError(systemId) : null;

  const [models, setModels] = useState(null);
  const [interfaces, setInterfaces] = useState(null);
  const [protocols, setProtocols] = useState(null);

  useAsyncTask(async ({ signal }) => {
    setModels(await fetchJson('/api/device-models', signal));
    setProtocols(await fetchJson('/api/server/protocol', signal));
  }, []);

  useAsyncTask(
    async ({ signal }) => {
      setInterfaces(null);
      if (!value.deviceModel) return;
      const url = `/api/category?deviceModel=${encodeURIComponent(value.deviceModel)}&details=true`;
      setInterfaces(await fetchJson(url, signal));
    },
    [value.deviceModel],
  );

  const format = interfaces?.find((it) => it.name === value.category)?.messageFormat;
  const carriers = format ? protocolsFor(protocols, format) : [];
  const enabledCarriers = carriers.filter((p) => p.enabled);

  // Only one protocol carries this interface (MAVLink): nothing to choose.
  useEffect(() => {
    if (!value.protocol && carriers.length === 1 && enabledCarriers.length === 1) {
      onChange(withProtocol(value, enabledCarriers[0].name));
    }
  }, [value, carriers.length, enabledCarriers, onChange]);

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
        {/* What the device IS: grouped by vehicle type on the server, generic last in each. */}
        <SelectField
          emptyValue=""
          value={value.deviceModel ?? ''}
          onChange={(e) => onChange(withModel(value, e.target.value || null))}
          data={models ?? undefined}
          keyGetter={(it) => it.name}
          titleGetter={(it) => `${it.label ?? it.name} · ${it.vehicleType}`}
          label="Model "
        />
        {value.deviceModel && (
          // The interfaces this model lists; a variant (dji_M210_melodic) follows the one it extends.
          <SelectField
            emptyValue=""
            value={value.category ?? ''}
            onChange={(e) => {
              const category = e.target.value || null;
              const picked = interfaces?.find((it) => it.name === category);
              onChange(withCategory(value, category, picked?.messageFormat));
            }}
            data={interfaces ?? undefined}
            keyGetter={(it) => it.name}
            titleGetter={(it) =>
              it.root === it.name ? it.name : `${it.name} (variant of ${it.root})`
            }
            label="Interface "
          />
        )}
        {value.category && format && (
          // The protocols that carry this interface's messages; the ones this server does not run
          // are greyed out (GET /api/server/protocol `enabled`).
          <SelectField
            emptyValue=""
            value={value.protocol ?? ''}
            onChange={(e) => onChange(withProtocol(value, e.target.value || null))}
            data={carriers}
            keyGetter={(it) => it.name}
            titleGetter={(it) => (it.enabled ? it.name : `${it.name} (disabled on the server)`)}
            itemDisabled={(it) => !it.enabled}
            label="Protocol "
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

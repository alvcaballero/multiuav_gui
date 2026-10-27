import { useEffect, useState } from 'react';
import {
  TextField,
  Accordion,
  AccordionSummary,
  AccordionDetails,
  Typography,
  FormControl,
  FormLabel,
  FormGroup,
  FormControlLabel,
  Checkbox,
} from '@mui/material';
import ExpandMoreIcon from '@mui/icons-material/ExpandMore';
import { makeStyles } from 'tss-react/mui';
import SelectField from '../../common/components/SelectField';
import { useAsyncTask } from '../../reactHelper';
import {
  VEHICLE_TYPE_LABELS,
  hasCapability,
  isMavlink,
  vehicleTypesOf,
  companionIpError,
  protocolsFor,
  radioIpError,
  radioMacError,
  systemIdError,
  withCapability,
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

// What this unit can do, against its model (server config/devices/capabilities.yaml): a box per
// capability the model declares, checked when the unit has it. `standard` ones start checked (uncheck
// a broken gimbal), `optional` ones unchecked (check an RTK module that is fitted). A generic model
// ('*') has whatever its interface offers: nothing to switch. Only the differences are stored, in
// attributes.capabilities. Whether the interface implements each one is the server's call.
const DeviceCapabilitiesEditor = ({ value, onChange, model, catalog }) => {
  if (!model) return null;
  if (model.capabilities === '*') {
    return (
      <Typography variant="body2" color="text.secondary">
        Generic model: the device can do everything its interface offers.
      </Typography>
    );
  }
  const declared = Object.entries(model.capabilities ?? {});
  if (declared.length === 0) {
    return (
      <Typography variant="body2" color="text.secondary">
        Telemetry only: this model declares no capabilities.
      </Typography>
    );
  }
  return (
    <FormControl component="fieldset" variant="standard">
      <FormLabel component="legend">Capabilities</FormLabel>
      <FormGroup>
        {declared.map(([name, level]) => (
          <FormControlLabel
            key={name}
            control={
              <Checkbox
                checked={hasCapability(value, name, level)}
                onChange={(e) => onChange(withCapability(value, name, level, e.target.checked))}
              />
            }
            label={`${catalog?.[name]?.label ?? name}${level === 'optional' ? ' (optional)' : ''}`}
          />
        ))}
      </FormGroup>
    </FormControl>
  );
};

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
  const [catalog, setCatalog] = useState(null);
  // The vehicle-type filter of the model list. Until the user picks one, it is the type of the model
  // the device already has (editing), or none (all models).
  const [vehicleType, setVehicleType] = useState(undefined); // undefined = untouched, null = all
  const currentModel = models?.find((m) => m.name === value.deviceModel);
  const shownType = vehicleType !== undefined ? vehicleType : (currentModel?.vehicleType ?? null);

  useAsyncTask(async ({ signal }) => {
    setModels(await fetchJson('/api/device-models', signal));
    setProtocols(await fetchJson('/api/server/protocol', signal));
    setCatalog(await fetchJson('/api/device-models/capabilities', signal));
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
        {/* Only narrows the model list: the type is the model's. Picking another type drops a model
            that is not of it. Empty = every type. */}
        <SelectField
          emptyValue=""
          emptyTitle="All"
          value={shownType ?? ''}
          onChange={(e) => {
            const type = e.target.value || null;
            setVehicleType(type);
            if (type && currentModel && currentModel.vehicleType !== type)
              onChange(withModel(value, null));
          }}
          data={models ? vehicleTypesOf(models).map((type) => ({ type })) : undefined}
          keyGetter={(it) => it.type}
          titleGetter={(it) => VEHICLE_TYPE_LABELS[it.type] ?? it.type}
          label="Vehicle type "
        />
        {/* What the device IS: grouped by vehicle type on the server, generic last in each. */}
        <SelectField
          emptyValue=""
          value={value.deviceModel ?? ''}
          onChange={(e) => onChange(withModel(value, e.target.value || null))}
          data={
            models ? models.filter((m) => !shownType || m.vehicleType === shownType) : undefined
          }
          keyGetter={(it) => it.name}
          titleGetter={(it) =>
            shownType
              ? (it.label ?? it.name)
              : `${it.label ?? it.name} · ${VEHICLE_TYPE_LABELS[it.vehicleType] ?? it.vehicleType}`
          }
          label="Model "
        />
        <DeviceCapabilitiesEditor
          value={value}
          onChange={onChange}
          model={currentModel}
          catalog={catalog}
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
        {value.protocol === 'ros' && (
          // ROS only (server ATTRIBUTE_GROUPS 'ros'): the companion republishes the telemetry slower under
          // /<device>/slow<topic>. Cleared with the other protocol attributes when the protocol changes.
          <FormControlLabel
            control={
              <Checkbox
                checked={Boolean(value.attributes?.slow)}
                onChange={(e) => setAttribute({ slow: e.target.checked || undefined })}
              />
            }
            label="Slow relay: read the telemetry from /<device>/slow/… (the relay on the companion)"
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
        {/* The radio on board (server ATTRIBUTE_GROUPS 'radio'), for any protocol: the link quality to the GCS's
            access point is read from the AP's station table. Empty = no radio watched. */}
        <TextField
          label="Radio IP (optional)"
          value={value.attributes?.radioIp ?? ''}
          onChange={(event) =>
            setAttribute({ radioIp: event.target.value === '' ? null : event.target.value })
          }
          error={Boolean(radioIpError(value.attributes?.radioIp))}
          helperText={
            radioIpError(value.attributes?.radioIp) ??
            "The Ubiquiti on board, NOT the companion computer's address. Used to find it in the access point's station table."
          }
        />
        <TextField
          label="Radio MAC (optional)"
          value={value.attributes?.radioMac ?? ''}
          onChange={(event) =>
            setAttribute({ radioMac: event.target.value === '' ? null : event.target.value })
          }
          error={Boolean(radioMacError(value.attributes?.radioMac))}
          helperText={
            radioMacError(value.attributes?.radioMac) ??
            'Wins over the IP when both are given: an address can change, the MAC does not.'
          }
        />
        {/* The onboard computer, pinged from the GCS (server ATTRIBUTE_GROUPS 'companion'): the real path to it,
            whatever the radio. Empty = not pinged. */}
        <TextField
          label="Onboard computer IP (optional)"
          value={value.attributes?.companionIp ?? ''}
          onChange={(event) =>
            setAttribute({ companionIp: event.target.value === '' ? null : event.target.value })
          }
          error={Boolean(companionIpError(value.attributes?.companionIp))}
          helperText={
            companionIpError(value.attributes?.companionIp) ??
            'The GCS pings it to show latency and packet loss. Needs `ping` on the server host.'
          }
        />
      </AccordionDetails>
    </Accordion>
  );
};

export default DeviceIdentityEditor;

import { useState } from 'react';
import SelectField from '../shared/components/SelectField';
import {
  TextField,
  Accordion,
  AccordionSummary,
  AccordionDetails,
  Typography,
} from '@mui/material';
import ExpandMoreIcon from '@mui/icons-material/ExpandMore';
import SettingsMenu from './components/SettingsMenu';
import useSettingsStyles from './common/useSettingsStyles';
import useQuery from '../shared/useQuery';
import EditItemView from './components/EditItemView';
import DeviceCameraEditor from '../components/devices/DeviceCameraEditor';
import DeviceFilesEditor from '../components/devices/DeviceFilesEditor';

const SettingsDevicesPageEdit = () => {
  const { classes } = useSettingsStyles();

  const query = useQuery();
  const uniqueId = query.get('uniqueId');

  const [item, setItem] = useState(uniqueId ? { uniqueId } : null);

  const validate = () => item && item.name && item.category && item.protocol && item.ip;

  return (
    <EditItemView
      endpoint="devices"
      item={item}
      setItem={setItem}
      validate={validate}
      menu={<SettingsMenu />}
      breadcrumbs={['settingsTitle', 'sharedDevice']}
    >
      {item && (
        <>
          <Accordion defaultExpanded>
            <AccordionSummary expandIcon={<ExpandMoreIcon />}>
              <Typography variant="subtitle1">Obligatory data</Typography>
            </AccordionSummary>
            <AccordionDetails className={classes.details}>
              <TextField
                required
                label="Name"
                name="uavid"
                value={item.name}
                onChange={(event) => setItem({ ...item, name: event.target.value })}
                helperText="The name must be unique, it will be used to identify the device, must to be the same of that device's name_space or device's identificator ,it is recommended to use the format uav_XXXX in case of using a UAV"
              />
              <SelectField
                emptyValue={null}
                value={item.category ? item.category : null}
                onChange={(e) => setItem({ ...item, category: e.target.value })}
                endpoint="/api/category"
                keyGetter={(it) => it}
                titleGetter={(it) => it}
                label={'Type '}
              />
              {item.category && (
                <SelectField
                  emptyValue={null}
                  value={item.protocol ? item.protocol : null}
                  onChange={(e) => setItem({ ...item, protocol: e.target.value })}
                  endpoint="/api/server/protocol"
                  keyGetter={(it) => it}
                  titleGetter={(it) => it}
                  label={'Protocol '}
                />
              )}
              <TextField
                label="ip"
                value={item.ip}
                onChange={(event) => setItem({ ...item, ip: event.target.value })}
                helperText="IP for camera stream"
              />
            </AccordionDetails>
          </Accordion>

          <DeviceCameraEditor
            value={item.camera}
            onChange={(camera) => setItem({ ...item, camera })}
          />
          <DeviceFilesEditor value={item.files} onChange={(files) => setItem({ ...item, files })} />
        </>
      )}
    </EditItemView>
  );
};

export default SettingsDevicesPageEdit;

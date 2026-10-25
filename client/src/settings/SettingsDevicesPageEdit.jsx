import { useState } from 'react';
import SettingsMenu from './components/SettingsMenu';
import useQuery from '../common/util/useQuery';
import EditItemView from './components/EditItemView';
import DeviceIdentityEditor from '../components/devices/DeviceIdentityEditor';
import { capabilityEnabled, isDeviceIdentityValid } from '../components/devices/deviceIdentity';
import useDeviceModel from '../components/devices/useDeviceModel';
import DeviceCameraEditor from '../components/devices/DeviceCameraEditor';
import DeviceFilesEditor from '../components/devices/DeviceFilesEditor';

const SettingsDevicesPageEdit = () => {
  const query = useQuery();
  const uniqueId = query.get('uniqueId');

  const [item, setItem] = useState(uniqueId ? { uniqueId } : null);
  const model = useDeviceModel(item?.deviceModel);

  const validate = () => isDeviceIdentityValid(item);

  return (
    <EditItemView
      endpoint="devices"
      item={item}
      setItem={setItem}
      validate={validate}
      menu={<SettingsMenu />}
      breadcrumbs={['Settings', 'Device']}
    >
      {item && (
        <>
          <DeviceIdentityEditor value={item} onChange={setItem} />

          {/* Only the source settings of the capabilities it has; switching one off hides its
              settings without deleting them, so switching it back on restores them. */}
          {capabilityEnabled(item, model, 'camera_stream') && (
            <DeviceCameraEditor
              value={item.camera}
              onChange={(camera) => setItem({ ...item, camera })}
            />
          )}
          {capabilityEnabled(item, model, 'file_download') && (
            <DeviceFilesEditor
              value={item.files}
              onChange={(files) => setItem({ ...item, files })}
            />
          )}
        </>
      )}
    </EditItemView>
  );
};

export default SettingsDevicesPageEdit;

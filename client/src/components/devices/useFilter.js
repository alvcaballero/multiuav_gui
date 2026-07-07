import { useMemo } from 'react';
import { useSelector } from 'react-redux';
import dayjs from 'dayjs';

export default (keyword, filter, filterSort, filterMap, positions) => {
  const devices = useSelector((state) => state.devices.items);

  const filteredDevices = useMemo(() => {
    const filtered = Object.values(devices).filter((device) => {
      if (filter.statuses.length && !filter.statuses.includes(device.status)) return false;
      const lowerCaseKeyword = keyword.toLowerCase();
      return [device.name, device.uniqueId, device.phone, device.model, device.contact].some(
        (s) => s && s.toLowerCase().includes(lowerCaseKeyword),
      );
    });
    switch (filterSort) {
      case 'name':
        filtered.sort((device1, device2) => device1.name.localeCompare(device2.name));
        break;
      case 'lastUpdate':
        filtered.sort((device1, device2) => {
          const time1 = device1.lastUpdate ? dayjs(device1.lastUpdate).valueOf() : 0;
          const time2 = device2.lastUpdate ? dayjs(device2.lastUpdate).valueOf() : 0;
          return time2 - time1;
        });
        break;
      default:
        break;
    }
    return filtered;
  }, [keyword, filter, filterSort, devices]);

  const filteredPositions = useMemo(
    () =>
      filterMap
        ? filteredDevices.flatMap((device) => (positions[device.id] ? [positions[device.id]] : []))
        : Object.values(positions),
    [filterMap, filteredDevices, positions],
  );

  return { filteredDevices, filteredPositions };
};

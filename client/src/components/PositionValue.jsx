import React from 'react';
import { useSelector } from 'react-redux';
import { Link } from '@mui/material';
//import { Link as RouterLink } from 'react-router-dom';
import {
  formatAlarm,
  formatAltitude,
  formatBoolean,
  formatCoordinate,
  formatCourse,
  formatDistance,
  formatNumber,
  formatNumericHours,
  formatPercentage,
  formatSpeed,
  formatTime,
} from '../common/formatter';

const PositionValue = ({ position, property, attribute }) => {
  const device = useSelector((state) => state.devices.items[position.deviceId]);

  const key = property || attribute;
  const value = property ? position[property] : position.attributes[attribute];

  const distanceUnit = 'm';
  const altitudeUnit = 'm';
  const speedUnit = 'm/s';
  const coordinateFormat = 'ddm';
  const hours12 = false;

  const formatValue = () => {
    switch (key) {
      case 'fixTime':
      case 'deviceTime':
      case 'serverTime':
        return formatTime(value, 'seconds', hours12);
      case 'latitude':
        return formatCoordinate('latitude', value, coordinateFormat);
      case 'longitude':
        return formatCoordinate('longitude', value, coordinateFormat);
      case 'speed':
        return formatSpeed(value, speedUnit);
      case 'course':
        return formatCourse(value);
      case 'altitude':
        return formatAltitude(value, altitudeUnit);
      case 'batteryLevel':
        return formatPercentage(value);
      case 'alarm':
        return formatAlarm(value);

      case 'gimbal':
        return value.toString();
      case 'obstacle_info':
        return value.toString();
      case 'odometer':
      case 'distance':
      case 'totalDistance':
        return formatDistance(value, distanceUnit);
      case 'hours':
        return formatNumericHours(value);
      default:
        if (typeof value === 'number') {
          return formatNumber(value);
        }
        if (typeof value === 'boolean') {
          return formatBoolean(value);
        }
        return value || '';
    }
  };

  switch (key) {
    case 'image':
    case 'video':
    case 'audio':
      return (
        <Link href={`/api/media/${device.uniqueId}/${value}`} target="_blank">
          {value}
        </Link>
      );
    case 'totalDistance':
    case 'hours':
    case 'address':
    case 'network':
      return '';
    default:
      return formatValue(value);
  }
};

export default PositionValue;

const speedConverter = (unit) => {
  switch (unit) {
    case 'kmh':
      return 1.852;
    case 'mph':
      return 1.15078;
    case 'kn':
    default:
      return 1;
  }
};

export const speedUnitString = (unit) => {
  switch (unit) {
    case 'kmh':
      return 'Km/h';
    case 'mph':
      return 'sharedMph';
    case 'kn':
    default:
      return 'm/s';
  }
};

export const speedFromKnots = (value, unit) => value * speedConverter(unit);

const distanceConverter = (unit) => {
  switch (unit) {
    case 'mi':
      return 0.000621371;
    case 'nmi':
      return 0.000539957;
    case 'km':
    default:
      return 0.001;
  }
};

export const distanceUnitString = (unit) => {
  switch (unit) {
    case 'mi':
      return 'Mi';
    case 'nmi':
      return 'Nmi';
    case 'km':
    default:
      return 'Km';
  }
};

export const distanceFromMeters = (value, unit) => value * distanceConverter(unit);

const altitudeConverter = (unit) => {
  switch (unit) {
    case 'ft':
      return 3.28084;
    case 'm':
    default:
      return 1;
  }
};

export const altitudeUnitString = (unit) => {
  switch (unit) {
    case 'ft':
      return 'sharedFeet';
    case 'm':
    default:
      return 'sharedMeters';
  }
};

export const altitudeFromMeters = (value, unit) => value * altitudeConverter(unit);

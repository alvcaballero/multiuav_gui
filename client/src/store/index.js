import { combineReducers, configureStore } from '@reduxjs/toolkit';
import { errorsReducer as errors } from './errors';
import { devicesReducer as devices } from './devices';
import { missionReducer as mission } from './mission';
import { sessionReducer as session } from './session';
import { eventsReducer as events } from './events';
import { geofencesReducer as geofences } from './geofences';

import throttleMiddleware from './throttleMiddleware';

const reducer = combineReducers({
  errors,
  devices,
  mission,
  session,
  events,
  geofences,
});

export { errorsActions } from './errors';
export { devicesActions } from './devices';
export { eventsActions } from './events';
export { missionActions } from './mission';
export { sessionActions } from './session';
export { geofencesActions } from './geofences';

const store = configureStore({
  reducer,
  middleware: (getDefaultMiddleware) => getDefaultMiddleware().concat(throttleMiddleware),
});

export default store;
export { store };
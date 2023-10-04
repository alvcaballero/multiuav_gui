import { combineReducers, configureStore } from '@reduxjs/toolkit';
import { errorsReducer as errors } from './errors';
import { devicesReducer as devices } from './devices';
import { dataReducer as data } from './data';
import { missionReducer as mission } from './mission';
import { sessionReducer as session } from './session';
import { eventsReducer as events } from './events';
import throttleMiddleware from './throttleMiddleware';

const reducer = combineReducers({
  errors,
  devices,
  data,
  events,
  mission,
  session,
});

export { errorsActions } from './errors';
export { devicesActions } from './devices';
export { dataActions } from './data';
export { eventsActions } from './events';
export { missionActions } from './mission';
export { sessionActions } from './session';

export default configureStore({
  reducer,
  middleware: (getDefaultMiddleware) => getDefaultMiddleware().concat(throttleMiddleware),
});

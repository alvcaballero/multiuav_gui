import { combineReducers, configureStore } from '@reduxjs/toolkit';
import { devicesReducer as devices } from './devices';
import { dataReducer as data } from './data';
import { missionReducer as mission } from './mission';
import throttleMiddleware from './throttleMiddleware';


const reducer = combineReducers({
    devices,
    data,
    mission,
  });

export { devicesActions } from './devices';
export { dataActions } from './data';
export { missionActions } from './mission';

export default configureStore({
    reducer,
    middleware: (getDefaultMiddleware) => getDefaultMiddleware().concat(throttleMiddleware),
  });
import { combineReducers, configureStore } from '@reduxjs/toolkit';
import { devicesReducer as devices } from './devices';
import { dataReducer as data } from './data';
import throttleMiddleware from './throttleMiddleware';


const reducer = combineReducers({
    devices,
    data,
  });

export { devicesActions } from './devices';
export { dataActions } from './data';

export default configureStore({
    reducer,
    middleware: (getDefaultMiddleware) => getDefaultMiddleware().concat(throttleMiddleware),
  });
import { createSlice } from '@reduxjs/toolkit';

const { reducer, actions } = createSlice({
  name: 'devices',
  initialState: {
    items:  {},
    selectedId: null,
    selectedIds: [],
  },
  reducers: {
    refresh(state, action) {
      console.log(action.payload);
      state.items = {};
      action.payload.forEach((item) => state.items[item.id] = item);
    },
    update1(state, action) {
      state.items[action.payload.id] = action.payload;
    },
    update(state, action) {
      action.payload.forEach((item) => state.items[item.id] = item);
    },
    select(state, action) {
      state.selectedId = action.payload;
    },
    selectId(state, action) {
      state.selectedId = action.payload;
      state.selectedIds = [state.selectedId];
    },
    selectIds(state, action) {
      state.selectedIds = action.payload;
      [state.selectedId] = state.selectedIds;
    },
    remove(state, action) {
      delete state.items[action.payload];
    },
    clear(state, action) {
      state.items={};
    },
    
  },
});

export { actions as devicesActions };
export { reducer as devicesReducer };
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
      state.items = {};
      action.payload.forEach((item) => state.items[item.id] = item);
    },
    update(state, action) {
      state.items[action.payload.id] = action.payload;
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
    
  },
});

export { actions as devicesActions };
export { reducer as devicesReducer };
import React, { useEffect, useRef, useState } from 'react';
import { useDispatch } from 'react-redux';
import { FixedSizeList } from 'react-window';
import AutoSizer from 'react-virtualized-auto-sizer';
import { devicesActions } from '../store';
import DeviceRow from './DeviceRow';

const liststyle ={
  maxHeight: '100%',
};
const listInnerStyle= {
  position: 'relative',
  margin: '15px 0px',
};

const Row = ({ index, style }) => (
  <div style={style}>Row {index}</div>
);

const DeviceList = ({ devices }) => {
  const dispatch = useDispatch();
  const listInnerEl = useRef(null);

  if (listInnerEl.current) {
    listInnerEl.current.className = listInnerStyle;
  }

  const [, setTime] = useState(Date.now());

  useEffect(() => {
    const interval = setInterval(() => setTime(Date.now()), 60000);
    return () => {
      clearInterval(interval);
    };
  }, []);


  return (
    <AutoSizer style={liststyle}>
      {({ height, width }) => (
        <FixedSizeList
          width={width}
          height={height}
          itemCount={devices.length}
          itemData={devices}
          itemSize={72}
          overscanCount={10}
          innerRef={listInnerEl}
        >
         {DeviceRow} 
        </FixedSizeList>
      )}
    </AutoSizer>
  );
};

export default DeviceList;

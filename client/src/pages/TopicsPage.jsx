import React from 'react';
import SquareMove from './SquareMove';
import { Divider } from '@mui/material';
const TopicsPage = () => {
  return (
    <div>
      <h3>Move the Square</h3>
      <SquareMove />
      <Divider></Divider>
      <SquareMove front_view={true} />
    </div>
  );
};

export default TopicsPage;

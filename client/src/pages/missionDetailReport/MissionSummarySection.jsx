import React from 'react';
import { Grid, Typography } from '@mui/material';

const MISSION_ITEMS = 'id,initTime,endTime,status';

const MissionSummarySection = ({ missions, formatValue, formatResult }) => (
  <div>
    <Typography variant="h4" gutterBottom>
      Resultado de la Misión
    </Typography>
    <Grid container spacing={2}>
      {MISSION_ITEMS.split(',')
        .filter((key) => missions.hasOwnProperty(key))
        .map((key) => (
          <Grid size={6} key={`ms${key}`}>
            <Typography variant="subtitle1" style={{ fontWeight: 'bold' }}>
              {key}
            </Typography>
            {key === 'status' ? (
              formatValue(missions, key)
            ) : (
              <Typography variant="body1">{formatValue(missions, key)}</Typography>
            )}
          </Grid>
        ))}
      <Grid size={6} key="msresult">
        <Typography variant="subtitle1" style={{ fontWeight: 'bold' }}>
          Results
        </Typography>
        {missions.results.flatMap((item) => formatResult(item))}
      </Grid>
    </Grid>
  </div>
);

export default MissionSummarySection;

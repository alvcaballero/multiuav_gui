import React from 'react';
import { Divider, Grid, ImageList, ImageListItem, Typography } from '@mui/material';

const ROUTE_ITEMS = 'id,initTime,endTime,status,deviceId,result';

const MissionRoutesSection = ({ routes, files, formatValue, onSelectFile }) => (
  <>
    {routes.map((route, routeIndex) => (
      <div key={`rt${routeIndex}`}>
        <Divider style={{ margin: '40px 0' }} />
        <Typography variant="h5" gutterBottom>
          {`Ruta-${routeIndex}`}
        </Typography>
        <Grid container spacing={2}>
          {ROUTE_ITEMS.split(',')
            .filter((key) => route.hasOwnProperty(key))
            .map((key) => (
              <Grid item xs={6} key={`rt${routeIndex}_${key}`}>
                <Typography variant="subtitle1" style={{ fontWeight: 'bold' }}>
                  {key}
                </Typography>
                {key === 'status' ? (
                  formatValue(route, key, 'route')
                ) : (
                  <Typography variant="body1">{formatValue(route, key)}</Typography>
                )}
              </Grid>
            ))}
        </Grid>
        {files && files.find((item) => item && item.routeId == route.id) && (
          <>
            <Typography variant="h6" gutterBottom style={{ marginTop: '20px' }}>
              Route files
            </Typography>
            <ImageList sx={{ width: '100%', height: 500 }} cols={3}>
              {files.flatMap((item) =>
                item.routeId == route.id && item.name.endsWith('.jpg')
                  ? [
                      <ImageListItem key={item.id}>
                        <button
                          type="button"
                          onClick={() => onSelectFile(item)}
                          style={{
                            padding: 0,
                            border: 'none',
                            background: 'none',
                            cursor: 'pointer',
                          }}
                        >
                          <img
                            src={`/api/files/download/${item.path}${item.name}`}
                            alt={item.name}
                            loading="lazy"
                          />
                        </button>
                      </ImageListItem>,
                    ]
                  : [],
              )}
            </ImageList>
          </>
        )}
      </div>
    ))}
  </>
);

export default MissionRoutesSection;

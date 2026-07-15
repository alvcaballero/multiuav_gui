import React from 'react';
import {
  Divider,
  Grid,
  ImageList,
  ImageListItem,
  ImageListItemBar,
  Typography,
} from '@mui/material';
import PlayArrowIcon from '@mui/icons-material/PlayArrow';
import { isImageFile, isVideoFile } from './fileKind';

const ROUTE_ITEMS = 'id,initTime,endTime,status,deviceId,result';

const fileButtonStyle = {
  display: 'block',
  width: '100%',
  height: '100%',
  padding: 0,
  border: 'none',
  background: 'none',
  cursor: 'pointer',
};

// Grid of clickable thumbnails. Videos have no <img> to show, so they render a
// play icon over a dark tile; both kinds open in the big viewer via onSelectFile.
const FileGrid = ({ items, onSelectFile }) => (
  <ImageList sx={{ width: '100%', height: 500 }} cols={3}>
    {items.map((item) => (
      <ImageListItem key={item.id}>
        <button type="button" onClick={() => onSelectFile(item)} style={fileButtonStyle}>
          {isVideoFile(item.name) ? (
            <div
              style={{
                width: '100%',
                height: '100%',
                display: 'flex',
                alignItems: 'center',
                justifyContent: 'center',
                background: '#1e1e1e',
              }}
            ></div>
          ) : (
            <img
              src={`/api/files/download/${item.path}${item.name}`}
              alt={item.name}
              loading="lazy"
              style={{ width: '100%', height: '100%', objectFit: 'cover' }}
            />
          )}
          <ImageListItemBar
            title={item.name}
            sx={{ '.MuiImageListItemBar-title': { fontSize: 11 } }}
          />
        </button>
      </ImageListItem>
    ))}
  </ImageList>
);

const MissionRoutesSection = ({ routes, files, formatValue, onSelectFile }) => (
  <>
    {routes.map((route, routeIndex) => {
      const routeFiles = (files ?? []).filter((item) => item && item.routeId == route.id);
      const images = routeFiles.filter((item) => isImageFile(item.name));
      const videos = routeFiles.filter((item) => isVideoFile(item.name));

      return (
        <div key={`rt${routeIndex}`}>
          <Divider style={{ margin: '40px 0' }} />
          <Typography variant="h5" gutterBottom>
            {`Ruta-${routeIndex}`}
          </Typography>
          <Grid container spacing={2}>
            {ROUTE_ITEMS.split(',')
              .filter((key) => route.hasOwnProperty(key))
              .map((key) => (
                <Grid size={6} key={`rt${routeIndex}_${key}`}>
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

          {images.length > 0 && (
            <>
              <Typography variant="h6" gutterBottom style={{ marginTop: '20px' }}>
                {`Imágenes (${images.length})`}
              </Typography>
              <FileGrid items={images} onSelectFile={onSelectFile} />
            </>
          )}

          {videos.length > 0 && (
            <>
              <Typography variant="h6" gutterBottom style={{ marginTop: '20px' }}>
                {`Videos (${videos.length})`}
              </Typography>
              <FileGrid items={videos} onSelectFile={onSelectFile} />
            </>
          )}
        </div>
      );
    })}
  </>
);

export default MissionRoutesSection;

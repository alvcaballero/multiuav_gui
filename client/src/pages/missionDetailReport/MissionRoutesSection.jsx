import React from 'react';
import {
  Chip,
  Divider,
  Grid,
  ImageList,
  ImageListItem,
  ImageListItemBar,
  Table,
  TableBody,
  TableCell,
  TableContainer,
  TableHead,
  TableRow,
  Typography,
} from '@mui/material';
import PlayArrowIcon from '@mui/icons-material/PlayArrow';
import { isImageFile, isVideoFile } from './fileKind';
import { formatTime } from '../../shared/formatter';
import { eventColor } from '../../shared/eventStyle';

const ROUTE_ITEMS = 'id,initTime,endTime,status,deviceId,result';

// Events carry deviceId + eventTime but no routeId, so a route's events are
// whatever that device logged while the route was running (endTime unset ⇒ still running).
const routeEvents = (events, route) => {
  const start = new Date(route.initTime).getTime();
  const end = route.endTime ? new Date(route.endTime).getTime() : Date.now();
  return (events ?? [])
    .filter((event) => event.deviceId === route.deviceId)
    .filter((event) => {
      const time = new Date(event.eventTime).getTime();
      return time >= start && time <= end;
    })
    .sort((a, b) => new Date(a.eventTime) - new Date(b.eventTime));
};

const RouteEvents = ({ items }) => (
  <TableContainer>
    <Table size="small">
      <TableHead>
        <TableRow>
          <TableCell>Type</TableCell>
          <TableCell>Action</TableCell>
          <TableCell>Time</TableCell>
          <TableCell>Message</TableCell>
        </TableRow>
      </TableHead>
      <TableBody>
        {items.map((event) => (
          <TableRow key={event.id} hover>
            <TableCell>
              <Chip
                label={event.type}
                size="small"
                sx={{ backgroundColor: eventColor(event.type), color: '#fff' }}
              />
            </TableCell>
            <TableCell>{event.attributes?.action ?? '—'}</TableCell>
            <TableCell>{formatTime(event.eventTime, 'minutes')}</TableCell>
            <TableCell>{event.attributes?.message}</TableCell>
          </TableRow>
        ))}
      </TableBody>
    </Table>
  </TableContainer>
);

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

const MissionRoutesSection = ({ routes, files, events, formatValue, onSelectFile }) => (
  <>
    {routes.map((route, routeIndex) => {
      const routeFiles = (files ?? []).filter((item) => item && item.routeId == route.id);
      const images = routeFiles.filter((item) => isImageFile(item.name));
      const videos = routeFiles.filter((item) => isVideoFile(item.name));
      const items = routeEvents(events, route);

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

          {items.length > 0 && (
            <>
              <Typography variant="h6" gutterBottom style={{ marginTop: '20px' }}>
                {`Eventos (${items.length})`}
              </Typography>
              <RouteEvents items={items} />
            </>
          )}

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

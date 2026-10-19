import React from 'react';
import { Card, CardContent, CardHeader, CardMedia, IconButton, Typography } from '@mui/material';
import CloseIcon from '@mui/icons-material/Close';
import { makeStyles } from 'tss-react/mui';
import { isVideoFile } from './fileKind';

const useStyles = makeStyles()(() => ({
  root_max: {
    pointerEvents: 'none',
    position: 'fixed',
    zIndex: 6,
    left: '50%',
    top: '8vh',
    transform: 'translateX(-50%)',
  },
  card: {
    pointerEvents: 'auto',
    maxWidth: '80vw',
  },
  media: {
    maxHeight: '70vh',
  },
}));

const ImageFull = ({ file, closecard }) => {
  const { classes } = useStyles();
  if (!file) return <div className={classes.root_max} />;

  const src = `/api/files/download/${file.path}${file.name}`;
  const isVideo = isVideoFile(file.name);

  return (
    <div className={classes.root_max}>
      <Card elevation={3} className={classes.card}>
        <CardHeader
          action={
            <IconButton aria-label="close" onClick={() => closecard()}>
              <CloseIcon />
            </IconButton>
          }
          title={file.name}
          subheader={file.date ?? ''}
        />
        {isVideo ? (
          // controls + preload metadata so the player is usable without pulling
          // the whole (large) video up front. The download endpoint streams the
          // file; seeking depends on server range support.
          <CardMedia
            component="video"
            className={classes.media}
            src={src}
            controls
            preload="metadata"
          />
        ) : (
          <CardMedia component="img" className={classes.media} alt={file.name} image={src} />
        )}
        <CardContent>
          <Typography variant="body2" color="text.secondary">
            {`Result: ${JSON.stringify(file.attributes)}`}
          </Typography>
        </CardContent>
      </Card>
    </div>
  );
};

export default ImageFull;

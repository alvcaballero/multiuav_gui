import React from 'react';
import { Card, CardContent, CardHeader, CardMedia, IconButton, Typography } from '@mui/material';
import CloseIcon from '@mui/icons-material/Close';
import { makeStyles } from 'tss-react/mui';

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
  },
}));

const ImageFull = ({ file, closecard }) => {
  const { classes } = useStyles();
  return (
    <div className={classes.root_max}>
      {file && (
        <Card elevation={3} className={classes.card}>
          <CardHeader
            action={
              <IconButton aria-label="close" onClick={() => closecard()}>
                <CloseIcon />
              </IconButton>
            }
            title={file.name}
            subheader="date:September 14, 2016"
          />
          <CardMedia
            component="img"
            alt={file.name}
            image={`/api/files/download/${file.path}${file.name}`}
          />
          <CardContent>
            <Typography variant="body2" color="text.secondary">
              Result:{JSON.stringify(file.attributes)}
            </Typography>
          </CardContent>
        </Card>
      )}
    </div>
  );
};

export default ImageFull;

import { forwardRef, useCallback, useEffect } from 'react';
import { makeStyles } from 'tss-react/mui';


import { IconButton, Typography } from '@mui/material';
import UploadFileIcon from '@mui/icons-material/UploadFile';

const useStyles = makeStyles()((theme) => ({
  input: {
    display: 'none',
  },

  parent: {
    alignItems: 'center',
    justifyContent: 'center',
    padding: '6px 16px',
  },
}));

const UploadButtons = ({ title, readFile, typefiles = '.yaml, .plan, .waypoint, .kml' }) => {
  const { classes } = useStyles();

  return (
    <div className={classes.parent}>
      <label htmlFor="upload-gpx">
        <input
          accept={typefiles}
          id="upload-gpx"
          type="file"
          className={classes.input}
          onChange={readFile}
        />
        {title ? (
          <Typography>{title}</Typography>
        ) : (
          <IconButton edge="end" component="span" onClick={() => {}}>
            <UploadFileIcon />
          </IconButton>
        )}
      </label>
    </div>
  );
};

export default UploadButtons;

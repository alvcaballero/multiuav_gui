import React, { useState, useEffect } from 'react';
import { useSelector } from 'react-redux';
import { useEffectAsync } from '../reactHelper';

import {
  Typography,
  Container,
  Paper,
  AppBar,
  Toolbar,
  IconButton,
  ImageList,
  ImageListItem,
  Card,
} from '@mui/material';
import CloseIcon from '@mui/icons-material/Close';
import makeStyles from '@mui/styles/makeStyles';
import ArrowBackIcon from '@mui/icons-material/ArrowBack';
import { useNavigate, useParams } from 'react-router-dom';

const useStyles = makeStyles((theme) => ({
  root: {
    height: '100%',
    display: 'flex',
    flexDirection: 'column',
  },
  content: {
    overflow: 'auto',
    paddingTop: theme.spacing(2),
    paddingBottom: theme.spacing(2),
  },
  buttons: {
    marginTop: theme.spacing(2),
    marginBottom: theme.spacing(2),
    display: 'flex',
    justifyContent: 'space-evenly',
    '& > *': {
      flexBasis: '33%',
    },
  },
  details: {
    display: 'flex',
    flexDirection: 'column',
    gap: theme.spacing(2),
    paddingBottom: theme.spacing(3),
  },
  card: {
    pointerEvents: 'auto',
  },
  media: {
    //height: theme.dimensions.popupImageHeight,
    width: theme.dimensions.popupMaxWidth,
    display: 'flex',
    justifyContent: 'flex-end',
    alignItems: 'flex-start',
    background: 'black',
  },
  media1: {
    //height: theme.dimensions.popupImageHeight
    width: '95vw',
    height: '90vh',
    display: 'flex',
    justifyContent: 'flex-end',
    alignItems: 'flex-start',
    background: 'black',
  },
  gruopBtn: {
    display: 'flex',
    right: '5px',
    height: '40px',
    position: 'absolute',
  },
  mediaButton: {
    color: theme.palette.colors.white,
    mixBlendMode: 'difference',
  },
  tittle: {
    display: 'block',
    width: 'calc( 100% - 60pt )',
    paddingLeft: '15pt',
    paddingTop: '10pt',
    paddingBottom: '10pt',
    textAlign: 'left',
  },
  header: {
    display: 'flex',
    justifyContent: 'space-between',
    alignItems: 'center',
    padding: theme.spacing(1, 1, 0, 2),
  },
  root_max: {
    pointerEvents: 'none',
    position: 'fixed',
    zIndex: 6,
    left: '51%',
    top: '5%',
    transform: 'translateX(-50%)',
  },
}));
const ImageFull = ({ file, closecard }) => {
  const classes = useStyles();

  return (
    <div className={classes.root_max}>
      <Card elevation={3} className={classes.card}>
        <div className={classes.gruopBtn}>
          <IconButton size="small" onClick={closecard}>
            <CloseIcon fontSize="small" className={classes.mediaButton} />
          </IconButton>
        </div>
        <div className={classes.tittle}>{file.name}</div>
        <img
          src={`/api/files/download/${file.route}${file.name}`}
          alt={file.name}
          loading="lazy"
          className={classes.media1}
        />
      </Card>
    </div>
  );
};

const MissionDetailReportPage = () => {
  const classes = useStyles();
  const navigate = useNavigate();
  const { id } = useParams();

  const [missions, setMissions] = useState(null);
  const [routes, setRoutes] = useState(null);
  const [files, setFiles] = useState(null);
  const [selectFile, setSelectFile] = useState(null);

  useEffectAsync(async () => {
    const response = await fetch(`/api/missions?id=${id}`);
    if (response.ok) {
      const myMissions = await response.json();
      setMissions(myMissions[0]);
    } else {
      throw Error(await response.text());
    }
    const response2 = await fetch(`/api/missions/routes?missionId=${id}`);
    if (response2.ok) {
      const myroutes = await response2.json();
      setRoutes(myroutes);
      console.log(myroutes);
    } else {
      throw Error(await response.text());
    }
    const response3 = await fetch(`/api/files/get?missionId=${id}`);
    if (response3.ok) {
      const myfiles = await response3.json();
      setFiles(myfiles);
      console.log(myfiles);
    } else {
      throw Error(await response.text());
    }
  }, []);

  return (
    <div className={classes.root}>
      <AppBar position="sticky" color="inherit">
        <Toolbar>
          <IconButton color="inherit" edge="start" sx={{ mr: 2 }} onClick={() => navigate(-1)}>
            <ArrowBackIcon />
          </IconButton>
          <Typography variant="h6">Mission - {id}</Typography>
        </Toolbar>
      </AppBar>
      <div className={classes.content}>
        <Container maxWidth="sm">
          <Paper>
            {missions && (
              <>
                <div>InitTime: {missions.initTime}</div>
                <div>EndTime: {missions.endTime}</div>
                <div>result: {JSON.stringify(missions.results)}</div>
              </>
            )}
            <div>space</div>
            {routes &&
              routes.map((route) => (
                <>
                  <div>InitTime: {route.initTime}</div>
                  <div>EndTime: {route.endTime}</div>
                  <div>device: {route.deviceId}</div>
                  <div>result: {JSON.stringify(route.results)}</div>
                  <ImageList sx={{ width: 500, height: 450 }} cols={3} rowHeight={164}>
                    {files &&
                      files.map((item) => {
                        if (item.routeId == route.id && item.name.includes('.jpg')) {
                          return (
                            <ImageListItem key={item.id}>
                              <img
                                src={`/api/files/download/${item.route}${item.name}`}
                                alt={item.name}
                                loading="lazy"
                                onClick={() => setSelectFile(item)}
                              />
                            </ImageListItem>
                          );
                        }
                        return <></>;
                      })}
                  </ImageList>
                </>
              ))}
            <div>space</div>
          </Paper>
        </Container>
      </div>
      {selectFile && <ImageFull file={selectFile} closecard={() => setSelectFile(null)} />}
    </div>
  );
};

export default MissionDetailReportPage;

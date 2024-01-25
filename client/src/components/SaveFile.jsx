import React, { useState } from 'react';
import { useSelector } from 'react-redux';
import {
  Card,
  CardContent,
  Typography,
  CardActions,
  IconButton,
  FormControl,
  InputLabel,
  MenuItem,
  Select,
  TextField,
  Button,
} from '@mui/material';
import CloseIcon from '@mui/icons-material/Close';
import palette from '../common/palette';
import makeStyles from '@mui/styles/makeStyles';
import YAML from 'yaml';

const useStyles = makeStyles((theme) => ({
  card: {
    pointerEvents: 'auto',
    width: '500px',
    height: '180px',
  },
  media: {
    height: theme.dimensions.popupImageHeight,
    display: 'flex',
    justifyContent: 'flex-end',
    alignItems: 'flex-start',
  },
  mediaButton: {
    color: theme.palette.colors.white,
    mixBlendMode: 'difference',
  },
  header: {
    display: 'flex',
    justifyContent: 'space-between',
    alignItems: 'center',
    padding: theme.spacing(1, 1, 0, 2),
  },
  content: {
    paddingTop: '15px',
    paddingBottom: theme.spacing(1),
  },
  negative: {
    color: theme.palette.colors.negative,
  },
  icon: {
    width: '25px',
    height: '25px',
    filter: 'brightness(0) invert(1)',
  },
  table: {
    '& .MuiTableCell-sizeSmall': {
      paddingLeft: 0,
      paddingRight: 0,
    },
  },
  cell: {
    borderBottom: 'none',
  },
  actions: {
    justifyContent: 'center',
  },
  root: {
    pointerEvents: 'none',
    position: 'fixed',
    zIndex: 6,
    left: '50%',
    top: '40%',
    transform: 'translateX(-50%)',
  },
}));

const SaveFile = ({ SetOpenSave, OpenSave }) => {
  const classes = useStyles();
  const mission = useSelector((state) => state.mission);

  const [fileType, setFileType] = useState('yaml');
  const [fileName, setFileName] = useState(mission.name);

  const SaveMission = async () => {
    let fileData;
    let archivetype = 'text/plain';
    if (fileType == 'yaml') {
      let yamlmission = { version: '3' }; //JSON.parse(JSON.stringify(mission));
      yamlmission['name'] = mission.name;
      yamlmission['route'] = mission.route;
      fileData = YAML.stringify(yamlmission);
    }
    if (fileType == 'kml') {
      archivetype = 'application/xml';
      let xmlString = '<?xml version="1.0" encoding="UTF-8"?>\n';
      xmlString += '<kml>\n';
      xmlString += '<Document>\n';
      mission.route.map((elem, elem_n, list) => {
        xmlString += `<Style id="sn_ylw-pushpin1${elem_n}">\n`;
        xmlString += '<LineStyle>\n';
        xmlString += `<color>ff${palette.colors_devices[elem_n].substr(-6)}</color>\n`;
        xmlString += '<width>3</width>\n';
        xmlString += '</LineStyle>\n';
        xmlString += '</Style>\n';
      });
      let init_elev = [];
      for (const initwp of mission.route) {
        //console.log(initwp.wp);
        if (initwp.wp.length) {
          let response = await fetch(
            `/api/map/elevation?locations=[[${initwp.wp[0].pos[0]},${initwp.wp[0].pos[1]}]]`
          );
          if (response.ok) {
            let myresponse = await response.json();
            //console.log(myresponse.results);
            init_elev.push(myresponse.results[0].elevation);
          } else {
            init_elev.push(0);
          }
        } else {
          init_elev.push(0);
        }
      }

      mission.route.map((elem, elem_n) => {
        xmlString += '<Placemark>\n';
        xmlString += `<name>${elem.uav}-${elem.name}</name>\n`;
        xmlString += `<styleUrl>#sn_ylw-pushpin1${elem_n}</styleUrl>\n`;
        xmlString += '<open>1</open>\n';
        xmlString += '<LineString>\n';
        let auxcoord = '';
        elem.wp.map((mywp) => {
          auxcoord =
            auxcoord +
            mywp.pos[1] +
            ',' +
            mywp.pos[0] +
            ',' +
            Number(+mywp.pos[2] + +init_elev[elem_n]) +
            ' ';
        });
        xmlString += '<tessellate>1</tessellate>\n';
        xmlString += '<altitudeMode>absolute</altitudeMode>\n'; // relativeToGround
        xmlString += '<coordinates>\n';
        xmlString += auxcoord + '\n';
        xmlString += '</coordinates>\n';
        xmlString += '</LineString>\n';
        xmlString += '</Placemark>\n';
      });

      xmlString += '</Document>\n';
      xmlString += '</kml>';
      fileData = xmlString;
    }
    if (fileType == 'plan') {
      archivetype = 'application/json';
      let planmission = { fileType: 'Plan' }; //JSON.parse(JSON.stringify(mission));
      planmission['geoFence'] = {
        circles: [],
        polygons: [],
        version: 2,
      };
      planmission['groundStation'] = 'QGroundControl';
      planmission['mission'] = {};
      planmission['mission']['cruiseSpeed'] = mission.route[0].attributes.idle_vel;
      planmission['mission']['firmwareType'] = 12;
      planmission['mission']['globalPlanAltitudeMode'] = 0;
      planmission['mission']['hoverSpeed'] = 5;
      planmission['mission']['items'] = [];
      mission.route.map((elem, elem_n, elem_list) => {
        elem.wp.map((mywp, mywp_n, mywp_list) => {
          let aux = {
            AMSLAltAboveTerrain: null,
            Altitude: mywp.pos[2],
            AltitudeMode: 1,
            autoContinue: true,
            command: 16,
            doJumpId: 1,
            frame: 3,
            params: [0, 0, 0, null, mywp.pos[0], mywp.pos[1], mywp.pos[2]],
            type: 'SimpleItem',
          };
          if (mywp_n == 0) {
            aux['command'] = 84;
            planmission['mission']['plannedHomePosition'] = [mywp.pos[0], mywp.pos[1], mywp.pos[2]];
          }
          mywp_n + 1 == mywp_list.length ? (aux['command'] = 84) : null;
          planmission['mission']['items'].push(aux);
        });
      });
      planmission['mission']['vehicleType'] = 20;
      planmission['mission']['version'] = 2;
      planmission['rallyPoints'] = {
        points: [],
        version: 2,
      };
      planmission['version'] = 1;
      fileData = JSON.stringify(planmission);
    }
    if (fileType == 'waypoint') {
      let xmlString = 'QGC WPL 110\n';
      mission.route.map((elem) => {
        elem.wp.map((mywp, mywp_n, mywp_list) => {
          if (mywp_n == 0) {
            xmlString += `${mywp_n}\t1\t0\t16\t0\t0\t0\t0\t${mywp.pos[0]}\t${mywp.pos[1]}\t${mywp.pos[2]}\t1\n`;
          } else {
            xmlString += `${mywp_n}\t0\t0\t16\t0\t0\t0\t0\t${mywp.pos[0]}\t${mywp.pos[1]}\t${mywp.pos[2]}\t1\n`;
          }
        });
      });
      fileData = xmlString;
    }

    const blob = new Blob([fileData], { type: archivetype });
    const url = URL.createObjectURL(blob);
    const link = document.createElement('a');
    link.download = mission.name + '.' + fileType;
    link.href = url;
    link.click();
  };

  const CloseSaveMission = () => {
    SetOpenSave(false);
  };

  return (
    <div className={classes.root}>
      <Card elevation={3} className={classes.card}>
        <div className={classes.header}>
          <Typography variant='body2' color='textSecondary'>
            Save Mission
          </Typography>
          <IconButton size='small' onClick={CloseSaveMission} onTouchStart={CloseSaveMission}>
            <CloseIcon fontSize='small' />
          </IconButton>
        </div>

        <CardContent className={classes.content}>
          <TextField
            required
            label='Name Mission'
            variant='standard'
            value={fileName ? fileName : ' '}
            onChange={(event) => setFileName(event.target.value)}
            style={{ width: '300px', marginRight: '5px' }}
          />

          <FormControl>
            <InputLabel>Type file mission</InputLabel>
            <Select
              label={'Type file mission'}
              value={fileType}
              onChange={(e) => setFileType(e.target.value)}
              sx={{ width: '120px' }}
            >
              {['yaml', 'kml', 'waypoint', 'plan'].map((item) => (
                <MenuItem key={item} value={item}>
                  {item}
                </MenuItem>
              ))}
            </Select>
          </FormControl>
        </CardContent>
        <CardActions classes={{ root: classes.actions }} disableSpacing>
          <Button onClick={SaveMission}>Save Mission</Button>
        </CardActions>
      </Card>
    </div>
  );
};
export default SaveFile;

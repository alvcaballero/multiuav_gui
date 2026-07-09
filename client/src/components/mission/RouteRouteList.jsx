import React, { Fragment, useEffect, useState } from 'react';
import { useDispatch, useSelector } from 'react-redux';
import { makeStyles } from 'tss-react/mui';

import {
  Box,
  Button,
  IconButton,
  TextField,
  Accordion,
  AccordionSummary,
  AccordionDetails,
  Typography,
} from '@mui/material';

import ExpandMore from '@mui/icons-material/ExpandMore';
import SelectField from '../../shared/components/SelectField';
import DeleteIcon from '@mui/icons-material/Delete';
import palette from '../../shared/palette';
import { map } from '../../map/core/mapInstance';
import WaypointRouteList from './WaypointRouteList';
import { missionActions } from '../../store';
import { applyUavTypeDefaults } from '../../store/mission';
import { useAsyncTask } from '../../reactHelper';
import { DEFAULT_UAV_TYPE } from './missionDefaults';

const useStyles = makeStyles()((theme) => ({
  list: {
    maxHeight: '100%',
    overflow: 'auto',
  },
  icon: {
    width: '25px',
    height: '25px',
    filter: 'brightness(0) invert(1)',
  },
  details: {
    display: 'flex',
    flexDirection: 'column',
    gap: theme.spacing(2),
    paddingBottom: theme.spacing(3),
  },
  attribute: {
    display: 'inline-block',
    width: '40%',
    textAlign: 'left',
    verticalAlign: 'middle',
  },
  attributeValue: {
    display: 'inline-block',
    width: '58%',
  },
  actionValue: {
    display: 'inline-block',
    width: '40%',
  },
}));

const AttributeField = ({ attrDef, value, uavType, onChange, disabled = false }) => {
  const { classes } = useStyles();

  if (attrDef.type === 'number') {
    // Clamp to the catalog [min,max] on blur so the committed value is firmware-valid.
    const clamp = (raw) => {
      let v = Number(raw);
      if (Number.isNaN(v)) v = attrDef.default ?? 0;
      if (attrDef.min != null && v < attrDef.min) v = attrDef.min;
      if (attrDef.max != null && v > attrDef.max) v = attrDef.max;
      return v;
    };
    return (
      <TextField
        fullWidth
        disabled={disabled}
        type="number"
        className={classes.attributeValue}
        defaultValue={value ?? attrDef.default ?? 0}
        slotProps={{
          htmlInput: { min: attrDef.min, max: attrDef.max, step: attrDef.step },
          input: attrDef.unit ? { endAdornment: attrDef.unit } : undefined,
        }}
        onBlur={(e) => onChange(clamp(e.target.value))}
      />
    );
  }

  return (
    <div className={classes.attributeValue}>
      <SelectField
        emptyValue={null}
        fullWidth={true}
        disabled={disabled}
        value={value ?? attrDef.default ?? 0}
        onChange={(e) => onChange(e.target.value)}
        endpoint={`/api/category/atributesparam/${uavType}/${attrDef.id}`}
        keyGetter={(it) => it.id}
        titleGetter={(it) => it.name}
      />
    </div>
  );
};

const RouteOptions = ({ index, route, uavType, NoEdit = false }) => {
  const { classes } = useStyles();
  const dispatch = useDispatch();
  const [expand, setExpand] = useState(false);
  const [attrDefs, setAttrDefs] = useState([]);
  const resolvedUavType = uavType || DEFAULT_UAV_TYPE;

  useAsyncTask(async () => {
    const response = await fetch(`/api/category/attributeslist/${resolvedUavType}`);
    if (response.ok) setAttrDefs(await response.json());
  }, [resolvedUavType]);

  const handleAttributeChange = (attrId, value) => {
    dispatch(missionActions.updateRouteAttribute({ index, attribute: attrId, value }));
  };

  return (
    <Accordion expanded={expand} onChange={() => setExpand(!expand)}>
      <AccordionSummary expandIcon={<ExpandMore />}>
        <Typography sx={{ width: '53%', flexShrink: 0 }}>Route Attributes</Typography>
      </AccordionSummary>
      <AccordionDetails className={classes.details}>
        {route.attributes &&
          attrDefs.map((attrDef) => (
            <div key={attrDef.id}>
              <Typography variant="subtitle1" className={classes.attribute}>
                {attrDef.name}
              </Typography>
              <AttributeField
                attrDef={attrDef}
                value={route.attributes[attrDef.id]}
                uavType={resolvedUavType}
                onChange={(value) => handleAttributeChange(attrDef.id, value)}
                disabled={NoEdit}
              />
            </div>
          ))}
      </AccordionDetails>
    </Accordion>
  );
};

const RouteRoutesList = ({ index, route, expanded, setExpanded, NoEdit = false }) => {
  const { classes } = useStyles();
  const dispatch = useDispatch();
  const devices = useSelector((state) => state.devices.items);
  const positions = useSelector((state) => state.session.positions);
  const idleVel = useSelector((state) => state.mission.route[index]?.attributes?.idle_vel);
  const matchedDevice = Object.values(devices).find((device) => device.name === route.uav);
  const routeUAV = matchedDevice ? matchedDevice.id : null;
  const matchedCategory = matchedDevice?.category;

  useEffect(() => {
    if (NoEdit) return;
    if (matchedCategory && route.uav_type !== matchedCategory) {
      dispatch(missionActions.updateRoute({ index, field: 'uav_type', value: matchedCategory }));
      dispatch(applyUavTypeDefaults({ routeIndex: index, uavType: matchedCategory }));
    }
  }, [NoEdit, matchedCategory, index, route.uav_type, dispatch]);

  const handleAddWaypoint = (index_route, index_wp) => {
    let center = map.getCenter();
    const mywp = { pos: [center.lat, center.lng, 5], action: {} };
    dispatch(
      missionActions.addWaypoint({ routeIndex: index_route, waypoint: mywp, insertAt: index_wp }),
    );
  };

  const handleRecordWp = (index_route) => {
    if (positions[routeUAV]) {
      let altitude = positions[routeUAV].altitude;
      if (positions[routeUAV].attributes?.home) {
        altitude = altitude - positions[routeUAV].attributes.home[2];
      }

      const mywp = {
        pos: [positions[routeUAV].latitude, positions[routeUAV].longitude, altitude],
        action: { yaw: 0, gimbal: 0 },
      };

      if (positions[routeUAV].course !== undefined) {
        mywp.action.yaw =
          +Number(positions[routeUAV].course).toFixed(2) <= 180
            ? +Number(positions[routeUAV].course).toFixed(2)
            : -360 + +Number(positions[routeUAV].course).toFixed(2);
      }
      if (positions[routeUAV].attributes?.gimbal) {
        mywp.action.gimbal = Number(positions[routeUAV].attributes.gimbal[0]);
      }

      dispatch(missionActions.addWaypoint({ routeIndex: index_route, waypoint: mywp }));
    }
  };

  const handleRemoveRoute = (index_route) => {
    dispatch(missionActions.deleteRoute(index_route));
  };

  const handleRouteFieldChange = (field, value) => {
    dispatch(missionActions.updateRoute({ index, field, value }));
  };

  const routeKey = `r${index}`;
  const isRouteOpen = expanded?.startsWith(routeKey);

  const handleChange_route = (_, isExpanded) => {
    setExpanded(isExpanded ? routeKey : null);
  };

  return (
    <Accordion expanded={!!isRouteOpen} onChange={handleChange_route}>
      <AccordionSummary expandIcon={<ExpandMore />} component="div">
        <Typography
          sx={{
            width: '33%',
            flexShrink: 0,
            color: palette.colors_devices[route.id],
          }}
        >
          {'Rute ' + index}
        </Typography>
        <Typography sx={{ color: 'text.secondary' }}>{route.name + '- ' + route.uav}</Typography>
        {!NoEdit && (
          <IconButton
            sx={{ py: 0, pr: 2, marginLeft: 'auto' }}
            onClick={(e) => {
              e.stopPropagation();
              handleRemoveRoute(index);
            }}
          >
            <DeleteIcon />
          </IconButton>
        )}
      </AccordionSummary>
      <AccordionDetails className={classes.details}>
        {isRouteOpen && (
          <Fragment>
            <TextField
              required
              disabled={NoEdit}
              label="Route Name"
              variant="standard"
              value={route.name || ''}
              onChange={(e) => handleRouteFieldChange('name', e.target.value)}
            />

            <TextField
              required
              disabled={NoEdit}
              label="UAV id"
              variant="standard"
              value={route.uav || 'uav_'}
              onChange={(e) => handleRouteFieldChange('uav', e.target.value)}
            />

            <SelectField
              emptyValue={null}
              disabled={NoEdit}
              value={route.uav_type || ''}
              onChange={(e) => handleRouteFieldChange('uav_type', e.target.value)}
              endpoint="/api/category"
              keyGetter={(it) => it}
              titleGetter={(it) => it}
              label={'Type UAV mission'}
              style={{ display: 'inline', width: '200px' }}
            />

            <RouteOptions index={index} route={route} uavType={route.uav_type} NoEdit={NoEdit} />

            <Typography variant="subtitle1">Waypoints</Typography>
            {route.wp.map((waypoint, index_wp) => (
              <WaypointRouteList
                key={`wp-${index}-${index_wp}`}
                indexWp={index_wp}
                routeIndex={index}
                waypoint={waypoint}
                idleVel={idleVel}
                uavType={route.uav_type}
                expanded={expanded}
                setExpanded={setExpanded}
                onAddWaypoint={handleAddWaypoint}
                NoEdit={NoEdit}
              />
            ))}

            {!NoEdit && (
              <Box sx={{ textAlign: 'center' }}>
                <Button
                  variant="contained"
                  size="large"
                  sx={{ width: '60%', flexShrink: 0 }}
                  style={{ marginTop: '15px' }}
                  onClick={() => handleAddWaypoint(index, -1)}
                >
                  Add Waypoint
                </Button>
                <Button
                  variant="contained"
                  color="secondary"
                  size="large"
                  disabled={routeUAV == null}
                  sx={{ width: '30%', flexShrink: 0 }}
                  style={{ marginTop: '15px' }}
                  onClick={() => handleRecordWp(index)}
                >
                  Record
                </Button>
              </Box>
            )}
          </Fragment>
        )}
      </AccordionDetails>
    </Accordion>
  );
};

export default RouteRoutesList;

import React, { useState } from "react";
import { useDispatch, useSelector } from "react-redux";
import { useNavigate } from "react-router-dom";
import Draggable from "react-draggable";
import {
  Card,
  CardContent,
  Typography,
  CardActions,
  IconButton,
  Table,
  TableBody,
  TableRow,
  TableCell,
  Menu,
  MenuItem,
  CardMedia,
  Button,
} from "@mui/material";
import CloseIcon from "@mui/icons-material/Close";
import ReplayIcon from "@mui/icons-material/Replay";
import PublishIcon from "@mui/icons-material/Publish";
import EditIcon from "@mui/icons-material/Edit";
import DeleteIcon from "@mui/icons-material/Delete";
import PendingIcon from "@mui/icons-material/Pending";

import PositionValue from "./PositionValue";
import RemoveDialog from "./RemoveDialog";
import makeStyles from "@mui/styles/makeStyles";
import { devicesActions } from "../store";
import { Alarm } from "@mui/icons-material";

const useStyles = makeStyles((theme) => ({
  card: {
    pointerEvents: "auto",
    width: theme.dimensions.popupMaxWidth,
  },
  media: {
    height: theme.dimensions.popupImageHeight,
    display: "flex",
    justifyContent: "flex-end",
    alignItems: "flex-start",
  },
  mediaButton: {
    color: theme.palette.colors.white,
    mixBlendMode: "difference",
  },
  header: {
    display: "flex",
    justifyContent: "space-between",
    alignItems: "center",
    padding: theme.spacing(1, 1, 0, 2),
  },
  content: {
    paddingTop: theme.spacing(1),
    paddingBottom: theme.spacing(1),
  },
  negative: {
    color: theme.palette.colors.negative,
  },
  icon: {
    width: "25px",
    height: "25px",
    filter: "brightness(0) invert(1)",
  },
  table: {
    "& .MuiTableCell-sizeSmall": {
      paddingLeft: 0,
      paddingRight: 0,
    },
  },
  cell: {
    borderBottom: "none",
  },
  actions: {
    justifyContent: "space-between",
  },
  root: ({ desktopPadding }) => ({
    pointerEvents: "none",
    position: "fixed",
    zIndex: 5,
    left: "50%",
    [theme.breakpoints.up("md")]: {
      left: `calc(50% + ${desktopPadding} / 2)`,
      bottom: theme.spacing(3),
    },
    [theme.breakpoints.down("md")]: {
      left: "50%",
      bottom: `calc(${theme.spacing(3)} + ${
        theme.dimensions.bottomBarHeight
      }px)`,
    },
    transform: "translateX(-50%)",
  }),
}));

const StatusRow = ({ name, content }) => {
  const classes = useStyles();

  return (
    <TableRow>
      <TableCell className={classes.card}>
        <Typography variant="body2">{name}</Typography>
      </TableCell>
      <TableCell className={classes.cell}>
        <Typography variant="body2" color="textSecondary">
          {content}
        </Typography>
      </TableCell>
    </TableRow>
  );
};

const StatusCard = ({
  deviceId,
  position,
  onClose,
  disableActions,
  desktopPadding = 0,
}) => {
  const classes = useStyles();
  const navigate = useNavigate();
  const dispatch = useDispatch();

  //const deviceReadonly = useDeviceReadonly();

  const device = useSelector((state) => state.devices.items[deviceId]);

  const deviceImage = device?.attributes?.deviceImage;

  const positionItems = "speed,course,batteryLevel,gimbal";

  const [anchorEl, setAnchorEl] = useState(null);

  const [removing, setRemoving] = useState(false);

  const serverCommand = async (uav_id, command) => {
    //event.preventDefault();
    console.log("send command ");
    console.log(uav_id);
    try {
      const response = await fetch("/api/commands", {
        method: "POST",
        body: JSON.stringify({ uav_id: uav_id, description: command }),
        headers: {
          "Content-Type": "application/json",
        },
      });
      if (response.ok) {
        let myresponse = await response.json();
        console.log(myresponse);
      } else {
        console.log("Error1:" + response);
        throw Error(await response.text());
      }
    } catch (error) {
      console.log("Error2:" + error);
    }
  };
  const servercommandmission = async (uav_id) => {
    //event.preventDefault();
    console.log("command mission " + uav_id);
    try {
      const response = await fetch(`/api/commandmission/${uav_id}`, {
        method: "POST",
        body: JSON.stringify({ uav_id: uav_id }),
        headers: {
          "Content-Type": "application/json",
        },
      });
      if (response.ok) {
        let myresponse = await response.json();
        if (myresponse.state === "connect") {
          notification("success", myresponse.msg);
        }
        if (myresponse.state === "fail") {
          notification("danger", myresponse.msg);
        }
        console.log(myresponse);
      } else {
        throw Error(await response.text());
      }
    } catch (error) {}
  };

  const handleRemove = async (removed) => {
    if (removed) {
      const response = await fetch("/api/devices");
      if (response.ok) {
        let myresponse = await response.json();
        dispatch(devicesActions.refresh(Object.values(myresponse)));
      } else {
        throw Error(await response.text());
      }
    }
    setRemoving(false);
  };

  return (
    <>
      <div className={classes.root}>
        {device && (
          <Card elevation={3} className={classes.card}>
            {deviceImage ? (
              <CardMedia
                className={classes.media}
                image={`/api/media/${device.uniqueId}/${deviceImage}`}
              >
                <IconButton
                  size="small"
                  onClick={onClose}
                  onTouchStart={onClose}
                >
                  <CloseIcon fontSize="small" className={classes.mediaButton} />
                </IconButton>
              </CardMedia>
            ) : (
              <div className={classes.header}>
                <Typography variant="body2" color="textSecondary">
                  {device.name}
                </Typography>
                <IconButton
                  size="small"
                  onClick={onClose}
                  onTouchStart={onClose}
                >
                  <CloseIcon fontSize="small" />
                </IconButton>
              </div>
            )}
            {position && (
              <CardContent className={classes.content}>
                <Table size="small" classes={{ root: classes.table }}>
                  <TableBody>
                    {position.hasOwnProperty("latitude") && (
                      <StatusRow
                        key="latitude"
                        name={"Position"}
                        content={
                          <a
                            href={
                              "https://www.google.com/maps?q=" +
                              position.latitude +
                              "," +
                              position.longitude
                            }
                            target="_blank"
                          >
                            {"[" +
                              position.latitude.toFixed(6) +
                              "," +
                              position.longitude.toFixed(6) +
                              "]"}
                          </a>
                        }
                      />
                    )}

                    {positionItems
                      .split(",")
                      .filter(
                        (key) =>
                          position.hasOwnProperty(key) ||
                          position.attributes.hasOwnProperty(key)
                      )
                      .map((key) => (
                        <StatusRow
                          key={key}
                          name={key}
                          content={
                            <PositionValue
                              position={position}
                              property={
                                position.hasOwnProperty(key) ? key : null
                              }
                              attribute={
                                position.hasOwnProperty(key) ? null : key
                              }
                            />
                          }
                        />
                      ))}

                    {position.attributes.hasOwnProperty("alarm") && (
                      <StatusRow
                        key="alarm1"
                        name={"Alarm " + position.attributes.alarm}
                        content={
                          <div style={{ width: "100%" }}>
                            <Button
                              variant="contained"
                              size="small"
                              color="primary"
                              style={{ margin: "1px", display: "inline-block" }}
                              onClick={() => serverCommand(device.id, "threat")}
                            >
                              Validate
                            </Button>
                            <Button
                              variant="contained"
                              size="small"
                              color="secondary"
                              style={{ margin: "1px", display: "inline-block" }}
                            >
                              Dismiss
                            </Button>
                          </div>
                        }
                      />
                    )}
                  </TableBody>
                </Table>
              </CardContent>
            )}
            <CardActions classes={{ root: classes.actions }} disableSpacing>
              <IconButton
                color="secondary"
                onClick={(e) => setAnchorEl(e.currentTarget)}
                disabled={!position}
              >
                <PendingIcon />
              </IconButton>
              <IconButton
                onClick={() => serverCommand(device.id, "sincronize")}
                //disabled={disableActions || !position}
              >
                <ReplayIcon />
              </IconButton>
              <IconButton
                onClick={() => servercommandmission(device.id)}
                disabled={disableActions}
              >
                <PublishIcon />
              </IconButton>
              <IconButton
              //onClick={() => navigate(`/settings/device/${deviceId}`)}
              //disabled={disableActions || deviceReadonly}
              >
                <EditIcon />
              </IconButton>
              <IconButton
                onClick={() => setRemoving(true)}
                //disabled={disableActions || deviceReadonly}
                className={classes.negative}
              >
                <DeleteIcon />
              </IconButton>
            </CardActions>
          </Card>
        )}
      </div>
      {position && (
        <Menu
          anchorEl={anchorEl}
          open={Boolean(anchorEl)}
          onClose={() => setAnchorEl(null)}
        >
          <MenuItem onClick={() => navigate(`/device/${deviceId}`)}>
            <Typography color="secondary">Detalle Dispositivo</Typography>
          </MenuItem>
          <MenuItem
            component="a"
            target="_blank"
            href={`https://www.google.com/maps/search/?api=1&query=${position.latitude}%2C${position.longitude}`}
          >
            {"linkGoogleMaps"}
          </MenuItem>
          <MenuItem
            component="a"
            target="_blank"
            href={`http://maps.apple.com/?ll=${position.latitude},${position.longitude}`}
          >
            {"linkAppleMaps"}
          </MenuItem>
          <MenuItem
            component="a"
            target="_blank"
            href={`https://www.google.com/maps/@?api=1&map_action=pano&viewpoint=${position.latitude}%2C${position.longitude}&heading=${position.course}`}
          >
            {"linkStreetView"}
          </MenuItem>
        </Menu>
      )}
      {
        <RemoveDialog
          open={removing}
          endpoint="devices"
          itemId={deviceId}
          onResult={(removed) => handleRemove(removed)}
        />
      }
    </>
  );
};

export default StatusCard;

import React, { useContext, useState, Fragment } from "react";
import { RosContext } from "./RosControl";
import makeStyles from "@mui/styles/makeStyles";
import CloseIcon from "@mui/icons-material/Close";
import ExpandMoreIcon from "@mui/icons-material/ExpandMore";
import {
  Divider,
  Card,
  IconButton,
  MenuItem,
  Button,
  Select,
  TextField,
  FormControl,
  InputLabel,
  Accordion,
  AccordionSummary,
  AccordionDetails,
  Typography,
  FormGroup,
  FormControlLabel,
  Checkbox,
} from "@mui/material";
import DeleteIcon from "@mui/icons-material/Delete";

const useStyles = makeStyles((theme) => ({
  card: {
    pointerEvents: "auto",
    display: "block",
    width: "400px",
    height: "70vh",
    transitionDuration: "0.3s",
    overflowY: "auto",
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
  root: {
    pointerEvents: "none",
    position: "fixed",
    zIndex: 6,
    left: "50%",
    top: "20%",
    transform: "translateX(-50%)",
  },
  button: {
    width: "80%",
    paddingBottom: "10pt",
    paddingTop: "10pt",
  },
  formControl: {
    margin: theme.spacing(1),
    gap: theme.spacing(2),
    paddingBottom: "20pt",
  },
  inputtext: {
    paddingBottom: "20px",
  },
  details: {
    display: "flex",
    flexDirection: "column",
    gap: theme.spacing(2),
    paddingBottom: theme.spacing(3),
  },
  title: {
    display: "block",
    width: "calc( 100% - 60pt )",
    paddingLeft: "15pt",
    paddingTop: "10pt",
    paddingBottom: "20pt",
    textAlign: "left",
  },
  closeBtn: {
    display: "flex",
    right: "5px",
    height: "35px",
    position: "absolute",
  },
  attributeName: {
    display: "inline-block",
    width: "40%",
    textAlign: "left",
    verticalAlign: "middle",
  },
  attributeValue: {
    display: "inline-block",
    width: "58%",
  },
  actionValue: {
    display: "inline-block",
    width: "40%",
  },
}));

export const Adduav = ({ SetAddUAVOpen }) => {
  const classes = useStyles();
  const rosContex = useContext(RosContext);
  const [item, setItem] = useState({
    name: "uav_",
    category: "dji",
    ip: "10.42.0.42",
    camera: [],
  });

  function closeAddUav() {
    SetAddUAVOpen(false);
  }
  function AddnewUAV() {
    console.log("add uav-" + item.name + "-" + item.category);
    console.log(item);
    rosContex.connectAddUav(item);
    SetAddUAVOpen(false);
  }
  const Remove_camera = (index) => {
    let auxcamera = JSON.parse(JSON.stringify(item.camera));
    auxcamera.splice(index, 1);
    setItem({ ...item, camera: auxcamera });
  };
  function addNewcamera() {
    let auxcamera = JSON.parse(JSON.stringify(item.camera));
    auxcamera.push({ type: "WebRTC", source: "" });
    setItem({ ...item, camera: auxcamera });
  }

  return (
    <div className={classes.root}>
      <Card elevation={3} className={classes.card}>
        <div className={classes.closeBtn}>
          <IconButton
            size="small"
            onClick={closeAddUav}
            onTouchStart={closeAddUav}
          >
            <CloseIcon fontSize="small" className={classes.mediaButton} />
          </IconButton>
        </div>

        <b>
          <div className={classes.title}>Add device </div>
        </b>

        <Accordion defaultExpanded>
          <AccordionSummary expandIcon={<ExpandMoreIcon />}>
            <Typography variant="subtitle1">Datos UAV</Typography>
          </AccordionSummary>
          <AccordionDetails className={classes.details}>
            <FormGroup style={{ margin: "20px" }}>
              <TextField
                required
                label="UAV ID"
                name="uavid"
                value={item.name}
                onChange={(event) =>
                  setItem({ ...item, name: event.target.value })
                }
                className={classes.inputtext}
              />

              <FormControl style={{ paddingBottom: "20px" }} variant="outlined">
                <InputLabel id="demo-simple-select-outlined-label">
                  UAV type
                </InputLabel>
                <Select
                  label="uavtipe"
                  labelId="demo-simple-select-outlined-label"
                  id="demo-simple-select-outlined"
                  value={item.category}
                  onChange={(event) =>
                    setItem({ ...item, category: event.target.value })
                  }
                >
                  <MenuItem value="dji">DJI</MenuItem>
                  <MenuItem value="dji_M300">DJI M300</MenuItem>
                  <MenuItem value="px4">PX4</MenuItem>
                  <MenuItem value="fuvex">Fuvex</MenuItem>
                  <MenuItem value="catec">Catec</MenuItem>
                </Select>
              </FormControl>
              <TextField
                label="ip"
                value={item.ip}
                onChange={(event) =>
                  setItem({ ...item, ip: event.target.value })
                }
              />
            </FormGroup>
          </AccordionDetails>
        </Accordion>
        {item && (
          <Accordion>
            <AccordionSummary expandIcon={<ExpandMoreIcon />}>
              <Typography variant="subtitle1">Camera</Typography>
            </AccordionSummary>
            <AccordionDetails className={classes.details}>
              {item.camera &&
                item.camera.map((action_key, index_ac, list_ac) => (
                  <Fragment key={"fragment-action-" + index_ac}>
                    <Typography
                      variant="subtitle1"
                      className={classes.attributeName}
                    >
                      {"Camera " + index_ac}
                    </Typography>
                    <div>
                      <FormControl variant="outlined">
                        <InputLabel id="demo-simple-select-outlined-label">
                          CameraType
                        </InputLabel>
                        <Select
                          labelId="demo-simple-select-outlined-label"
                          id="demo-simple-select-outlined"
                          value={action_key["type"]}
                          label="type"
                          onChange={(e) =>
                            setItem({
                              ...item,
                              camera: item.camera.map((cam, cam_ind) => {
                                let mycam = JSON.parse(JSON.stringify(cam));
                                index_ac == cam_ind
                                  ? (mycam["type"] = e.target.value)
                                  : null;
                                return mycam;
                              }),
                            })
                          }
                        >
                          <MenuItem value="WebRTC">WebRTC</MenuItem>
                          <MenuItem value="Websocket">Websocket</MenuItem>
                        </Select>
                      </FormControl>
                      <div className={classes.actionValue}>
                        <TextField
                          required
                          fullWidth={true}
                          value={action_key["source"]}
                          onChange={(e) =>
                            setItem({
                              ...item,
                              camera: item.camera.map((cam, cam_ind) => {
                                let mycam = JSON.parse(JSON.stringify(cam));
                                index_ac == cam_ind
                                  ? (mycam["source"] = e.target.value)
                                  : null;
                                return mycam;
                              }),
                            })
                          }
                        />
                      </div>
                      <IconButton
                        sx={{
                          py: 0,
                          pr: 2,
                          marginLeft: "auto",
                        }}
                        onClick={() => Remove_camera(index_ac)}
                        className={classes.negative}
                      >
                        <DeleteIcon />
                      </IconButton>
                    </div>
                    <Divider></Divider>
                  </Fragment>
                ))}

              <Button
                variant="contained"
                size="large"
                sx={{ width: "80%", flexShrink: 0 }}
                style={{ marginTop: "15px" }}
                onClick={addNewcamera}
              >
                Add camera source
              </Button>
            </AccordionDetails>
          </Accordion>
        )}

        <div
          style={{
            paddingBottom: "20pt",
            paddingTop: "20pt",
            display: "flex",
            justifyContent: "center",
          }}
        >
          <Button
            className={classes.button}
            variant="contained"
            onClick={AddnewUAV}
          >
            ADD
          </Button>
        </div>
      </Card>
    </div>
  );
};

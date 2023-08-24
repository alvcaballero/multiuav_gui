import React, { useState } from "react";
import { useSelector } from "react-redux";

import {
  Typography,
  Container,
  Paper,
  AppBar,
  Toolbar,
  IconButton,
  Table,
  TableHead,
  TableRow,
  TableCell,
  TableBody,
} from "@mui/material";
import makeStyles from "@mui/styles/makeStyles";
import ArrowBackIcon from "@mui/icons-material/ArrowBack";
import { useNavigate, useParams } from "react-router-dom";
import { useEffectAsync } from "../reactHelper";
import { prefixString } from "../common/stringUtils";

const useStyles = makeStyles((theme) => ({
  root: {
    height: "100%",
    display: "flex",
    flexDirection: "column",
  },
  content: {
    overflow: "auto",
    paddingTop: theme.spacing(2),
    paddingBottom: theme.spacing(2),
  },
}));

const DevicePage = () => {
  const classes = useStyles();
  const navigate = useNavigate();

  const { id } = useParams();

  const [item, setItem] = useState();

  useEffectAsync(async () => {
    if (id) {
      const response = await fetch(`/api/positions?id=${id}`);
      if (response.ok) {
        const positions = await response.json();
        if (positions.length > 0) {
          setItem(positions[0]);
        }
      } else {
        throw Error(await response.text());
      }
    }
  }, [id]);

  const deviceName = useSelector((state) => {
    if (item) {
      const device = state.devices.items[item.deviceId];
      if (device) {
        return device.name;
      }
    }
    return null;
  });

  return (
    <div className={classes.root}>
      <AppBar position="sticky" color="inherit">
        <Toolbar>
          <IconButton
            color="inherit"
            edge="start"
            sx={{ mr: 2 }}
            onClick={() => navigate(-1)}
          >
            <ArrowBackIcon />
          </IconButton>
          <Typography variant="h6">{deviceName}</Typography>
        </Toolbar>
      </AppBar>
      <div className={classes.content}>
        <Container maxWidth="sm">
          <Paper>
            <Table>
              <TableHead>
                <TableRow>
                  <TableCell>stateName</TableCell>
                  <TableCell>sharedName</TableCell>
                  <TableCell>stateValue</TableCell>
                </TableRow>
              </TableHead>
              <TableBody>
                {item &&
                  Object.getOwnPropertyNames(item)
                    .filter((it) => it !== "attributes")
                    .map((property) => (
                      <TableRow key={property}>
                        <TableCell>{property}</TableCell>
                        <TableCell>
                          <strong>{prefixString("position", property)}</strong>
                        </TableCell>
                        <TableCell>{property} </TableCell>
                      </TableRow>
                    ))}
                {item &&
                  Object.getOwnPropertyNames(item.attributes).map(
                    (attribute) => (
                      <TableRow key={attribute}>
                        <TableCell>{attribute}</TableCell>
                        <TableCell>
                          <strong>
                            {prefixString("position", attribute) ||
                              prefixString("device", attribute)}
                          </strong>
                        </TableCell>
                        <TableCell>{attribute} </TableCell>
                      </TableRow>
                    )
                  )}
              </TableBody>
            </Table>
          </Paper>
        </Container>
      </div>
    </div>
  );
};

export default DevicePage;

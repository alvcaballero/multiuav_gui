import { forwardRef, useCallback, useEffect } from "react";
import { makeStyles } from 'tss-react/mui';


import { Card, IconButton, CardMedia, Typography } from "@mui/material";
import CloseIcon from "@mui/icons-material/Close";

const useStyles = makeStyles()((theme) => ({
  title: {
    fontWeight: 700,
    fontSize: "16px",
    textAlign: "left",
    marginTop: 0,
    marginBottom: "6px",
    width: "300px",
    height: "18px",
  },
  mediaButton: {
    color: "#fff",
    mixBlendMode: "difference",
    right: 0,
    top: 0,
  },
  description: {
    margin: 0,
    textAlign: "left",
    fontSize: "12px",
    paddingLeft: "1rem",
  },
  toast: {
    height: "60px",
    width: "365px",
    color: "#fff",
    marginBottom: "1rem",
  },
  header: {
    display: "flex",
    justifyContent: "space-between",
    alignItems: "center",
    padding: theme.spacing(0.5, 1, 0, 2),
  },
  card: {
    pointerEvents: "auto",
    height: "60px",
    width: "365px",
    color: "#fff",
    marginBottom: "1rem",
  },
  root: {
    pointerEvents: "none",
    position: "fixed",
    zIndex: 5,
    bottom: "1rem",
    right: "1rem",
  },
}));

const Toast = ({ toastlist, position, setList }) => {
  const { classes } = useStyles();

  const deleteToast = useCallback(
    (id) => {
      const toastListItem = toastlist.filter((e) => e.id !== id);
      setList(toastListItem);
    },
    [toastlist, setList]
  );

  useEffect(() => {
    const interval = setInterval(() => {
      if (toastlist.length) {
        deleteToast(toastlist[0].id);
      }
    }, 3000);

    return () => {
      clearInterval(interval);
    };
  }, [toastlist, deleteToast]);

  return (
    <div className={classes.root}>
      {toastlist.map((toast, i) => (
        <Card
          elevation={3}
          className={classes.card}
          style={{ backgroundColor: toast.backgroundColor }}
        >
          <div className={classes.header}>
            <Typography variant="body2">{toast.title}</Typography>
            <IconButton
              size="small"
              onClick={() => deleteToast(toast.id)}
              onTouchStart={() => deleteToast(toast.id)}
            >
              <CloseIcon fontSize="small" className={classes.mediaButton} />
            </IconButton>
          </div>
          <div>
            <p className={classes.description}>{toast.description}</p>
          </div>
        </Card>
      ))}
    </div>
  );
};

export default Toast;

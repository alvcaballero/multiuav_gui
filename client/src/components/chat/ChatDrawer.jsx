import { useState, useEffect } from 'react';
import { Rnd } from 'react-rnd';
import { IconButton } from '@mui/material';
import { makeStyles } from 'tss-react/mui';
import ChevronRightIcon from '@mui/icons-material/ChevronRight';
import useChatLogic from './useChatLogic';
import ChatPanel from './ChatPanel';

const useStyles = makeStyles()(() => ({
  panel: {
    background: '#f0f0f0',
    borderLeft: '2px solid #ccc',
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center',
    flexDirection: 'column',
    boxSizing: 'border-box',
    zIndex: 9,
    cursor: 'default !important',
    '& *': {
      cursor: 'default',
    },
  },
}));

const ChatDrawer = ({ open, onClose }) => {
  const { classes } = useStyles();
  const chatLogic = useChatLogic(open);

  const [panelState, setPanelState] = useState({
    width: 500,
    height: window.innerHeight,
    x: window.innerWidth - 500,
    y: 0,
  });

  useEffect(() => {
    const handleWindowResize = () => {
      setPanelState((prev) => ({ ...prev, x: window.innerWidth - prev.width }));
    };
    window.addEventListener('resize', handleWindowResize);
    return () => window.removeEventListener('resize', handleWindowResize);
  }, []);

  const handleResizeStop = (e, direction, ref, delta, position) => {
    setPanelState({
      width: parseInt(ref.style.width, 10),
      height: '100vh',
      ...position,
    });
  };

  if (!open) return null;

  return (
    <Rnd
      className={classes.panel}
      size={{ width: panelState.width, height: panelState.height }}
      position={{ x: panelState.x, y: panelState.y }}
      minWidth={300}
      maxWidth={window.innerWidth / 2}
      enableResizing={{ left: true, right: false }}
      dragAxis="none"
      onResizeStop={handleResizeStop}
      disableDragging
      style={{ cursor: 'default' }}
    >
      <ChatPanel
        {...chatLogic}
        sx={{ borderRadius: 0 }}
        titleSlot={
          <IconButton size="small" color="inherit" onClick={() => onClose(false)}>
            <ChevronRightIcon fontSize="small" />
          </IconButton>
        }
      />
    </Rnd>
  );
};

export default ChatDrawer;

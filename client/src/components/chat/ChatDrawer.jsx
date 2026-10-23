import { useState, useEffect } from 'react';
import { Rnd } from 'react-rnd';
import { useTheme } from '@mui/material';
import useChatLogic from './useChatLogic';
import ChatPanel from './ChatPanel';

const DEFAULT_WIDTH = 452;
const MIN_WIDTH = 360;

/**
 * The assistant docked to the right edge, as tall as the map: it starts under the navbar and runs
 * to the bottom of the window. Its width is the only thing the operator moves (the left edge).
 */
const ChatDrawer = ({ open, onClose }) => {
  const theme = useTheme();
  const chatLogic = useChatLogic(open);
  const top = parseInt(theme.dimensions.navbarHeight, 10);
  // Drawn around the resizable box, outside the panel's ThemeProvider: the app theme's divider and shadow.
  const panelStyle = {
    zIndex: 9,
    borderLeft: `1px solid ${theme.palette.divider}`,
    boxShadow: theme.shadows[8],
    boxSizing: 'border-box',
  };

  const [width, setWidth] = useState(DEFAULT_WIDTH);
  const [viewport, setViewport] = useState({
    width: window.innerWidth,
    height: window.innerHeight,
  });

  useEffect(() => {
    const handleWindowResize = () =>
      setViewport({ width: window.innerWidth, height: window.innerHeight });
    window.addEventListener('resize', handleWindowResize);
    return () => window.removeEventListener('resize', handleWindowResize);
  }, []);

  if (!open) return null;

  return (
    <Rnd
      style={panelStyle}
      size={{ width, height: viewport.height - top }}
      position={{ x: viewport.width - width, y: top }}
      minWidth={MIN_WIDTH}
      maxWidth={viewport.width / 2}
      enableResizing={{ left: true, right: false }}
      dragAxis="none"
      onResizeStop={(e, direction, ref) => setWidth(parseInt(ref.style.width, 10))}
      disableDragging
    >
      <ChatPanel {...chatLogic} onClose={() => onClose(false)} />
    </Rnd>
  );
};

export default ChatDrawer;

import { useMemo, useState } from 'react';
import { IconButton, Menu, MenuItem, ListItemText, Switch, Tooltip } from '@mui/material';
import LayersIcon from '@mui/icons-material/Layers';
import Scene3DControl from './registry/Scene3DControl';
import { controlSurfaceStyle } from './controlStyles';
import { useScene3DLayerContext } from '../layers/Scene3DLayerProvider';

const Scene3DLayerSwitcher = () => {
  const { layers, hidden, setHidden } = useScene3DLayerContext();
  const [anchorEl, setAnchorEl] = useState(null);
  const titles = useMemo(() => [...new Set(Object.values(layers))], [layers]);

  return (
    <Scene3DControl corner="top-right">
      <Tooltip title="Capas de la escena 3D">
        <IconButton onClick={(e) => setAnchorEl(e.currentTarget)} style={controlSurfaceStyle}>
          <LayersIcon color="primary" />
        </IconButton>
      </Tooltip>
      <Menu anchorEl={anchorEl} open={Boolean(anchorEl)} onClose={() => setAnchorEl(null)}>
        {titles.map((title) => (
          <MenuItem
            key={title}
            onClick={() =>
              setHidden((previous) =>
                previous.includes(title)
                  ? previous.filter((value) => value !== title)
                  : [...previous, title],
              )
            }
          >
            <ListItemText>{title}</ListItemText>
            <Switch edge="end" size="small" checked={!hidden.includes(title)} onChange={() => {}} />
          </MenuItem>
        ))}
      </Menu>
    </Scene3DControl>
  );
};

export default Scene3DLayerSwitcher;

import { useState } from 'react';
import {
  Button,
  Paper,
  TextField,
  ToggleButton,
  ToggleButtonGroup,
  Typography,
} from '@mui/material';
import { makeStyles } from 'tss-react/mui';

import PositionValue from '../../common/components/PositionValue';

const MONO = 'ui-monospace, Menlo, monospace';

const useStyles = makeStyles()((theme) => ({
  root: {
    display: 'flex',
    flexDirection: 'column',
    minHeight: 0,
    height: '100%',
  },
  header: {
    display: 'flex',
    alignItems: 'center',
    gap: theme.spacing(1),
    padding: theme.spacing(1, 1.5),
    borderBottom: `1px solid ${theme.palette.divider}`,
  },
  spacer: {
    flex: 1,
  },
  body: {
    flex: 1,
    minHeight: 0,
    overflow: 'auto',
  },
  table: {
    width: '100%',
    borderCollapse: 'collapse',
    fontFamily: MONO,
    fontSize: 12,
    '& th': {
      position: 'sticky',
      top: 0,
      background: theme.palette.background.paper,
      textAlign: 'left',
      fontFamily: theme.typography.fontFamily,
      fontSize: 10,
      letterSpacing: '0.1em',
      color: theme.palette.text.secondary,
      padding: theme.spacing(0.75, 1.5),
      borderBottom: `1px solid ${theme.palette.divider}`,
    },
    '& td': {
      padding: theme.spacing(0.5, 1.5),
      borderBottom: `1px solid ${theme.palette.divider}`,
      verticalAlign: 'top',
    },
    '& td:first-of-type': { color: theme.palette.text.secondary },
    '& td:last-of-type': { textAlign: 'right', wordBreak: 'break-all' },
  },
  stream: {
    display: 'flex',
    flexDirection: 'column-reverse',
    fontFamily: MONO,
    fontSize: 11,
    padding: theme.spacing(0.75, 1.5),
    background: theme.palette.action.hover,
    minHeight: '100%',
  },
  line: {
    whiteSpace: 'pre-wrap',
    wordBreak: 'break-all',
  },
}));

// Stable color per topic, from a small palette of the theme's functional colors.
const topicColor = (theme, topic) => {
  if (topic === 'command') return theme.palette.text.primary;
  const colors = [
    theme.palette.primary.main,
    theme.palette.success.main,
    theme.palette.warning.main,
    theme.palette.secondary.main,
    theme.palette.info.main,
  ];
  const hash = [...topic].reduce((sum, ch) => sum + ch.charCodeAt(0), 0);
  return colors[hash % colors.length];
};

const RawDataPanel = ({ position, stream }) => {
  const { classes, theme } = useStyles();
  const [tab, setTab] = useState('table');
  const [filter, setFilter] = useState('');

  const query = filter.toLowerCase();
  const properties = position
    ? Object.keys(position).filter(
        (key) => key !== 'attributes' && key.toLowerCase().includes(query),
      )
    : [];
  const attributes = position?.attributes
    ? Object.keys(position.attributes).filter((key) =>
        `attributes.${key}`.toLowerCase().includes(query),
      )
    : [];

  return (
    <Paper variant="outlined" square className={classes.root}>
      <div className={classes.header}>
        <Typography variant="overline" color="primary" sx={{ lineHeight: 1.2 }}>
          Raw data
        </Typography>
        <ToggleButtonGroup
          size="small"
          exclusive
          value={tab}
          onChange={(_, value) => value && setTab(value)}
          sx={{ height: 26 }}
        >
          <ToggleButton value="table">Table</ToggleButton>
          <ToggleButton value="stream">Stream</ToggleButton>
        </ToggleButtonGroup>
        <span className={classes.spacer} />
        {tab === 'table' ? (
          <TextField
            size="small"
            placeholder="Filter"
            value={filter}
            onChange={(e) => setFilter(e.target.value)}
            sx={{ width: 140, '& input': { py: 0.5, fontSize: 12 } }}
          />
        ) : (
          <Button size="small" variant="outlined" onClick={() => stream.setPaused(!stream.paused)}>
            {stream.paused ? 'Resume' : 'Pause'}
          </Button>
        )}
      </div>
      <div className={classes.body}>
        {tab === 'table' ? (
          <table className={classes.table}>
            <thead>
              <tr>
                <th>ATTRIBUTE</th>
                <th style={{ textAlign: 'right' }}>VALUE</th>
              </tr>
            </thead>
            <tbody>
              {properties.map((property) => (
                <tr key={property}>
                  <td>{property}</td>
                  <td>
                    <PositionValue position={position} property={property} />
                  </td>
                </tr>
              ))}
              {attributes.map((attribute) => (
                <tr key={`attributes.${attribute}`}>
                  <td>{`attributes.${attribute}`}</td>
                  <td>
                    <PositionValue position={position} attribute={attribute} />
                  </td>
                </tr>
              ))}
            </tbody>
          </table>
        ) : (
          <div className={classes.stream}>
            {/* column-reverse keeps the view pinned to the newest line */}
            <div>
              {stream.lines.map((line) => (
                <div key={line.key} className={classes.line}>
                  <span style={{ color: theme.palette.text.secondary }}>{line.time}</span>{' '}
                  <span style={{ color: topicColor(theme, line.topic) }}>{line.topic}</span>{' '}
                  {line.payload}
                </div>
              ))}
            </div>
          </div>
        )}
      </div>
    </Paper>
  );
};

export default RawDataPanel;

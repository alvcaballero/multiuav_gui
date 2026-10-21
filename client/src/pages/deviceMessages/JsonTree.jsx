import { useState } from 'react';
import { makeStyles } from 'tss-react/mui';

const MONO = 'ui-monospace, Menlo, monospace';
// Levels open from the start; the rest is one click away.
const OPEN_DEPTH = 3;

const useStyles = makeStyles()((theme) => ({
  root: {
    fontFamily: MONO,
    fontSize: 12.5,
    lineHeight: 1.7,
  },
  children: {
    marginLeft: theme.spacing(2),
    paddingLeft: theme.spacing(1),
    borderLeft: `1px solid ${theme.palette.divider}`,
  },
  toggle: {
    all: 'unset',
    cursor: 'pointer',
    '&:hover': { background: theme.palette.action.hover },
    '&:focus-visible': { outline: `1px solid ${theme.palette.primary.main}` },
  },
  arrow: {
    display: 'inline-block',
    width: '1.2em',
    color: theme.palette.text.secondary,
  },
  key: { color: theme.palette.text.secondary },
  hint: { color: theme.palette.text.disabled, marginLeft: theme.spacing(1) },
  string: { color: theme.palette.success.main, wordBreak: 'break-all' },
  number: { color: theme.palette.info.main },
  boolean: { color: theme.palette.warning.main },
  nil: { color: theme.palette.text.disabled },
}));

const isContainer = (value) => value !== null && typeof value === 'object';

const summary = (value) => {
  if (Array.isArray(value)) return `[${value.length}]`;
  const count = Object.keys(value).length;
  return `{${count} ${count === 1 ? 'key' : 'keys'}}`;
};

const Primitive = ({ value, classes }) => {
  if (value === null) return <span className={classes.nil}>null</span>;
  if (typeof value === 'string')
    return <span className={classes.string}>{JSON.stringify(value)}</span>;
  if (typeof value === 'number') return <span className={classes.number}>{String(value)}</span>;
  if (typeof value === 'boolean') return <span className={classes.boolean}>{String(value)}</span>;
  return <span className={classes.nil}>{String(value)}</span>;
};

// A node keeps its open/closed state while the message updates: React matches it by its key
// (the property name), so a stream of messages with the same shape does not collapse the tree.
const Node = ({ name, value, depth, classes }) => {
  const [open, setOpen] = useState(depth < OPEN_DEPTH);
  const label = name != null && <span className={classes.key}>{name}: </span>;

  if (!isContainer(value)) {
    return (
      <div>
        <span className={classes.arrow} />
        {label}
        <Primitive value={value} classes={classes} />
      </div>
    );
  }

  const entries = Object.entries(value);
  return (
    <div>
      <button
        type="button"
        className={classes.toggle}
        aria-expanded={open}
        onClick={() => setOpen(!open)}
      >
        <span className={classes.arrow}>{open ? '▾' : '▸'}</span>
        {label}
        {!open || entries.length === 0 ? (
          <span className={classes.hint}>{summary(value)}</span>
        ) : null}
      </button>
      {open && (
        <div className={classes.children}>
          {entries.map(([key, child]) => (
            <Node key={key} name={key} value={child} depth={depth + 1} classes={classes} />
          ))}
        </div>
      )}
    </div>
  );
};

/** A JSON value as a collapsible tree. */
const JsonTree = ({ value }) => {
  const { classes } = useStyles();
  // The root has no key to label it, so its entries are the first level instead of a nameless node.
  const entries = isContainer(value) ? Object.entries(value) : [[null, value]];
  return (
    <div className={classes.root}>
      {entries.map(([key, child]) => (
        <Node key={key} name={key} value={child} depth={1} classes={classes} />
      ))}
    </div>
  );
};

export default JsonTree;

import { useState } from 'react';
import { useParams } from 'react-router-dom';
import {
  IconButton,
  Button,
  TextField,
  Accordion,
  AccordionSummary,
  AccordionDetails,
  Typography,
  Table,
  TableHead,
  TableRow,
  TableCell,
  TableBody,
  Box,
} from '@mui/material';
import Tooltip from '@mui/material/Tooltip';
import DeleteIcon from '@mui/icons-material/Delete';
import ExpandMoreIcon from '@mui/icons-material/ExpandMore';
import SelectField from '../common/components/SelectField';
import { useAsyncTask } from '../reactHelper';
import SettingsMenu from './components/SettingsMenu';
import useSettingsStyles from './common/useSettingsStyles';
import EditItemView from './components/EditItemView';

// The devices_msg.yaml blocks a category declares; every block names its type with `messageType`.
// Which keys and types can be picked comes from GET /api/category/messages, derived on the
// server from the decoders and from what categories already declare.
const BLOCKS = [
  { block: 'subscribers', title: 'Telemetry - subscribers' },
  { block: 'services', title: 'Commands - services' },
  { block: 'actions', title: 'Commands - actions' },
  { block: 'publishers', title: 'Commands - publishers' },
];

const EndpointSection = ({ block, title, item, setItem, options }) => {
  const { classes } = useSettingsStyles();
  const [adding, setAdding] = useState(false);
  const [newKey, setNewKey] = useState(null);

  const entries = item[block] ?? {};
  const available = Object.keys(options).filter((key) => !(key in entries));

  const update = (key, field, value) =>
    setItem({ ...item, [block]: { ...entries, [key]: { ...entries[key], [field]: value } } });

  const remove = (key) => {
    const rest = { ...entries };
    delete rest[key];
    setItem({ ...item, [block]: rest });
  };

  const add = () => {
    if (newKey) update(newKey, 'messageType', options[newKey][0]);
    setNewKey(null);
    setAdding(false);
  };

  return (
    <Accordion>
      <AccordionSummary expandIcon={<ExpandMoreIcon />}>
        <Typography variant="subtitle1">{title}</Typography>
      </AccordionSummary>
      <AccordionDetails className={classes.details}>
        <Table className={classes.table}>
          <TableHead>
            <TableRow>
              <TableCell>Key</TableCell>
              <TableCell>Name</TableCell>
              <TableCell>Type</TableCell>
              <TableCell className={classes.columnAction} />
            </TableRow>
          </TableHead>
          <TableBody>
            {Object.entries(entries).map(([key, entry]) => (
              <TableRow key={key}>
                <TableCell>{key}</TableCell>
                <TableCell>
                  <TextField
                    value={entry.name ?? ''}
                    onChange={(e) => update(key, 'name', e.target.value)}
                    placeholder="/topic_or_service"
                  />
                </TableCell>
                <TableCell>
                  <SelectField
                    data={options[key] ?? [entry.messageType]}
                    value={entry.messageType}
                    keyGetter={(e) => e}
                    titleGetter={(e) => e}
                    onChange={(e) => update(key, 'messageType', e.target.value)}
                    emptyValue={null}
                  />
                </TableCell>
                <TableCell className={classes.columnAction} padding="none">
                  <Tooltip title="Remove">
                    <IconButton size="small" onClick={() => remove(key)}>
                      <DeleteIcon fontSize="small" />
                    </IconButton>
                  </Tooltip>
                </TableCell>
              </TableRow>
            ))}
          </TableBody>
        </Table>

        <Box sx={{ textAlign: 'center' }}>
          {adding ? (
            <div>
              <SelectField
                fullWidth
                label="Key"
                data={available}
                emptyValue={null}
                value={newKey}
                onChange={(e) => setNewKey(e.target.value)}
                keyGetter={(e) => e}
                titleGetter={(e) => e}
              />
              <div>
                <Button onClick={() => setAdding(false)}>Cancel</Button>
                <Button onClick={add} disabled={!newKey}>
                  Add
                </Button>
              </div>
            </div>
          ) : (
            <Button
              variant="contained"
              onClick={() => setAdding(true)}
              disabled={!available.length}
            >
              Add
            </Button>
          )}
        </Box>
      </AccordionDetails>
    </Accordion>
  );
};

const SettingsCategoryPageEdit = () => {
  const { id } = useParams();
  const [item, setItem] = useState(null);
  const [catalog, setCatalog] = useState(null);

  useAsyncTask(async () => {
    const response = await fetch('/api/category/messages');
    if (response.ok) {
      setCatalog(await response.json());
    } else {
      throw Error(await response.text());
    }
  }, []);

  return (
    <EditItemView
      endpoint="category"
      item={item}
      setItem={setItem}
      validate={() => Boolean(item)}
      menu={<SettingsMenu />}
      breadcrumbs={['Category', id ?? 'New']}
    >
      {catalog &&
        item &&
        BLOCKS.map((section) => (
          <EndpointSection
            key={section.block}
            {...section}
            item={item}
            setItem={setItem}
            options={catalog[section.block] ?? {}}
          />
        ))}
    </EditItemView>
  );
};

export default SettingsCategoryPageEdit;

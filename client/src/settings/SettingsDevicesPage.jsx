import React, { useState } from 'react';
import { useNavigate } from 'react-router-dom';
import { Table, TableRow, TableCell, TableHead, TableBody, IconButton } from '@mui/material';
import Tooltip from '@mui/material/Tooltip';
import EditIcon from '@mui/icons-material/Edit';
import DeleteIcon from '@mui/icons-material/Delete';
import { useAsyncTask } from '../reactHelper';
import PageLayout from '../shared/components/PageLayout';
import SettingsMenu from './components/SettingsMenu';
import TableShimmer from '../shared/components/TableShimmer';
import SearchHeader, { filterByKeyword } from './components/SearchHeader';
import useSettingsStyles from './common/useSettingsStyles';

const SettingsDevicesPage = () => {
  const { classes } = useSettingsStyles();
  const navigate = useNavigate();

  const [timestamp] = useState(() => Date.now());
  const [items, setItems] = useState([]);
  const [searchKeyword, setSearchKeyword] = useState('');
  const [showAll] = useState(false);
  const [loading, setLoading] = useState(false);

  useAsyncTask(async () => {
    setLoading(true);
    try {
      const response = await fetch(`/api/devices`);
      if (response.ok) {
        setItems(await response.json());
      } else {
        throw Error(await response.text());
      }
    } finally {
      setLoading(false);
    }
  }, [timestamp, showAll]);

  return (
    <PageLayout menu={<SettingsMenu />} breadcrumbs={['settingsTitle', 'deviceTitle']}>
      <SearchHeader keyword={searchKeyword} setKeyword={setSearchKeyword} />
      <Table className={classes.table}>
        <TableHead>
          <TableRow>
            <TableCell>{'deviceId'}</TableCell>
            <TableCell>{'Name'}</TableCell>
            <TableCell>{'Category'}</TableCell>
            <TableCell>{'Protocol'}</TableCell>
            <TableCell>{'camera'}</TableCell>
            <TableCell>{'files'}</TableCell>
            <TableCell className={classes.columnAction} />
          </TableRow>
        </TableHead>
        <TableBody>
          {!loading ? (
            items.filter(filterByKeyword(searchKeyword)).map((item) => (
              <TableRow key={item.id}>
                <TableCell>{item.id}</TableCell>
                <TableCell>{item.name}</TableCell>
                <TableCell>{item.category}</TableCell>
                <TableCell>{item.protocol}</TableCell>
                <TableCell>{item.camera ? item.camera.length : 0}</TableCell>
                <TableCell>{item.files ? item.files.length : 0}</TableCell>
                <TableCell className={classes.columnAction} padding="none">
                  <div className={classes.row}>
                    <Tooltip title={'Edit'}>
                      <IconButton
                        size="small"
                        onClick={() => navigate(`/settings/devices/${item.id}`)}
                      >
                        <EditIcon fontSize="small" />
                      </IconButton>
                    </Tooltip>
                    <Tooltip title={'Remove'}>
                      <IconButton size="small" onClick={() => null}>
                        <DeleteIcon fontSize="small" />
                      </IconButton>
                    </Tooltip>
                  </div>
                </TableCell>
              </TableRow>
            ))
          ) : (
            <TableShimmer columns={7} endAction />
          )}
        </TableBody>
      </Table>
    </PageLayout>
  );
};

export default SettingsDevicesPage;

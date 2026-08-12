import React, { useState, useEffect } from 'react';
import { useNavigate } from 'react-router-dom';
import { Table, TableRow, TableCell, TableHead, TableBody, IconButton, Fab } from '@mui/material';
import Tooltip from '@mui/material/Tooltip';
import EditIcon from '@mui/icons-material/Edit';
import DeleteIcon from '@mui/icons-material/Delete';
import AddIcon from '@mui/icons-material/Add';
import { useCatchCallback } from '../reactHelper';
import PageLayout from '../shared/components/PageLayout';
import SettingsMenu from './components/SettingsMenu';
import TableShimmer from '../shared/components/TableShimmer';
import SearchHeader from './components/SearchHeader';
import { filterByKeyword } from './components/filterByKeyword';
import useSettingsStyles from './common/useSettingsStyles';
import RemoveDialog from '../components/ui/RemoveDialog';
import { invalidateMarkerTypesCache } from '../hooks/useMarkerTypes';
import { describeGeometry } from '../shared/components/GeometryFields';

const SettingsElementTypesPage = () => {
  const { classes } = useSettingsStyles();
  const navigate = useNavigate();

  const [items, setItems] = useState([]);
  const [searchKeyword, setSearchKeyword] = useState('');
  const [loading, setLoading] = useState(false);
  const [removing, setRemoving] = useState(false);
  const [removingId, setRemovingId] = useState(null);

  const loadItems = useCatchCallback(async () => {
    setLoading(true);
    try {
      const response = await fetch('/api/markers/types');
      if (response.ok) {
        setItems(await response.json());
      } else {
        throw Error(await response.text());
      }
    } finally {
      setLoading(false);
    }
  }, []);

  useEffect(() => {
    loadItems();
  }, [loadItems]);

  const handleRemove = (id) => {
    setRemovingId(id);
    setRemoving(true);
  };

  const handleRemoveResult = (removed) => {
    setRemoving(false);
    setRemovingId(null);
    if (removed) {
      invalidateMarkerTypesCache();
      loadItems();
    }
  };

  return (
    <>
      <PageLayout menu={<SettingsMenu />} breadcrumbs={['settingsTitle', 'Element Types']}>
        <SearchHeader keyword={searchKeyword} setKeyword={setSearchKeyword} />
        <Table className={classes.table}>
          <TableHead>
            <TableRow>
              <TableCell>{'Id'}</TableCell>
              <TableCell>{'Name'}</TableCell>
              <TableCell>{'Geometría por defecto'}</TableCell>
              <TableCell className={classes.columnAction} />
            </TableRow>
          </TableHead>
          <TableBody>
            {!loading ? (
              items.flatMap((item) =>
                filterByKeyword(searchKeyword)(item)
                  ? [
                      <TableRow key={item.id}>
                        <TableCell>{item.id}</TableCell>
                        <TableCell>{item.name}</TableCell>
                        <TableCell>{describeGeometry(item.attributes?.geometry) ?? '—'}</TableCell>
                        <TableCell className={classes.columnAction} padding="none">
                          <div className={classes.row}>
                            <Tooltip title={'Edit'}>
                              <IconButton
                                size="small"
                                onClick={() => navigate(`/settings/elementTypes/${item.id}`)}
                              >
                                <EditIcon fontSize="small" />
                              </IconButton>
                            </Tooltip>
                            <Tooltip title={'Remove'}>
                              <IconButton size="small" onClick={() => handleRemove(item.id)}>
                                <DeleteIcon fontSize="small" />
                              </IconButton>
                            </Tooltip>
                          </div>
                        </TableCell>
                      </TableRow>,
                    ]
                  : [],
              )
            ) : (
              <TableShimmer columns={4} endAction />
            )}
          </TableBody>
        </Table>
        <Tooltip title={'Add element type'}>
          <Fab
            color="primary"
            size="medium"
            onClick={() => navigate('/settings/elementTypes/new')}
            sx={{ position: 'fixed', bottom: 24, right: 24 }}
          >
            <AddIcon />
          </Fab>
        </Tooltip>
      </PageLayout>
      {removingId && (
        <RemoveDialog
          open={removing}
          endpoint="markers/types"
          itemId={removingId}
          onResult={handleRemoveResult}
        />
      )}
    </>
  );
};

export default SettingsElementTypesPage;

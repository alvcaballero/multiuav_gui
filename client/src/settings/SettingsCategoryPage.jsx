import React, { useState } from 'react';
import { useNavigate } from 'react-router-dom';
import { Table, TableRow, TableCell, TableHead, TableBody, IconButton } from '@mui/material';
import Tooltip from '@mui/material/Tooltip';
import EditIcon from '@mui/icons-material/Edit';
import DeleteIcon from '@mui/icons-material/Delete';
import { useEffectAsync } from '../reactHelper';
import PageLayout from '../shared/components/PageLayout';
import SettingsMenu from './components/SettingsMenu';
import TableShimmer from '../shared/components/TableShimmer';
import SearchHeader, { filterByKeyword } from './components/SearchHeader';
import useSettingsStyles from './common/useSettingsStyles';
import RemoveDialog from '../components/ui/RemoveDialog';

const SettingsCategoryPage = () => {
  const { classes } = useSettingsStyles();
  const navigate = useNavigate();

  const [timestamp] = useState(() => Date.now());
  const [items, setItems] = useState([]);
  const [searchKeyword, setSearchKeyword] = useState('');
  const [showAll] = useState(false);
  const [loading, setLoading] = useState(false);
  const [removing, setRemoving] = useState(false);
  const [myCategory, setMyCategory] = useState(null);

  useEffectAsync(async () => {
    setLoading(true);
    try {
      const response = await fetch(`/api/category`);
      if (response.ok) {
        setItems(await response.json());
      } else {
        throw Error(await response.text());
      }
    } finally {
      setLoading(false);
    }
  }, [timestamp, showAll]);

  const handleEdit = (item) => {
    navigate(`/settings/category/${item}`);
  };

  const handleRemove = (item) => {
    setMyCategory(item);
    setRemoving(true);
  };

  const hamdleRemoveResult = () => {
    setMyCategory(null);
    setRemoving(false);
  };

  return (
    <>
      <PageLayout menu={<SettingsMenu />} breadcrumbs={['settingsTitle', 'deviceTitle']}>
        <SearchHeader keyword={searchKeyword} setKeyword={setSearchKeyword} />
        <Table className={classes.table}>
          <TableHead>
            <TableRow>
              <TableCell>{'Id'}</TableCell>
              <TableCell>{'Name'}</TableCell>
              <TableCell className={classes.columnAction} />
            </TableRow>
          </TableHead>
          <TableBody>
            {!loading ? (
              items.filter(filterByKeyword(searchKeyword)).map((item, indexItem) => (
                <TableRow key={indexItem}>
                  <TableCell>{indexItem}</TableCell>
                  <TableCell>{item}</TableCell>
                  <TableCell className={classes.columnAction} padding="none">
                    <div className={classes.row}>
                      <Tooltip title={'Edit'}>
                        <IconButton size="small" onClick={() => handleEdit(item)}>
                          <EditIcon fontSize="small" />
                        </IconButton>
                      </Tooltip>
                      <Tooltip title={'Remove'}>
                        <IconButton size="small" onClick={() => handleRemove(item)}>
                          <DeleteIcon fontSize="small" />
                        </IconButton>
                      </Tooltip>
                    </div>
                  </TableCell>
                </TableRow>
              ))
            ) : (
              <TableShimmer columns={3} endAction />
            )}
          </TableBody>
        </Table>
      </PageLayout>
      {myCategory && (
        <RemoveDialog
          open={removing}
          endpoint="category"
          ItemId={myCategory}
          onResult={hamdleRemoveResult}
        />
      )}
    </>
  );
};

export default SettingsCategoryPage;

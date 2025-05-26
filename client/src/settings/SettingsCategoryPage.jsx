import React, { useState } from 'react';
import { useSelector } from 'react-redux';
import { useNavigate } from 'react-router-dom';
import {
  Table,
  TableRow,
  TableCell,
  TableHead,
  TableBody,
  Button,
  TableFooter,
  FormControlLabel,
  Switch,
} from '@mui/material';
import { IconButton, Menu, MenuItem, useMediaQuery, useTheme } from '@mui/material';
import Tooltip from '@mui/material/Tooltip';
import MoreVertIcon from '@mui/icons-material/MoreVert';
import EditIcon from '@mui/icons-material/Edit';
import DeleteIcon from '@mui/icons-material/Delete';
import LinkIcon from '@mui/icons-material/Link';
import { useEffectAsync } from '../reactHelper';
import PageLayout from '../common/components/PageLayout';
import SettingsMenu from './components/SettingsMenu';
import TableShimmer from '../common/components/TableShimmer';
import SearchHeader, { filterByKeyword } from './components/SearchHeader';
import { formatTime } from '../common/formatter';
import useSettingsStyles from './common/useSettingsStyles';
import RemoveDialog from '../components/RemoveDialog';

const SettingsCategoryPage = () => {

  const { classes } = useSettingsStyles();
  const navigate = useNavigate();

  const [timestamp, setTimestamp] = useState(Date.now());
  const [items, setItems] = useState([]);
  const [searchKeyword, setSearchKeyword] = useState('');
  const [showAll, setShowAll] = useState(false);
  const [loading, setLoading] = useState(false);
  const [removing, setRemoving] = useState(false);
  const [myCategory, setMyCategory] = useState(null);

  useEffectAsync(async () => {
    setLoading(true);
    try {
      const query = new URLSearchParams({ all: showAll });
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
  }

  const handleRemove = (item) => {
    setMyCategory(item);
    setRemoving(true);
  }

  const hamdleRemoveResult = (result) => {
    setMyCategory(null);
    setRemoving(false);
  }



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
      {myCategory && <RemoveDialog open={removing} endpoint="category" ItemId={myCategory} onResult={hamdleRemoveResult} />}
    </>
  );
};

export default SettingsCategoryPage;

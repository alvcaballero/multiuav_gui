import { useState } from 'react';
import {
    IconButton, Menu, MenuItem, useMediaQuery, useTheme,
} from '@mui/material';
import Tooltip from '@mui/material/Tooltip';
import MoreVertIcon from '@mui/icons-material/MoreVert';
import EditIcon from '@mui/icons-material/Edit';
import DeleteIcon from '@mui/icons-material/Delete';
import { useNavigate } from 'react-router-dom';
import { makeStyles } from 'tss-react/mui';
import RemoveDialog from '../../common/components/RemoveDialog';

const useStyles = makeStyles()(() => ({
    row: {
        display: 'flex',
    },
}));

const CollectionActions = ({
    itemId, editPath, endpoint, setTimestamp, customActions, readonly,
}) => {
    const theme = useTheme();
    const { classes } = useStyles();
    const navigate = useNavigate();

    const phone = useMediaQuery(theme.breakpoints.down('sm'));

    const [menuAnchorEl, setMenuAnchorEl] = useState(null);
    const [removing, setRemoving] = useState(false);

    const handleEdit = () => {
        navigate(`${editPath}/${itemId}`);
        setMenuAnchorEl(null);
    };

    const handleRemove = () => {
        setRemoving(true);
        setMenuAnchorEl(null);
    };

    const handleCustom = (action) => {
        action.handler(itemId);
        setMenuAnchorEl(null);
    };

    const handleRemoveResult = (removed) => {
        setRemoving(false);
        if (removed) {
            setTimestamp(Date.now());
        }
    };

    return (
        <>
            {phone ? (
                <>
                    <IconButton size="small" onClick={(event) => setMenuAnchorEl(event.currentTarget)}>
                        <MoreVertIcon fontSize="small" />
                    </IconButton>
                    <Menu open={!!menuAnchorEl} anchorEl={menuAnchorEl} onClose={() => setMenuAnchorEl(null)}>
                        {customActions && customActions.map((action) => (
                            <MenuItem onClick={() => handleCustom(action)} key={action.key}>{action.title}</MenuItem>
                        ))}
                        {!readonly && (
                            <>
                                {editPath && <MenuItem onClick={handleEdit}>{'sharedEdit'}</MenuItem>}
                                <MenuItem onClick={handleRemove}>{'sharedRemove'}</MenuItem>
                            </>
                        )}
                    </Menu>
                </>
            ) : (
                <div className={classes.row}>
                    {customActions && customActions.map((action) => (
                        <Tooltip title={action.title} key={action.key}>
                            <IconButton size="small" onClick={() => handleCustom(action)}>
                                {action.icon}
                            </IconButton>
                        </Tooltip>
                    ))}
                    {!readonly && (
                        <>
                            {editPath && (
                                <Tooltip title={'sharedEdit'}>
                                    <IconButton size="small" onClick={handleEdit}>
                                        <EditIcon fontSize="small" />
                                    </IconButton>
                                </Tooltip>
                            )}
                            <Tooltip title={'sharedRemove'}>
                                <IconButton size="small" onClick={handleRemove}>
                                    <DeleteIcon fontSize="small" />
                                </IconButton>
                            </Tooltip>
                        </>
                    )}
                </div>
            )}
            <RemoveDialog style={{ transform: 'none' }} open={removing} endpoint={endpoint} itemId={itemId} onResult={handleRemoveResult} />
        </>
    );
};

export default CollectionActions;

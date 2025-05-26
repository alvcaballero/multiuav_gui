import Button from '@mui/material/Button';
import { Snackbar } from '@mui/material';
import { makeStyles } from 'tss-react/mui';
import { useCatch } from '../../reactHelper';
import { snackBarDurationLongMs } from '../util/duration';

const useStyles = makeStyles()((theme) => ({
    root: {
        [theme.breakpoints.down('md')]: {
            bottom: `calc(${theme.dimensions.bottomBarHeight}px + ${theme.spacing(1)})`,
        },
    },
    button: {
        height: 'auto',
        marginTop: 0,
        marginBottom: 0,
    },
}));

const RemoveDialog = ({
    open, endpoint, itemId, onResult,
}) => {
    const { classes } = useStyles();

    const handleRemove = useCatch(async () => {
        const response = await fetch(`/api/${endpoint}/${itemId}`, { method: 'DELETE' });
        if (response.ok) {
            onResult(true);
        } else {
            throw Error(await response.text());
        }
    });

    return (
        <Snackbar
            className={classes.root}
            open={open}
            autoHideDuration={snackBarDurationLongMs}
            onClose={() => onResult(false)}
            message={'sharedRemoveConfirm'}
            action={(
                <Button size="small" className={classes.button} color="error" onClick={handleRemove}>
                    {'sharedRemove'}
                </Button>
            )}
        />
    );
};

export default RemoveDialog;

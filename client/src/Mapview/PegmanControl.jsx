import React, { useCallback, useState } from 'react';
import { IconButton } from '@mui/material';
import StreetviewIcon from '@mui/icons-material/Streetview';
import { useDispatch } from 'react-redux';
import { useNavigate } from 'react-router-dom';
import { map } from './MapView'; // Access the global map instance
import { sessionActions } from '../store';

const PegmanControl = () => {
    const dispatch = useDispatch();
    const navigate = useNavigate();
    const [isDragon, setDragging] = useState(false);

    const handleDragStart = useCallback((e) => {
        setDragging(true);
        e.dataTransfer.effectAllowed = 'copyMove';
        // Create a ghost image if needed, or rely on default browser drag image
        // const img = new Image();
        // img.src = 'path/to/pegman.png';
        // e.dataTransfer.setDragImage(img, 10, 10);
    }, []);

    const handleDragEnd = useCallback((e) => {
        setDragging(false);
        // Get drop coordinates relative to the viewport
        const { clientX, clientY } = e;

        // Calculate the position relative to the map container
        // We assume the map covers the entire viewport or we can get the element
        const mapContainer = map.getContainer();
        const rect = mapContainer.getBoundingClientRect();

        const x = clientX - rect.left;
        const y = clientY - rect.top;

        // Convert screen coordinates to Lat/Lng
        try {
            const lngLat = map.unproject([x, y]);

            // Update 3D scene origin
            dispatch(sessionActions.updateScene3dOrigin({
                lat: lngLat.lat,
                lng: lngLat.lng,
                alt: 100 // Default altitude
            }));

            // Navigate to 3D view
            navigate('/3Dview');

        } catch (error) {
            console.error("Error projecting coordinates:", error);
        }
    }, [dispatch, navigate]);

    return (
        <div
            draggable
            onDragStart={handleDragStart}
            onDragEnd={handleDragEnd}
            style={{
                position: 'absolute',
                bottom: '120px', // Adjust as needed to not overlap with other controls
                right: '10px',
                zIndex: 10,
                backgroundColor: 'white',
                borderRadius: '50%',
                boxShadow: '0 0 0 2px rgba(0,0,0,0.1)',
                cursor: 'grab',
                width: 40,
                height: 40,
                display: 'flex',
                alignItems: 'center',
                justifyContent: 'center'
            }}
        >
            <StreetviewIcon color="primary" />
        </div>
    );
};

export default PegmanControl;

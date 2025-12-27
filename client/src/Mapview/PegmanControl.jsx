// based on https://github.com/rezw4n/maplibre-google-streetview/tree/master
import React, { useCallback, useState, useEffect, useMemo } from 'react';
import { createPortal } from 'react-dom';
import StreetviewIcon from '@mui/icons-material/Streetview';
import { useDispatch } from 'react-redux';
import { useNavigate } from 'react-router-dom';
import { map } from './MapView'; // Access the global map instance
import { sessionActions } from '../store';

// Helper class for MapLibre control
class PegmanControlImpl {
    constructor() {
        this._container = document.createElement('div');
        this._container.className = 'maplibregl-ctrl'; // Base class for controls
        // We don't use 'maplibregl-ctrl-group' to avoid default styling that might conflict with the round button
        // But we can add it if we want the standard look. Given the custom style, we keep it minimal.
    }

    onAdd(map) {
        this._map = map;
        return this._container;
    }

    onRemove() {
        if (this._container.parentNode) {
            this._container.parentNode.removeChild(this._container);
        }
        this._map = undefined;
    }

    getContainer() {
        return this._container;
    }
}

const PegmanButton = () => {
    const dispatch = useDispatch();
    const navigate = useNavigate();
    const [isDragon, setDragging] = useState(false); // Preserving variable name 'isDragon' from original even if typo for 'isDragging'

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
                // Removed absolute positioning as the map control handles placement
                backgroundColor: 'white',
                borderRadius: '50%',
                boxShadow: '0 0 0 2px rgba(0,0,0,0.1)',
                cursor: 'grab',
                width: 40,
                height: 40,
                display: 'flex',
                alignItems: 'center',
                justifyContent: 'center',
                pointerEvents: 'auto', // Ensure interaction
            }}
        >
            <StreetviewIcon color="primary" />
        </div>
    );
};

const PegmanControl = () => {
    // Create the control instance once
    const control = useMemo(() => new PegmanControlImpl(), []);

    useEffect(() => {
        map.addControl(control, 'bottom-right');
        return () => { map.removeControl(control); };
    }, [control]);

    // Render the button into the control's container using a Portal
    return createPortal(
        <PegmanButton />,
        control.getContainer()
    );
};

export default PegmanControl;

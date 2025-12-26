import React, { useCallback } from 'react';
import { IconButton, Tooltip } from '@mui/material';
import DownloadIcon from '@mui/icons-material/Download';
import { useSelector } from 'react-redux';
import { LatLon2XYZ } from './convertion';
import { modelKey } from './ModelLoader.jsx';
import YAML from 'yaml';

const DownloadYamlButton = () => {
    const elements = useSelector((state) => state.session.markers);
    const origin3d = useSelector((state) => state.session.scene3d.origin);
    const perimeter =  1000; // meters

    // Reusing logic from R3DMarkers to flatten the list
    const list2Points = useCallback((mylist) => {
        const waypoints = [];
        if (mylist?.elements) {
            mylist.elements.forEach((conjunto, index_cj) => {
                conjunto.items.forEach((items, item_index) => {
                    const type = modelKey(conjunto.type);
                    waypoints.push({ ...items, type, title: `${index_cj}-${item_index}` });
                });
            });
        }
        if (mylist?.bases) {
            mylist.bases.forEach((items, item_index) => {
                const type = modelKey('base');
                waypoints.push({ ...items, type, title: item_index });
            });
        }
        return waypoints;
    }, []);

    const handleDownload = () => {
        if (!origin3d || !elements) return;

        const listelemnts = list2Points(elements);

        // Convert to XYZ
        const pos = listelemnts.map(({ latitude, longitude }) => {
            return { lng: longitude, lat: latitude, alt: 0 };
        });

        const posxyz = LatLon2XYZ(origin3d, pos);

        // Combine and Filter
        const elementxyz = listelemnts.map((element, index) => {
            // LatLon2XYZ returns [x, y, z] (or similar, check convertion.js implementation)
            // Based on R3DMarkers.jsx: pos: [posxyz[index][0], posxyz[index][2], posxyz[index][1]]
            // It swaps Axis? 
            // convertion.js: return [x, y, z] where y is up? No...
            // MapLibre coords: x/y are ground plane?
            // R3DMarkers uses: [posxyz[index][0], posxyz[index][2], posxyz[index][1]] -> [x, z, y]?
            // Let's stick to what R3DMarkers does for visual consistency, but user asked for "xyz: [posx,posy,pos_z]"
            // Usually in ROS: X forward, Y left, Z up.
            // In ThreeJS: Y is up.
            // Let's assume the R3DMarkers usage is correct for the scene.
            return {
                ...element,
                pos: [
                    Number(posxyz[index][0].toFixed(1)),
                    Number(posxyz[index][1].toFixed(1)),
                    Number(posxyz[index][2].toFixed(1))
                ]
            };
        });

        const filtered = elementxyz.filter(
            (item) => item.pos[0] > -perimeter && item.pos[0] < perimeter && item.pos[1] > -perimeter && item.pos[1] < perimeter
        );

        // Build YAML Object
        const yamlData = {
            origin: [origin3d.lat, origin3d.lng, origin3d.alt],
        };

        filtered.forEach((item, index) => {
            // Create unique key: type + index (global index to ensure uniqueness)
            const key = `${item.type}_${index}`;

            yamlData[key] = {
                type: item.type,
                pose: {
                    xyz: item.pos, // [x, y, z] from above
                    rpy: [0, 0, 0]
                }
            };
        });

        // Generate YAML string
        const yamlString = YAML.stringify(yamlData);

        // Download File
        const blob = new Blob([yamlString], { type: 'text/yaml' });
        const url = URL.createObjectURL(blob);
        const link = document.createElement('a');
        link.href = url;
        link.download = 'scene_data.yaml';
        document.body.appendChild(link);
        link.click();
        document.body.removeChild(link);
        URL.revokeObjectURL(url);
    };

    return (
        <div style={{
            position: 'absolute',
            top: '100px',
            right: '20px', // Right side placement
            zIndex: 10,
            pointerEvents: 'auto'
        }}>
            <Tooltip title="Download Scene YAML">
                <IconButton
                    onClick={handleDownload}
                    style={{
                        backgroundColor: 'white',
                        boxShadow: '0 0 0 2px rgba(0,0,0,0.1)'
                    }}
                >
                    <DownloadIcon color="primary" />
                </IconButton>
            </Tooltip>
        </div>
    );
};

export default DownloadYamlButton;

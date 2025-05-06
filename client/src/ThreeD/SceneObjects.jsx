import React from 'react';
import { useModelLoader } from './ModelLoader';

const SceneObjects = ({ elements }) => {
    return (
        <>
            {elements.map((element, index) => {
                const { model, error } = useModelLoader(element.type);

                if (error) {
                    console.error(error);
                    return null;
                }

                if (!model) {
                    return null;
                }

                return (
                    <primitive
                        key={index}
                        object={model.scene.clone()}
                        position={element.pos}
                        scale={[1, 1, 1]} // Ajusta la escala según necesites
                    />
                );
            })}
        </>
    );
};

export default SceneObjects; 
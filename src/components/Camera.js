import React from 'react'
import novideo from '../assets/img/placeholder.jpg';

const cameraStyle={
  height: '300px',
  width: '300px',
  objectFit: 'contain'
};

export const Camera = () => {
  return (
    <div className="camera_container">
    <img id="my_image" style={cameraStyle} src={novideo}/>
  </div>
  )
}

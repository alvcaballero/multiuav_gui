import React , {useState }from 'react'

import { use } from 'builder-util';



export const Statuswindow = () => {

const [open,SetOpen] = useState(false);

const handleClick = () =>{
  SetOpen(true);
};

const handleClose = (event, reason) => {
  if (reason === 'clickaway'){
    return null;
  }
  SetOpen(false);
};


  return (
<div>
        <div id="statusWindow">
          Status Window.
        </div>


        </div>

  )
}

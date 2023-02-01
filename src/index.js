'use strict'

import { app, BrowserWindow, Tray,dialog, ipcMain } from 'electron'
// import rosnodejs, {nh} from 'rosnodejs'
import devtools from './devtools'
//import exporkillallbag from './renderer/frontend'

//var tools = require('./renderer/frontend');


if (process.env.NODE_ENV === 'development') {
  devtools()
}


app.on('before-quit', () => {
//  exporkillallbag()
  console.log('Closing app...')
})

app.on('ready', () => {
  let win = new BrowserWindow({
    width: 800,
    height: 600,
    title: 'Aerial-Core GUI',
    icon: `${__dirname}/crate.png`,
    center: true,
    show: false,
    webPreferences: {
        nodeIntegration: true //Delete to avoid security issues
    },
    autoHideMenuBar: true
  })
  
  win.once('ready-to-show', () => {
    win.show()
  })

  win.on('closed', () => {
    win = null
    app.quit()
  })

  win.loadURL(`file://${__dirname}/renderer/index.html`)
  

})




  
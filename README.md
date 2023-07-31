


# GCS for multiple UAV
Interface for control and monitoring of multiple UAV in mision


you can use 

```
tmuxinator start -p muav-gui.yml
```
for create a new  tmux file.
```
EDITOR=nano tmuxinator edit my_project
```
this work is divide in two parts, multiauv_gui and mauv-lc.


solo esta disponible para navegadores Web basados en Chromiun

you have to install  dependencies 
for run  the project:
```
nvm use 18 
cd server
npm install
```


for run the project  you can access to http://localhost:4000/
```
nvm use 18 // this is onli for change the version of npm, if you hav installed npm 18 dont need this line
cd server
npm run server
```

for development of MUAV-GUI  can access to http://localhost:3000/
```
nvm use 18
npm install 
npm run start
```


# multiuav_gui
GRVC repository including the Graphical User Interface developed for the operation of a team of heterogeneous UAVs.


# Descripcions

By install the GUI its necesary  install node 10 you can use nvm [like this tutorial for multiple](https://www.baeldung.com/linux/multiple-node-js-versions)

and after to clone  npm install to download all dependencies of the proyect.
```
        npm install 

```
# Setting up the project

1. [Dependencias e instalacion de GCS](guides/HOWRUN.md)
1. [PX4 autopilot](guides/Autopilot-Px4.md)
1. [Actividades](guides/Activities.md)
1. [gcs guide](guides/gcs_guide.md)
1. [dji guide](guides/gcs_guide.md)


# How run the GCS

Para ejecutar la GSC es nenecesario tener ros bridge corriendo y ejercutar el archivo npm.

## Run the rosbridge
```
    $ roslaunch rosbridge_server rosbridge_websocket.launch

```
## Run the Aerial-Core GUI
```
    $ npm run dev
```


# Install GSC

First, install nodejs (v10) and npm (v6):

```
$ cd ~
$ curl -sL https://deb.nodesource.com/setup_10.x -o nodesource_setup.sh
$ sudo bash nodesource_setup.sh
$ sudo apt install nodejs
```

Then, install the rosbridge-server package
```
$ sudo apt x ros-melodic-rosbridge-server
```

Finally, install dependencies with npm:
```
$ roscd aerialcore_gcs/aerialcore_gui
$ npm i
```
# References
* (inspector software_UAV)[https://github.com/AlejandroCastillejo/inspector_software_uav]  en este proyecto se optiene archivos de la camara mediante ftp y se almacenas en la mochila del dron para luego ser enviados a la gcs  mediante sshpass  usando  y scp, esto se ejeucta directamente en el drone

* (autonomus landing)[https://github.com/MikeS96/autonomous_landing_uav/blob/master/Usage.md]

* (altitude sensor)[https://github.com/AlejandroCastillejo/sf11_altitude_sensor]

*(inspector software_UAV)[https://github.com/AlejandroCastillejo/inspector_software_uav] 
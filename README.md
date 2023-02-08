# multiuav_gui
GRVC repository including the Graphical User Interface developed for the operation of a team of heterogeneous UAVs.


# GCS for multiple UAV
Interface for control and monitoring of multiple UAV in mision



# Descripcions




# Setting up the project

1. [Dependencias e instalacion de GCS](guides/HOWRUN.md)
1. [PX4 autopilot](guides/Autopilot-Px4.md)
1. [Actividades](guides/Activities.md)
1. [gcs guide](guides/gcs_guide.md)
1. [dji guide](guides/gcs_guide.md)


# How run the GCS

Para ejecutar la GSC es nenecesario tener ros bridge corriendo y ejercutar el archivo npm.

## Run the rosbridge

    $ roslaunch rosbridge_server rosbridge_websocket.launch


## Run the Aerial-Core GUI

    $ npm run dev



# Install GSC

First, install nodejs (v10) and npm (v6):

```
$ cd ~
$ curl -sL https://deb.nodesource.com/setup_10.x -o nodesource_setup.sh
$ sudo bash nodesource_setup.sh
$ sudo apt install nodejs
```

Then, install the rosbridge-server package:
```
$ sudo apt x ros-melodic-rosbridge-server
```

Finally, install dependencies with npm:
```
$ roscd aerialcore_gcs/aerialcore_gui
$ npm i
```

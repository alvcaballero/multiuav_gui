# Aerial-Core GUI

## Install dependencies

First, install nodejs (v10) and npm (v6):

ahora estoy trabajando con la verison de nodejs V14
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

## Run the rosbridge
```
$ roslaunch rosbridge_server rosbridge_websocket.launch
```

## Run the Aerial-Core GUI

```
$ npm run dev
```

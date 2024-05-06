#!/bin/bash

docker start elegant_bhabha


docker run --rm -i --network=host -v ~/work/px4/multiuav_gui/server/config/mediamtx.yml:/mediamtx.yml bluenviron/mediamtx


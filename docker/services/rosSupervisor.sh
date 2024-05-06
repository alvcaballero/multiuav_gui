#!/bin/bash

echo " init service for px4_noetic"


myflatvar=0
while  [ $myflatvar -lt 2 ];
do
    myvar=$(docker ps --filter "name=px4_noetic" --filter "status=running" )
    echo -e  $myvar
    if [[ "$myvar" == *'px4_noetic'* ]]; then
        echo "already running"
        ((myflatvar++))
    fi
    sleep 2
done
sleep 10


var=0

while ! ping -c 1 -n -w 1 10.42.0.2 &> /dev/null
do
    echo " ping failt waiting for exec ping ."
	sleep 10
    var=$((var+1))
done

sleep 2

if [ $var -ne 0 ]; then
    echo "var is different from zero,init docker again..."
    docker stop px4_noetic
    source ~/.nvm/nvm.sh && nvm use 21 && pm2 restart roscore
    source ~/.nvm/nvm.sh && nvm use 21 && pm2 restart rosbridge
fi


if [ $var -eq 0 ]; then
    echo " find the ip address in first ping"
fi



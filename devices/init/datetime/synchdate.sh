#!/bin/bash

printf "waiting for GCS ...\n"
while ! ping -c 1 -n -w 1 10.42.0.2 &> /dev/null
do
	sleep 2
done

var=0

while  [ "$var" -eq 0 ];do
    sleep 2
    myvar=$(curl -s http://10.42.0.2:4000/api/utils/datetime)
    echo -e  $myvar
        if [[ "$myvar" == *'datetime'* ]]; then
        echo "It's there."
        var=1
    fi
done
property_value=$(echo "$myvar" | grep -o '"datetime": *"[^"]*"' | awk -F'"' '{print $4}')
echo -e  "$property_value"
date --set "$property_value"

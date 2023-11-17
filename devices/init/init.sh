#!/bin/bash
printf "waiting for internet ..."
/home/ubuntu/init/datetime/synchdate.sh &
#openvpn --config /home/ubuntu/init/vpn/RESISTO-IRECVPN.ovpn  --auth-user-pass /home/ubuntu/init/vpn/login.text&

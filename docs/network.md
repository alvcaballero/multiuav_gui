use netplan

https://linuxconfig.org/how-to-configure-static-ip-address-on-ubuntu-18-04-bionic-beaver-linux

sudo apt install netplan.io

networkctl # for show if is configure red

ip a # ver la configuracion

sudo systemctl status systemd-networkd
sudo systemctl status netplan-wpa-wlan0.service

nmcli device wifi list # list wifi ssid avaliable

sudo wpa_cli -i wlan0 list_networks # list networks
sudo wpa_cli -i wlan0 select_network network_id

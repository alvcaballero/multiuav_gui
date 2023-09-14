Confugurar el wifi ingresar usuario y contraseña

    nmtui

ver prioridad de internet

    ip r

si primero esta ethernet borrarlo

    sudo ip r delete default via 10.42.0.1 dev eth0 proto static

d
cambiar la fecha y la hora para que pueda hacer el update

    sudo date --set "2023-06-23 15:03"

vpn

sudo openvpn --config ~/vpn/RESISTO-IRECVPN.ovpn --auth-user-pass ~/vpn/login.text

https://elblogdelazaro.org/posts/2022-12-05-inicio-automatico-del-cliente-openvpn-en-systemd/

esta pagina solo le falta que hay que cambiar el archivo .ovpn to .conf

problema no esta iniciando bien el servicion de open vpn ya que apace la interfaz de red pero no da ping, al ejecutar el comando si funciono y luego se reinicio el servicio y tambien funcionaba.nota por alguna razon toca de Reinciar el computador para que en el sistema se cargen los datos de host.

docker
$ sudo apt-get update
$ sudo apt-get install ca-certificates curl gnupg

https://developer.dji.com/doc/cloud-api-tutorial/en/quick-start/environment-prepare-list.html

Configuracion de netplan DNS en caso de que no coga internet
https://www.ochobitshacenunbyte.com/2021/04/26/netplan-configurar-la-red-en-ubuntu-20-04/

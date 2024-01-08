En simulación DJI -hexarotor, ¿que es alias M210-sim-on?

¿Para que sirve multimaster requirements?

ssh ubuntu@10.42.0.42 para conectarse remotamente al ordenador a bordo (que es la raspberry, y essta es su IP)

Diego arregló la configuración de la Ubiquiti de tierra subiéndole un archivo de configuración que tenía él
hay que mirar si los puertos de expansion están habilitados desde el mando (viene en el manual, se supone que esto solo hay que hacerlo la primera vez)
el USB mode switch debe estar hacia la derecha, de este modo se puede conectar con dispositivos OSDK
Al final era que el switch estaba reventado y no hacia bien contacto hacia la derecha, hay que asegurarse de esto porque una vez que ya se encajó bien ya detectó el ordenador a bordo (la raspberry, OSDK device externo. DJI tiene una pagina de como conectar con dispositivos externos)

comando para mostrar dispositivos: ls/dev/tty tab tab

multimaster:

comprobar que direcciones ip y hostname (se puede comrpobar el hostname de cada maquina escribiendo hostname en el terminal) estan bien escritos en /etc/hosts, sudo nano /etc/hosts (ctrl + x para salir, aceptar, intro)

en el .bashrc también hay que tocar parámetros (mirar wiki de multirotor, ahi viene ROS IP... y esas cosas). Cuando se haya cambiado hay que acordarse de hacer source .bashrc. ESto solo hay que hacerlo una vez.

sudo sh -c "echo 1 >/proc/sys/net/ipv4/ip_forward"
sudo sh -c "echo 0 >/proc/sys/net/ipv4/icmp_echo_ignore_broadcasts" en cada maquina con cada reboot de las maquinas

estaban puertos deshabilitados! sudo ufw allow 11311, sudo ufw allow 11611, esto solo hace falta hacerlo una vez y nos notificara si la orden ya estaba añadida

Para comprobar se pueden ir mirando los nodos que hay activos con rosnode list. Cuando lacnemos el multimaster.launch en la raspi y el connect_uas.launch en el portatil, podremos lanzar por ejemplo roslaunch aerialcore_onboard_dji atlas. launch (que no tiene nada que ver ya con el multimaster) y ver como se han añadido estos nodos haciendo rosnode list otra vez. desde el terminal de la otra maquina (este protatil) si hacemos rosnode list tambien se verian.

Para ver los canales de comunicacion que se han abierto, rostopic echo /master_discovery/linkstat

conecta raspi a router por la entrada fisica 1, poner el movil para dar datos y configurar el nombre y la pswd de la zona wifi a lo que pone en la pegatina de la raspi.

Despues sacar otro cable ethernet desde la salida fisica 2 del router hacia el puerto LAN del POE (el enchufe que va con la Ubiquiti

El cable ethernet qye se saca desde la salida fisica 3 del router es hacia el puerto ethernet del portatil.

Para conectarse directamente del POE al portatil via ethernet: hay que configurar la red como IPv4 (direccion 10.42.0.2, mascara de red 255.255.255.0, puerta de enlace 10.42.0.1) e IPv6 en automatico.

La ip al parecer hay que configurarla en dinámica, (¿esto no puede chocar con cuando pusimos la ip a 10.42.0.2?)

Acceder al router para poner una ip visible en el rango del resto de elementos del sistema (10.42...).DIego accede a la config del router desde http://192.168.31.1/ en el navegador, y para acceder hay que estar conectado por wifi al router (giorgio, grvc1234)

config router: direcciopn IP LAN a 10.42.0.1, la misma que la puerta de enlace puesta en el portatil, despues cambiar la direccion de inicio a 10.42.0.2, y finalizar IP la deja a 254 y el tiempo de concesion a 720.La direccion del router es 10.42.0.1, y le hemos dicho al router que empiece a asignar direcciones a partir de la 10.42.0.2 (hasta la 10.42.0.254). Por eso, a nuestro portatil le ha asignado la direccion 10.42.0.2, y no es porque en nuestro portatil le hayamos dicho que es esa direccion la que queremos, ya que tenemos puesto, como hemos dicho antes, todo en automatico tanto ipv4 como ipv6.

Hay que hacer https://developer.dji.com/onboard-sdk/documentation/M210-Docs/real-world-test-checklist.html antes de cada vuelo. Este archivo está en /programming/Onboard-SDK/utility/bin/armv8/64-bit/64-bit, y hay 3: entiendo que el que importa es el UserConfig.txt a secas. LO unico que cambia respecto a los otros 2 txt es el app_id y el app_key.

ctrl r
rosservice call /uav_2/dji_control/get_control "data: true"

roslaunch rosbridge_server rosbridge_websocket.launch


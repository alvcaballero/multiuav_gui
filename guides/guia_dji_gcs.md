# DJI SETUP COMPLETO

Nuestro equipo se compone de un portátil que hace las veces de GCS (pero donse se realizan más operaciones adicionales, como la configuración previa) y varios UAVs. 

Esta guía está realizada para dos UAVs (M210 matrice v2, DJI).


Cada dron tiene una mochila atornillada encima. En esta mochila se encuentra una placa (raspberry, por ejemplo) que hace las veces de OSDK externo. También tiene lo necesario para comunicación aérea, pues tiene una antena conectada a otro módulo.

De esta mochila salen varios cables:

- Un cable ethernet que se conecta en el mismo puerto ethernet de la mochila. Sin embargo, si se quiere se puede acceder directamente a la raspberry si se alimenta y se lleva un cable ethernet desde el portátil hasta el puerto ethernet de la mochila. COn esta útilma configuración, obviamente, no se puede volar.

- Un cable que se encarga de alimentar la mochila, que acaba en un conector amarillo XT30 y que debe ir al puerto I/O del dron. Este puerto es el que está más a la izquierda del dron, viendo los puertos desde enfrente. Si este puerto está bien configurado, al encender el dron y tras haber sonado los pitidos del dron, a los pocos segundos se escucharán unos pitidos más estridentes que corresponden a la raspberry encendiéndose. Es importante estar atento a estos últimos pitidos, porque puede pasar que la raspberry no se haya iniciado. Si no ha sonado la causa más probable es que el puerto de alimentación del dron está apagado, por lo que hay que irse a la consola del mando radiocontrol del dron, darle a ajustes (los tres puntos de arriba a la derecha), estar en la pestaña que tiene un dron dibujado y bajar hasta abajo del todo donde pone ajustes de puerto expandidos. Una vez dentro, encender el puerto de alimentación si estaba apagada la opción, y guardar los cambios saliendo, y si estaba encendida, apagarla, salir y guardar, entrar de nuevo, activarla y salir y guardar. Es importante señalar que en determinados casos el dron bloquea este puerto, como cuando se realiza un lock de seguridad por volar demasiado bajo, por lo que no hay que confiarse por haber señalado esta opción a lo largo del día: si la raspberry no se enciende, mirad esto.

    ```
    Flight Controller Settings
    ---- Endended IO option
    -------- Enable power supply port
    ```

- Otro cable RX, TX y GND que va conectado en los pines de la derecha, en el tercero empezando por la izquierda en posición vertical, debiendo estar el GND arriba. De todas formas si hay duda con esto o con cualquier otra cosa del conexionado se puede consultar la guía de usuario del DJI M210 matrice v2.

- Por último, un cable USB-USB oficial de DJI conectado entre cualquiera de los puertos USB de la mochila y el puerto USB del dron. Es MUY IMPORTANTE que el switch que está a la izquierda de puerto USB esté en su posición a la derecha, ya que si está a la derecha indica que se va a conectar un dispositivo externo al puerto USB del dron que hará las veces de ordenador externo, mientras que si está a la izquierda indica que el puerto USB se va a usar solo para dar alimentación. Es importante recalcar que no hay que asumir que el switch, aunque aparentemente esté en su posición correcta, esté haciendo bien contacto. Si la raspberry no detecta el puerto ACM0, el correspondiente a un USB, y el switch está a la derecha, seguramente sea que no hace bien contacto. Hay que asegurarse de ello moviéndolo varias veces y teniendo especial cuidado.

# Preparacion de comunicaciones

En cuanto a la preparación de las conexiones en tierra, el esquema sería el siguiente.Tenemos una Ubiquiti rocket de la que sale un cable Ethernet. Este cable va conectado al puerto POE del módulo de alimentación de la Ubiquiti, y este módulo debe estar enchufado. Del puerto LAN del módulo se saca otro cable Ethernet hacia un router, que se encarga de conectar todas estas conexiones. Este cable lo conectaremos al segundo puerto del router, que también se enchufa a corriente.

En el primer puerto (empezando desde la izquierda, antes igual) del router se conecta un cable Ethernet que va hasta una raspberry (no tiene nada que ver con las placas qeu van encima de los drones, esta es otra y está en tierra), proporcionada por Diego, que sirve para poder tener Internet en el sistema. Esta Raspberry tiene que conectarse a corriente, con un cargador microUSB oficial de Raspberry. Esta Raspberry se conectará automáticamente a un móvil puesto en modo zona WiFI, previamanete habiendo puesto como nombre del móvil y contraseña de la zona WiFi lo que se indica en la pegatina de la raspberry.

Por último, en el tercer puerto del router se saca otro cable ethernet hasta el puerto ethernet del portátil. Es importante que el wifi del portátil esté apagado o no funciona bien conectado a la Ubiquiti, de todas formas el internet ya nos llega vía ethernet gracias a la zona wifi.

Hecha toda está configuración, se puede pasar a encender el sistema.

Se asume conectado ya a corriente el router, el módulo de aliemntación de la UBiquiti, y la raspberry que recoge la zona wifi. En la UBiquiti deben estar encendidas luces azules pero apagadas las cuatro luces de señal.

Se encienden los drones, pulsando una vez y después manteniendo pulsando el botón semitransparente de los drones. Hay que estar atento a que, como escribí anteriormente, tras el sonido de inicio de cada dron suene el sonido de inicio de la raspberry que lleva encima.


Cuando el primer dron se conecté a la UBiquiti, en esta se iluminarán las cuatro luces de señal. Este proceso puede durar hasta cinco minutos, y no hay manera de saber ahora mirando la Ubiquiti cuándo se conectan el resto de drones.

Para acceder a los distintos drones accedemos remotamente con ssh. En un terminal escribimos "ssh ubuntu@10.42.0.42" para acceder al UAV 2, y "ssh grvc@10.42.0.99" para acceder al UAV 1. Las contraseñas son, respectivamente, ubuntu y grvc. Este proceso habrá que hacerlo en cada terminal que se quiera tener abierto con acceso a cada dron, yo recomiendo cuatro terminales para cada UAV y para el portátil (12 en total).

Lanzamos roscore en cada UAV y en el portátil.

En los UAVS DJI lanzamos el comando M210-real (o m210-sim-on). Esto no es más que un shortcut para lanzar un comando bastante largo, que se puede leer si accedemos al .bashrc, que le indica al dron que somos conscientes de que vamos a volar con un dispositivo conectado al puerto USB y que queremos hacerlo. Si no lanzamos esto, entonces entrará en bloqueo de seguridad y no podremos moverlo.

Lanzamos los atlas.launch tal y como viene abajo, esto lanza los nodos de los UAV (UAV_1  y UAV_2, haciendo rosnode list en cada máquina se deberían ver pero todavía no se comparten entre ellos, no son visibiles). Como en este caso no tenemos drones px4, no lanzamos los launch del px4 que viene en la guía.

Para que todos los nodos sean visibles entre ellos usamos el multimaster. Lanzamos en cada UAV el multimaster.launch, y en el portatil el connect_uas.launch. Si vamos haciendo rosnode list en el portátil durante este proceso varias veces, veremos como va encontrando los nodos UAV_1 y UAV_2 que están corriendo en las otras máquinas, y los añadirá.

Ahora desde la carpeta aerialcore_gui hacemos "npm run dev", se nos abre la interfaz. 

COnectamos ROS.

Añadimos drones: uav_1 de DJI, y uav_2 de DJI. Si cuando se añaden algún dron la GCS no ha pillado bien su posición GPS, reinicia ese dron.


DRON DJI (aterrizaje en estacion de carga) - uav_1

    Ricardo y Batista lo controlan (Similar al DJI uav_2)

## DRON DJI - uav_2
1. ```ssh ubuntu@10.42.0.42 ```  //pwd: ubuntu

1. Verificar coneccion(hacer ping)
1. Lanzar Roscore
1. Indicar al drone que se va alanzar simulacion ```$M210-sim-on``` 
1. Lanzar dji_vehicle_node  ```$roslaunch aerialcore_onboard_dji atlas.launch```
1. Lanzar el multimasater ```roslaunch aerialcore_onboard_dji multimaster.launch```
1. En el caso de que el vuelo sea real  hay que quitarle el comando al piloto ```rosservice call /uav_2/dji_control/get_control "data: true```

### VTOL - uav_3

Se lanza en la GCS. Comprobar con la QGround que se reciben las misiones correctamente. En el archivo atlas.launch situado en la carpeta aerialcore_onboard_px4 se puede modificar la ip a la que se envia la telemetria (pc de Victor. Abrir ahi la Qground)

### MARVIN FUVEX - uav_4

Depende de Fuvex. SU GCS ha de conectarse al puerto 3000 de la GCS USE (éste pc) para conectar los websockets

## PC - GCS

- Conectar antena al pc en puerto serie (receptor del VTOL)
- Lanzar roslaunch ```aerialcore_onboard_px4 atlas.launch``` (en el Vtol)
- Lanzar el multimaster ```roslaunch aerialcore_gui connect_uas.launch```(ya levanta roscore)
- Lanzar rosbridge_server ```roslaunch cd .. rosbridge_websocket.launch``` (necesita los archivos de grvc)
- Lanzar GUI (desde la carpeta "aerialcore_gui" -> ```npm run dev```)
- conectar Ros
- añadir drones
Revisar bien los uav_id que coincidan con los correspondientes en la mision.
- Cargar mision
- Lanzar mision


# configuraciones iniciales en drone y en la pc 
Para que se comunicuuqe con ros master

## Configuracion en la pc
ere are lots of resources for it and can be easily implemen
1. instalacion de multimaster

    sudo apt-get install ros-melodic-multimaster-fkie

2. Configuracion ip en el caso de estar conectado con cable a 10.42.0.2

configuracions host localhost

    sudo nano /etc/hosts
    10.42.0.42 ubuntu-black


## configuracion de Drone

Conexion 

    ssh ubuntu@10.42.0.42   //pwd: ubuntu

 configuracion en host  del drones

    sudo nano /etc/hosts
    10.42.0.2  nombre_pc

 configuracion de multimaster en drone 

    roscd aerialcore_onboard_dji/
    cd launch    
    nano multimaster.launch
        anadir en robot_hosts  nombre_pc
        anadir en static_hosts  nombre_pc



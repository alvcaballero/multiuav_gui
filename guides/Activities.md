
# How actually work:


    El sistema se basa en 2 aplicaciones:
        1. sistema de escritorio usando el framework Electron (/src/index.js) que se visualiza los drones
        2. una pagina web levantada en http://localhost:3000 (/public/index.html) que se sirve para enviar comandos.

    

    The sistem read the topic of the uav  usando ROSLIB.Topic:
        dji UAV:  http://wiki.ros.org/dji_sdk
            uav_3/dji_osdk_ros/rtk_position



# how will work:
    El sistema  usa ros bag vscode guardar los datos recopilados del vuelo http://wiki.ros.org/rosbag
        https://stackoverflow.com/questions/50652887/kill-rosbag-gracefully-via-kill
        rosbag record -O session2_2023-01-03-10-50-02.bag  -e "/uav_3/dji_osdk_ros/(.*)" __name:=node_bag_uav_3
        
        rosnode kill /node_bag_uav_3


        rosbag play recorded1.bag recorded2.bag


        rosnode kill /uav_1-rosbag

        rosbag info session2_2023-01-03-10-50-02.bag

        https://github.com/foxglove/rosbag/


## Como obtener la informacion del rosbag
    rosbag info session*.bag           // 
    rosbag info -y /path/to/my.bag     //
    rosbag check old.bag               // revisa el estado del paquete e informa si es necesario alguna migracion.
    rqt_bag session*.bag               //

rosbag with 
https://github.com/foxglove/rosbag


31/01/2023
    


30/01/2023

    Se realizo cambios para que se pueda visualizar la respuestas de los servicios en node mission, que es doende se colorara el rosbag
    tambien se cambio la interfaz

    queda pendiente establecer el codigo del rosbag. y talvez instalar vnc, a la raspberry no tiene internet u esta medio complicado actualizar los paquetes  se utiliza github
    

27/01/2023
    Se esta revisando la respuesta de los servicios levantados en mission node para dar una visualizacion del estado del drone
    node_mission --------- gcs
    config_mission()    ->loadMission()
    run_mission()   ->commandMission()

    esto se piensa a escribir en

    uavTable - info pero actualmente es esta escribiendo conected cada vez que recibe undato subcriptor.
    no cambia la palabra connectec  cuando se desconecta el drone.
    se puede llavar con una funcion con timer   y con una bancdera   si no recibe el topic despues  de cierto tiempo si no poner como desconectado, este tiempo podria ser 5 segundos pero tiene que ser mayor al tiempo de envio del valor.
        showData[5].innerHTML = "Connected"


26/01/2023
    ----------fui pero me puse a estudiar

25/01/2023
    se reviso como enviar las misiones otra vez y se tubo un error 

24/01/2023
    Se tubo un error  y se soluciono. (estoy llenado despues y no me acuerdo que hice pero si trabaje )

23/01/203

    se correcjio que  se si ya existe el dispositivo ingresado no lo ingreses de nuevo.
    hay un problema de eleminacion de las filas.
    se detecto  que el rosbridge tiene que correr en el lado del px4.

20/01/2023

    Se trato de conectar con el dron y ver el funcionamiento para poder realizar el rosbag dentro de drone.

19/01/2023

    Realizacion de pruebas anadir una nueva pagina 

18/01/2023

    se esta buscando crear una nueva pestana donde se pueda visualizar de cada uno de los dispositivos que se encuentrans 
    y ver camara y valores mas detallados

17/01/2023

    se tratata de crear una pestana donde se colocara una  una ventana nueva y se tratara de ver la camara en la interfaz.
     se trato de poner las bases de donde y como se cambiara la interfaz

16/01/2023

    Conversacion con Alvaro para ver como avanza el proyecto y futuros trabajos se determino que se hiba avanzar en la interfaz especialmente en visualizar la camara.

13/01/2023

    Se probo usando los comandos de GRVC para el levantamiento de las misiones sin tener exito de momento para las pruebas del rosbag.
    pero se logro ejecutar el multi auv con el un lauch  que viene en px4 ya integrado.

12/01/2023

    se mosdifico  el archivo run_pxsiltl.py
    se anadio la linea 44 y se comento la 45
    esto es poque no se encontraba el archivo a que hace referencia en el codiogo.
    igaul se sige teniedo problemas con los datos que se reciben  no se estan recibiendo datos de los topic debido a que no se ejecuta correctamente el script.

11/01/2023

    se legro ejecuatar roslaunch aerialcore_onboard_px4 simulation.launch ns:=uav_2
    sin errores y logrando enlasar a PX4, pero no se conecta con la qtground control.eso queda pendiente de resolver.

10/01/2023

    Se logro instalar px4 y realizar la simulacion de 1 vtol y sesimulo.
    y se documento en un Readme

09/01/2023

    Se trato de realizar la compilacion de rosbag en la computadora de carlos, y determine que es necesario realizar  la mision y guardar los datos directamente en 1 rosbag
    se trato de instalar px4 de los paquetes pasados por Carlos pero se fallo en el intento.

05/01/2023

    Se termino las balidaciones y las pruebas utilizando.
    Se elecminaron bug
    

04/01/2023

    Se realizo la validacion de el ingreso del UAV.
    se realizo en codigo para el envio y cierre automatico del  rosbag , falta probar    

03/01/2023

    Se continuo con los trabajos en la interfaz grafica, ya que no se pudo avanzar con los archivos del proyecto.
    se realizo los comandos de rosbag 
    y se desarrollo otro contenedor que hara la interfaz grafica.

30/12/202

    Se trato de realiza el la compilacion de los archivos se soluciono los problemas del github
        locale
        export LANGUAGE=en_US.UTF-8; export LANG=en_US.UTF-8; export LC_ALL=en_US.UTF-8; locale-gen en_US.UTF-8
        dpkg-reconfigure locales

    Se trato de ver si con otro contenedor se tiene los mismo problemas, y efectivamente se tubieron los mismo problemas.

29/12/2022

    se trato de realizar la instalacion del contenedor.

28/12/2022

    se realizo 
    se reviso como instalar firmware de PX4
        https://hippocampusrobotics.github.io/fav_docs/installation/install_firmware.html
        https://qiita.com/k-koh/items/1fd81f0af4b9c9b98a1e
        https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html


27/12/2022

    rosbag like a shell script
    how stop rosbag like a shell scritp
    Se pudo realizar un contenedor con que funcione como maquina virtual.

23/12/2022

    Investigacion de rosbag
    pruebas de rosbag
    roslaunch rosbridge_server rosbridge_websocket.launch


22/12/2022

    pedir el rosbag
    simular datos de bateria y velocidad en drone
    busqueda de informacion de rosbag en npm

 21/12/2022- hoy no vino carlos 

 Revision de funcionamiento de rosbag

 20/12/2022

    Revision de codigo

19/12/2022

    Revision de codigo interfaz grafica
    Revision de estructura de codigo

16/12/2022

    Preguntas y consultas a carlos 

15/12/2022

    ingreso al trabajo
    Preguntas y consultas a carlos
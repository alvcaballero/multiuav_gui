# GRVC - OMICRO
Este es un proyecto que lleva desarrollo de antemano y esta realizado utilizando ubunto 18 con ros noetic, autopilot.

Para continuar con estos trabajos  yo estoy utilizando ubuntu 22  y docker.

# Composicion del proyecto 
- Autopilot
- Paquetes desarrollados
- interfaz grafica

##  Autopilot

I dont have ubuntu 18 in my personal machine,this is reason for work with docker and beacuse docker do the thingks more easy to share proyect.

To run the proyect we try to install the firmware folder for simulate a drone.
por lo cual se uso la siguiente imagen de docker, si estas interesado en utilizar docker con ros puedes revisar este [link ](https://roboticseabass.com/2021/04/21/docker-and-ros/) y el [docker hub](https://hub.docker.com/r/px4io/px4-dev-ros-melodic)

    docker pull px4io/px4-dev-ros-melodic   

se clono la el repositorio

    git clone https://github.com/PX4/PX4-Autopilot.git --recursive


y se esta usando el siguiente scrip para poder  acceder a la maquina

    ~/work/px4$ ./px4-docker-melodic.sh

 se siguio las instrucciones del siguiente [tutorial](https://docs.px4.io/main/en/test_and_ci/docker.html)

y se ejecuto la siguiente  linea de codigo para simular el Vtool

    ~/src/PX$-Autopilot$ make px4_sitl gazebo_standard_vtol
    or
    make px4_sitl_default gazebo

    make px4_sitl gazebo_typhoon_h480 (este tiene camara)

y para conectar a ros se sigio el siguiente [tutorial](https://dev.px4.io/v1.9.0_noredirect/en/simulation/ros_interface.html).

    roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"

para usar utilizando las herramientas de Px4


    cd /home/user/src/PX4-Autopilot
    source Tools/simulation/gazebo/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
    export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd):$(pwd)/Tools/simulation/gazebo/sitl_gazebo

se baso en este link    http://docs.px4.io/main/en/simulation/multi_vehicle_simulation_gazebo.html


    roslaunch px4 multi_uav_mavros_sitl.launch


## GNS  - interfaz grafica

Para la interfaz se a realizado  de la misma manera un contenedor en ros  melodic, que  se puede conectar a cualquier ros maste en la misma maquina .

**cabe recalcar que este contenerdor hay que colocarle un usuario a lo que se cree el contenedor ya que el usuario que esta actualmente no se presenta correctamente y electron no vale  ejecutar como root**






## paquetes -GRVC

#### instalar onboard-sdk
you need to download onboard-sdk4.1.0,and install it.
>$mkdir build  
>$cd build  
>$cmake..  
>$sudo make -j7 install

como no se puede compilar el archivo de px4 se tiene el siguiente archivo
 ahora se ha  cambio la ubicacion de px4 en los siguientes arhcivos de px4_bringup
        run_px4sitl.py              --  linea 32
        launch_gzworld.py           --  linea 41


esto soluciona el problema de que ya ejecuta

    roslaunch aerialcore_onboard_px4 simulation.launch ns:=uav_2

    pero no se conecta a la qtground controler por lo que se ha cambiado a 
     name="fcu_url" default="udp://:14540@127.0.0.1:14557"
     pensando que este es el error.
    rostopic echo /uav_2/mavros/battery
    launch/spawn_robot.launch ---  linea 28



se mosdifico  el archivo run_pxsiltl.py
se anadio la linea 44 y se comento la 45
 esto es poque no se encontraba el archivo a que hace referencia en el codiogo.


ahora PX4 ya tiene una propia herramienta para la gestion de  multi-uav  por lo caul no se continua  con los paquetes de GRVC

    http://docs.px4.io/main/en/simulation/multi_vehicle_simulation_gazebo.html

## modificaciones  gazebo

/home/arpa/work/px4/src/PX4-Autopilot/Tools/simulation/gazebo/sitl_gazebo/models/typhoon_h480

 se adio en  la linea 404 
         <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
          <frame_name>cgo3_camera_link_optica</frame_name>
        </plugin>
 ys e descubrio que el protocolo maplink manda por estos puertos las imagenes.
         <plugin name="GstCameraPlugin" filename="libgazebo_gst_camera_plugin.so">
            <robotNamespace></robotNamespace>
            <udpHost>127.0.0.1</udpHost>
            <udpPort>5600</udpPort>
        </plugin>
        <plugin name="CameraManagerPlugin" filename="libgazebo_camera_manager_plugin.so">
            <robotNamespace>typhoon_h480</robotNamespace>
            <interval>1</interval>
            <width>3840</width>
            <height>2160</height>
            <maximum_zoom>8.0</maximum_zoom>
            <video_uri>udp://127.0.0.1:5600</video_uri>
            <system_id>1</system_id>
            <cam_component_id>100</cam_component_id>
            <mavlink_cam_udp_port>14530</mavlink_cam_udp_port>
        </plugin>
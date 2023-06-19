Red GIORGIO (Contraseña: grvc1234)

Antes de usar fastcom en una nueva computadora hay que correr:
echo "export FASTCOM_MOLE_IP=\"10.42.0.99\"" >> ~/.bashrc

Fastcom se usa con python con el codigo test_image_subscriber.py en la carpeta fastcomm/python/
La configuracion es:
pub = fastcom.ImageSubscriber.ImageSubscriber("10.42.0.99", 8888)


COMANDO LANDING CON CONFIGS QUE FUNCIONAN
rosrun autonomous_landing state_machine src/autonomous_landing/config/config.yaml src/autonomous_landing/config/config_tracker.yaml 
El compilado 1

Para cambiar el namespace:
export ROS_NAMESPACE=uav_1

comandos para iniciar el aterrizaje:
1- Iniciar el nodo de autonomous landing al principio (Antes de despegar):
-> cd programming/landing_ws/
-> source devel/setup.bash
-> rosrun autonomous_landing state_machine src/autonomous_landing/config/config.yaml src/autonomous_landing/config/config_tracker.yaml  
(compilado 1)

OPCIONAL PERO RECOMENDADO: en la computadora (no en la jetson) correr fastcom como se explica arriba

UNA VEZ EL DRON ESTA ENCIMA DE LA PLATAFORMA:
2- Mover la gimball: rosservice call /uav_1/autonomous_landing/move_gimbal "data: true" 
3- Activar el aterrizaje: rosservice call /uav_1/autonomous_landing/activate_visualservoing "data: true"

SI NOS QUITAN EL CONTROL, SE RECUPERA CON: 
rosservice call /uav_1/dji_control/get_control "data: true" 







ARRANCAR CÁMARA:
Asegurarse de que el código de test_image_subscriber.py tiene la línea de publicador como: pub = fastcom.ImageSubscriber.ImageSubscriber("10.42.0.99", 8888)
Asegurarse de que la IP de fastcom está en bashrc. Si no, hacer echo "export FASTCOM_MOLE_IP=\"10.42.0.99\"" >> ~/.bashrc


TERMINAL 1:


cd ~/Instalaciones/fastcom/python
python3 test_image_subscriber.py


TERMINAL 2:

ssh grvc@10.42.0.99
cd ~/programming/landing_ws
source devel/setup.sh
#Lanzar master y arrancar /uav_1

TERMINAL 3:

ssh grvc@10.42.0.99
cd ~/programming/landing_ws
source devel/setup.sh
export ROS_NAMESPACE=uav_1
rosrun autonomous_landing state_machine src/autonomous_landing/config/config.yaml src/autonomous_landing/config/config_tracker.yaml 





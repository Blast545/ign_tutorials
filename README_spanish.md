# Tutorial: Nodo ROS teleop_twist_keyboard ROS para controlar un robot diff drive en Ignition

Este tutorial tiene como propósito dar una introduccion al uso de las herramientas de ROS en conjunto a una simulacion corriendo en Ignition.
Al final de este tutorial, tendrá un resultado similar al que se muestra en este [video](https://youtu.be/2ZiRw2ZkTOY).

Para este tutorial usaremos Ignition Dome y ROS Melodic; no obstante,
tenga en cuenta que este tutorial es valido para releases de Ignition compatibles con `ros_ign_bridge`
y releases de ROS compatibles con `teleop_twist_keyboard` y `ros_ign`.

Primero completaremos los pasos del tutorial y posteriormente explicaremos que sucede detrás de escenas.

## Pre-Requisitos

+ Instalar Ignition Dome
	+ Enlace a las instrucciones: [Dome Installation](https://ignitionrobotics.org/docs/dome)

+ Instalar ROS Melodic
	+ Enlace a las instrucciones: [ROS Melodic Installation](http://wiki.ros.org/melodic/Installation)

+ Instalar `ros_ign`. Usaremos ROS Melodic
	``` bash
        # Agregar https://packages.osrfoundation.org
        sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
        wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
        sudo apt-get update
        # Instalar `ros_ign`
        sudo apt install ros-melodic-ros-ign
	```

+ Instalar `teleop_twist_keyboard`
	``` bash
	sudo apt-get install ros-melodic-teleop-twist-keyboard
	```

## Pasos para completar este tutorial

### Instalar este paquete ROS con el demo

``` bash
mkdir -p ~/ign_teleop_tutorial_ws/src
git clone git@github.com:Blast545/ign_tutorials.git ~/ign_teleop_tutorial_ws/src
cd ~/ign_teleop_tutorial_ws
catkin_make
```

Abriremos dos terminales que compartan el mismo entorno. En cada una, correremos los siguientes comandos:

#### Terminal 1: launchfile con ROS, Ignition y `ros_ign`
```bash
# Shell 1
source ~/ign_teleop_tutorial_ws/devel/setup.bash
roslaunch roslaunch ign_tutorials diff_drive_demo.launch
```

#### Terminal 2: nodo `teleop_twist_keyboard`
```bash
# Shell 2
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

Después de estos pasos, queda un entorno similar al siguiente:
![Selection_092](https://user-images.githubusercontent.com/8069967/113123464-fe509200-91ea-11eb-89a4-6d4a8406e6ef.png)

Por ultimo, usando la terminal con el nodo teleop_twist_keyboard se pueden enviar comandos para mover el robot:
![Selection_091](https://user-images.githubusercontent.com/8069967/113123514-0ad4ea80-91eb-11eb-989b-46ce8fbaefdc.png)

## Entendiendo la arquitectura del sistema

Para este tutorial hemos usado el mundo de ejemplo `tunnel.sdf`, este incluye un vehículo tipo "diff-drive" y contiene varias funcionalidades
disponibles del simulador Ignition. El sistema de comandos para el plugin diff drive funciona de esta manera:

![diagrama_tutorial (1)](https://user-images.githubusercontent.com/8069967/113031878-99068d80-9165-11eb-8fce-2e31c104e5eb.png)

* El nodo `teleop_twist_keyboard` toma las teclas presionadas por el usuario y las convierte en comandos ROS Twist. 
* `ros_ign_bridge` se subscribe a esos comandos ROS Twist y los convierte en mensajes Ignition Transport.
*  Los mensajes Ignition Transport son usados por el plugin diff drive ejecutado como parte de la simulacion del `tunnel.sdf`.
*  El simulador mueve al robot apropiadamente, de acuerdo a los mensajes recibidos desde el plugin diff drive.

Podemos observar la parte del codigo del launchfile que usa el ros_ign_bridge:

```xml
  <node
    pkg="ros_ign_bridge"
    type="parameter_bridge"
    name="$(anon ros_ign_bridge)"
    output="screen"
    args="/model/vehicle/odometry@nav_msgs/Odometry@ignition.msgs.Odometry /cmd_vel@geometry_msgs/Twist@ignition.msgs.Twist ">
  </node>
```

Los mensajes van y vienen a través del tópico `/cmd_vel`, este tópico es usado tanto en la capa de Ignition Transport como del lado de ROS. 
De entrada, toma mensajes `geometry_msgs/Twist` y de salida usa mensajes de tipo `ignition.msgs.Twist`.
Lo mismo aplica para la configuración de odometría mostrada en `rviz`.

Por último, mirando el código del archivo SDF `ign_ws/src/ign-gazebo/examples/worlds/tunnel.sdf` vemos que:

``` xml
<plugin
  filename="ignition-gazebo-diff-drive-system"
  name="ignition::gazebo::systems::DiffDrive">
  <left_joint>left_rear_wheel</left_joint>
  <left_joint>left_front_wheel</left_joint>
  <right_joint>right_rear_wheel</right_joint>
  <right_joint>right_front_wheel</right_joint>
  <wheel_separation>1.25</wheel_separation>
  <wheel_radius>0.3</wheel_radius>
  <odom_publish_frequency>1</odom_publish_frequency>
  <topic>cmd_vel</topic>
</plugin>
```

Así queda configurado el plugin diff drive para recibir comandos desde el tópico `cmd_vel` y controlar las 
cuatro articulaciones: `left_rear_wheel`, `left_front_wheel`, `right_rear_wheel` y `right_front_wheel` 
que controlan el movimiento del robot.

## Pasos adicionales

Como ejercicio, ¿Puede tomar alguna de las siguientes tareas?

* Enviar mensajes al robot directamente usando Ignition Transport.
  * Puede chequear este tutorial de ignition: [Moving the robot](https://ignitionrobotics.org/docs/dome/moving_robot).
  * Revise la estructura del model sdf [`tunnel.sdf`](https://github.com/ignitionrobotics/ign-gazebo/blob/ign-gazebo4/examples/worlds/tunnel.sdf#L1575).
* ¿Puede cambiar el launchfile para usar otro motor de física? como TPE.
  * Revise el tutorial de `ign-physics`: [Switching physics engines](https://github.com/ignitionrobotics/ign-physics/blob/ign-physics3/tutorials/04-switching-physics-engines.md).
* El mundo de ejemplo `diff_drive.sdf` (`ign_ws/src/ign-gazebo/examples/worlds/diff_drive.sdf`) no usa el mismo tópico `cmd_vel` para recibir comandos de velocidad, 
  ¿Puede modificar el tutorial para usar este mundo de ejemplo y enviar comandos a alguno de esos robots?
  * Puede revisar `diff_drive.sdf` [example commands](https://github.com/ignitionrobotics/ign-gazebo/blob/ign-gazebo4//examples/worlds/diff_drive.sdf#L7) y ajustar los`ros_ign` bridges apropiadamente.

¿Quiere saber mas de Ignition Gazebo? Revise los siguientes enlaces:

* Portal principal del simulador [Ignition portal](https://ignitionrobotics.org/home).
* Revise los tutoriales oficiales [tutorials for Ignition Dome](https://ignitionrobotics.org/docs/dome).
* Discussiones y anuncios en [Gazebo/Ignition community forum](https://community.gazebosim.org/).

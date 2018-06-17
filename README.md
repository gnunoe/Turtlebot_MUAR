# Tutlebot_MUAR

ARCHIVOS PARA EL TRABAJO DE SIMULACION DE TURTLEBOT DE LA MUAR

INSTALAR ANTES LOS SIGUIENTES PAQUETES:

    $sudo apt-get install 

    ros-kinetic-turtlebot 
    
    ros-kinetic-turtlebot-apps 
    
    ros-kinetic-turtlebot-interactions 
    
    ros-kinetic-turtlebot-simulator 
    
    ros-kinetic-kobuki-ftdi
    
    ros-kinetic-ar-track-alvar-msgs
    
    ros-kinetic-frontier-exploration
    
    ros-kinetic-aruco-ros
    
INSTALAR DARKNET-YOLO

  Debido al almacenamiento limitado, el paquete de reconocimento de objetos se debe descargar en el enlace https://drive.google.com/open?id=1raN7mVBISUNW29vJXJtU6k76f0SNX7g5. Para compilarlo, añadir la carpeta darknet_ros al /src y ejecutar el siguientew comando:

    catkin_make -DCMAKE_BUILD_TYPE=Release --pkg darknet_ros


INCLUIR EN EL .bashrc LOS SIGUIENTES COMANDOS:

    export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/catkin_ws/src/labrob_gazebo/models
    
PRIMERA ENTREGA (Robot, sensores y mundo)

  Ejecutar el siguiente comando:
  
    roslaunch turtlebot_gazebo turtlebot_lab.launch 
    
  Para teleoperar el robot, usar el siguiente comando en otra terminal:
  
    roslaunch turtlebot_teleop keyboard_teleop.launch

SEGUNDA ENTREGA (Navegación básica y detección de obstáculos)

  Ejecutar el siguiente comando donde ROBOT_INITIAL_POSE es la posicion en la escena (colocarlo en la esquina). Se carga uno de los mapas finales por defecto (ToDo: No hace spawn de dos objetos):
  OJO: A veces image rectification peta, comprobarlo!
  
    ROBOT_INITIAL_POSE="-x 8.5 -y -8.5" roslaunch turtlebot_gazebo turtlebot_lab.launch
 
  Para genera el mapa ejecutar el comando, donde arucoNum es el número del aruco (Si no estan todos los objetos, ejecutarlos varias veces. Se pueden mover todos los objetos a mano para distribuirlos excepto monedas y bombas) 
  OJO: laas monedas deberian desaparecer, pero no siempre lo hacen. Si el robot ha tocado una y no desaparece, suprimir la moneda de manera manual en gazebo para que continue;
    
    roslaunch labrob_gazebo plugin_sim.launch arucoNum:=1 
 
  Ejecutar el YOLO para reconocimiento de objetos (He desactivado la imagen para que vaya menos petado, pero se puede activar en su archivo de configuracion) 

    roslaunch darknet_ros yolo_lab.launch
   
  Activar stack de navegación + exploración con el siguiente comando
  
    roslaunch turtlebot_gazebo gmapping_demo_frontier.launch
    
  Ejecutar scheduler, el cual organizará los pasos de la mision (Buscar Aruco, Explorar y marcar objetos). 
    
    roslaunch simple_navigation_goals lab_mission.launch

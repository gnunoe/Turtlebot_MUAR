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
    
INCLUIR EN EL .bashrc LOS SIGUIENTES COMANDOS:

    export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/catkin_ws/src/labrob_gazebo/models
    
PRIMERA ENTREGA (Robot, sensores y mundo)

  Ejecutar el siguiente comando:
  
    roslaunch turtlebot_gazebo turtlebot_lab.launch 
    
  Para teleoperar el robot, usar el siguiente comando en otra terminal:
  
    roslaunch turtlebot_teleop keyboard_teleop.launch

SEGUNDA ENTREGA (Navegación básica y detección de obstáculos)

  Ejecutar el siguiente comando donde ROBOT_INITIAL_POSE es la posicion en la escena (colocarlo en la esquina):
  
    ROBOT_INITIAL_POSE="-x 8.5 -y -8.5" roslaunch turtlebot_gazebo turtlebot_lab.launch
    
  Para genera mapa aleatorio usuando plugin, ejecutar siguiente comando.
    
    roslaunch labrob_gazebo plugin_sim.launch 
   
  Activar stack de navegación con el siguiente comando
  
    roslaunch turtlebot_gazebo gmapping_demo_2.launch
    
  Enviar Navigation Goal cercano al Aruco (Principio de misión)
    
    rosrun simple_navigation_goals simple_navigation_goals

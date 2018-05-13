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

  Ejecutar el siguiente comando donde ROBOT_INITIAL_POSE es la posicion en la escena (colocarlo en la esquina). Se carga uno de los mapas finales por defecto (ToDo: No hace spawn de dos objetos):
  
    ROBOT_INITIAL_POSE="-x 8.5 -y -8.5" roslaunch turtlebot_gazebo turtlebot_lab.launch
    
  Para genera el aruco ejecutar siguiente comando, donde arucoNum es el número del aruco (ToDo:Se carga otro objeto cada vez que hace spawn).
    
    roslaunch labrob_gazebo plugin_sim_aruco.launch arucoNum:=8 
   
  Activar stack de navegación + exploración con el siguiente comando
  
    roslaunch turtlebot_gazebo gmapping_demo_frontier.launch
    
  Ejecutar scheduler, el cual organizará los pasos de la mision (Buscar Aruco, Explorar y marcar objetos). ToDo: matar proceso por terminal xq esta en un bucle while, necesito depurarlo.
    
    rosrun simple_navigation_goals scheduler

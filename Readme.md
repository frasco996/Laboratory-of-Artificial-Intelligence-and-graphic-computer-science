# Collision Avoidance di un robot in ROS

# Che cosa fa?

Lo scopo di questo nodo di ROS è quello di far muovere un robot e quando la distanza del robot rispetto ad un ostacolo scende sotto una certa soglia, il robot gira con un certo momento angolare e trova una nuova via.

# Come farlo girare

1) Copiare il pacchetto nella cartella #src della cartella #catkin_ws
2) Compilare il codice attraverso il comando #catkin_make
3) Per eseguire aprire 4 terminali
4) Nel primo terminale eseguire il comando 'roscore'
5) Nel secondo terminale eseguire rosrun 'Obstacle_avoidance robot_Avoidance'
6) Nel terzo 'rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=cmd_vel_sub'
7) Nel quarto 'rosrun stage_ros stageros cappero_laser_odom_diag_obstacle_2020-05-06-16-26-03.world'
8) Per muoversi mettere in primo piano il terminale con il telecomando e la mappa e premere il tasto #i


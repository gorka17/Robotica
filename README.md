# Robotica

# Proyecto de Publicador y Suscriptor en ROS

Este proyecto contiene scripts en Python que implementan un sistema de publicación y suscripción utilizando ROS (Robot Operating System). Los scripts permiten la comunicación entre nodos mediante la publicación y recepción de mensajes en diferentes *topics*. Además, se incluyen configuraciones y poses iniciales guardadas en archivos YAML para el control del robot.

## Estructura del Proyecto

- **publisher.py**: Este script actúa como publicador y envía mensajes a varios *topics* para comunicar diferentes comandos y datos. Está pensado para que el usuario pueda teclear qué mensaje quiere publicar y que el robot lo ejecute. La idea es seguir el orden 0-5 para ver el correcto funcionamiento del robot.
- **subscriber.py**: Este script actúa como suscriptor, recibe los mensajes de los *topics* a los que está suscrito y ejecuta las órdenes correspondientes.
- **confs_y_poses.py**: Script que guarda la configuración inicial y la pose inicial del robot en dos archivos YAML ubicados dentro de la carpeta `confs_y_poses`.

### Archivos y Carpetas

- **confs_y_poses**: Carpeta que contiene dos archivos YAML donde se almacena:
  - Una configuración inicial para el robot.
  - Una pose a la que moverse.

#!/usr/bin/env python
import sys
import copy
import rospy
from moveit_commander import MoveGroupCommander, RobotCommander, roscpp_initialize, PlanningSceneInterface
import moveit_msgs.msg
from math import pi, tau, dist, fabs, cos
from std_msgs.msg import String, Int32, Header
from sensor_msgs.msg import JointState
from moveit_commander.conversions import pose_to_list
from typing import List
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from practica_1.msg import Floor

class ControlRobot:
    def __init__(self) -> None:
        roscpp_initialize(sys.argv)
        rospy.init_node("control_robot", anonymous=True)
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.group_name = 'robot'
        self.move_group = MoveGroupCommander(self.group_name)

        # CREAR NODO PUBLICADOR PARA ENVIAR CONF Y POSE DEL ROBOT
        # Inicializar nodos suscriptores.
        self.pose_subscriber = rospy.Subscriber("/mover_pose", Pose, self.pose_callback)
        self.conf_subscriber = rospy.Subscriber("/mover_configuracion", JointState, self.conf_callback)
        self.trajectory_subscriber = rospy.Subscriber("trayectoria_cartesiana", PoseArray, self.trajectory_callback)
        self.obstacle_subscriber = rospy.Subscriber("/añadir_obstaculo", Floor, self.obstacle_callback)

    def get_motor_angles(self) -> list: # Obtener ángulos de motores
        return self.move_group.get_current_joint_values()

    def move_motors(self, joint_goal: List[float]) -> bool: # Mover motores
        return self.move_group.go(joint_goal, wait=True) #wait espera hasta finalizar movimiento antes de seguir con el programa.

    def get_pose(self) -> Pose: # Obtener la pose actual
        return self.move_group.get_current_pose().pose

    def move_to_pose(self, pose_goal: Pose)-> bool: # Mover a una pose
        self.move_group.set_pose_target(pose_goal)
        return self.move_group.go(wait=True)

    def add_to_planning_scene(self, pose_caja: Pose,
                              name: str, tamaño: tuple = (.1, .1, .1)) -> None:

        box_pose = PoseStamped()
        box_pose.header.frame_id = 'base_link'
        box_pose.pose = pose_caja
        box_name = name
        self.scene.add_box(box_name, box_pose, size=tamaño)

    def move_trajectory(self, poses: List[Pose], wait: bool = True) -> bool:
        
        (plan, fraction) = self.move_group.compute_cartesian_path(poses, 0.01)

        if fraction != 1.0:
            print("Trayectoria Fallida")
            return False
        
        self.move_group.execute(plan, wait=wait)

    def add_floor(self) -> None:
        pose_suelo = Pose()
        pose_suelo.position.z = -0.025 # Para no chocar con la base
        self.add_to_planning_scene(pose_suelo, 'suelo', (2, 2, .05)) 

    # Se recibe el mensaje del topic consignas y se ejecuta la acción 
    # correspondiente.
    def pose_callback(self, pose: Pose) -> None:
        print("Moviendo a la pose deseada...")
        self.pose = pose
        self.move_to_pose(self.pose)

    def conf_callback(self, conf: JointState) -> None:
        print("Moviendo a la configuración deseada...")
        self.position = conf.position
        self.move_motors(self.position)

    def trajectory_callback(self, trajectory: PoseArray) -> None:
        print("Siguiendo trayectoria deseada...")

        # Tomamos la posición incicial y luego usamos las poses enviadas.
        self.waypoints = trajectory.poses
        self.waypoints.insert(0, self.get_pose())
        self.move_trajectory(poses=self.waypoints, wait=True)

    def obstacle_callback(self, obstacle: Floor) -> None:
        print("Se ha añadido el obstaculo deseado.")
        self.add_to_planning_scene(obstacle.pose_caja, obstacle.name, tuple(obstacle.tamaño))

    def action(self) -> None:
        # El nodo suscriptor creado en el __init__ empieza a funcionar
        # gracias al .spin() (se activa el callback).
        rospy.spin()

if __name__ == "__main__":
    control = ControlRobot()
    # Empezamos a ejecutar ordenes asociadas a mensajes que llegan.
    control.action()
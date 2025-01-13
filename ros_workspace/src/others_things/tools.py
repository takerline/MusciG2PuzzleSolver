#!/user/bin/python3
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, tau, dist, fabs, cos
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from typing import List
from geometry_msgs.msg import Pose
import yaml
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface
from actionlib import SimpleActionClient

class ControlRobot:

    def __init__(self, name_node: str) -> None:
        """
        Inicializa el nodo de ROS y configura el entorno de MoveIt.

        Args:
            - name_node (str): Nombre del nodo de ROS.
        """
        moveit_commander.roscpp_initialize(sys.argv)

        rospy.init_node(name_node, anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        self.group_name = "robot"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        self.move_group.set_planning_time(10)
        self.add_floor()

        self.move_group = MoveGroupCommander(self.group_name)
        self.gripper_group_name = "gripper" 
        self.gripper_move_group = MoveGroupCommander(self.gripper_group_name)
        self.gripper_action_client = SimpleActionClient("rg2_action_server", GripperCommandAction)

    def get_motor_angles(self) -> list:
        """
        Obtiene los ángulos actuales de los motores del robot.

        Returns:
            list: Lista de ángulos actuales de los motores.
        """
        return self.move_group.get_current_joint_values()

    def move_motors(self, joint_goal: List[float], wait: bool = True) -> bool:
        """
        Mueve los motores del robot a la configuración deseada.

        Args:
            - joint_goal (List[float]): Lista de ángulos deseados para los motores.
            - wait (bool): Indica si se debe esperar a que el movimiento se complete.

        Returns:
            bool: Resultado del movimiento.
        """
        return self.move_group.go(joint_goal, wait=wait)

    def get_pose(self) -> Pose:
        """
        Obtiene la pose actual del efector final del robot.

        Returns:
            Pose: Pose actual del efector final.
        """
        return self.move_group.get_current_pose().pose

    def move_to_pose(self, pose_goal: Pose, wait: bool = True) -> bool:
        """
        Mueve el efector final del robot a la pose objetivo.

        Args:
            - pose_goal (Pose): Pose objetivo a la que se desea mover el robot.
            - wait (bool): Indica si se debe esperar a que el movimiento se complete.

        Returns:
            bool: Resultado del movimiento.
        """
        self.move_group.set_pose_target(pose_goal)
        return self.move_group.go(wait=wait)

    def move_to_trajectory(self, path: List[Pose]) -> None:
        """
        Mueve el robot a través de una trayectoria especificada.

        Args:
            - path (List[Pose]): Lista de poses que definen la trayectoria a seguir.
        """
        path.insert(0, self.get_pose())

        plan, fraction = self.move_group.compute_cartesian_path(
            path, 0.01,  # waypoints to follow  # eef_step
        )

        if fraction != 1.0:
            print(
                "No se puede completar la trayectoria debido a que solo se puede completar el siguiente porcentaje: ",
                fraction,
            )
            return fraction

        self.move_group.execute(plan, wait=True)
        return None

    def add_to_planning_scene(
        self, pose_caja: Pose, name: str = "box", tamano: tuple = (0.075, 0.075, 0.075)
    ) -> None:
        """
        Añade un objeto a la escena de planificación.

        Args:
            - pose_caja (Pose): Pose del objeto que se desea añadir.
            - name (str): Nombre del objeto a añadir.
            - tamano (tuple): Tamaño del objeto a añadir.
        """
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "base_link"
        box_pose.pose = pose_caja
        box_name = name
        self.scene.add_box(box_name, box_pose, size=tamano)

    def add_floor(self) -> None:
        """
        Añade el suelo a la escena de planificación.
        """
        pose_suelo = Pose()
        pose_suelo.position.z = -0.025
        self.add_to_planning_scene(pose_suelo, "suelo", (2, 2, 0.05))

    def write_actual_cartesian_pose(
        self, filename: str = "cartesian_pose.yaml"
    ) -> None:
        """
        Escribe la pose cartesiana actual en un archivo YAML.

        Args:
            - filename (str): Nombre del archivo donde se guardará la pose.
        """
        pose = self.get_pose()

        global_info = {"cartesian_pose": pose}

        with open(filename, "+a") as f:
            yaml.dump(global_info, f)

    def write_actual_motors_pose(self, filename: str = "configuration.yaml") -> None:
        """
        Escribe la configuración actual de los motores en un archivo YAML.

        Args:
            - filename (str): Nombre del archivo donde se guardará la configuración.
        """
        configuration = self.get_motor_angles()

        global_configuration = {"configuration": configuration}

        with open(filename, "+a") as f:
            yaml.dump(global_configuration, f)

    def load_cartesian_pose(self, filename: str = "cartesian_pose.yaml") -> None:
        """
        Carga una pose cartesiana desde un archivo YAML.

        Args:
            - filename (str): Nombre del archivo desde donde se cargará la pose.
        """
        with open(filename, "+r") as f:
            self.load_poses = yaml.load(f, yaml.Loader)

    def load_motor_pose(self, filename: str = "configuration.yaml") -> None:
        """
        Carga la configuración de los motores desde un archivo YAML.

        Args:
            - filename (str): Nombre del archivo desde donde se cargará la configuración.
        """
        with open(filename, "+r") as f:
            self.load_configuration = yaml.load(f, yaml.Loader)

    def mover_pinza(self, anchura_dedos: float, fuerza: float) -> bool:
        goal = GripperCommandGoal()
        goal.command.position = anchura_dedos
        goal.command.max_effort = fuerza
        
        # Attendez que le serveur d'action soit disponible
        self.gripper_action_client.wait_for_server()
        
        self.gripper_action_client.send_goal(goal)
        self.gripper_action_client.wait_for_result()
        result = self.gripper_action_client.get_result()
        return result.reached_goal


if __name__ == '__main__':
    robot = ControlRobot('robot')
    rate = rospy.Rate(1)

    config = robot.get_motor_angles()
    print(config)


    t1_1 = [-1.374493424092428, -2.1778112850584925, -0.437191903591156, -2.095952173272604, 1.5742331743240356, -1.4433601538287562]
    t1_2 = [-1.3774264494525355, -2.038260599175924, -0.9830565452575684, -1.6894885502257289, 1.575183391571045, -1.4442108313189905]
    t2_1 = [-1.2160595099078577, -2.0711685619749964, -0.7529467940330505, -1.8857790432372035, 1.5753809213638306, -1.2431162039386194]
    t2_2 = [-1.2175233999835413, -2.0248166523375453, -1.0042200088500977, -1.6806789837279261, 1.5758346319198608, -1.2435653845416468]
    t3_1 = [-1.0644963423358362, -2.2208391628661097, -0.3601789176464081, -2.1299525700011195, 1.5741199254989624, -1.1335981527911585]
    t3_2 = [-1.0651372114764612, -2.0701753101744593, -0.9336225390434265, -1.7074572048582972, 1.5751261711120605, -1.1320918242083948]

    # robot.move_motors(t3_2)
    # robot.move_motors(t3_1)
    # robot.move_motors(t1_1)
    robot.move_motors(t1_1)
    robot.mover_pinza(100, 7)
    robot.move_motors(t1_2)
    pose = robot.get_pose()
    
    pose.position.z -= 0.061
    # # # pose.position.y += 0.105617977

    # # # pose.position.x += 0.2
    # # # pose.position.y += 0.2
    
    robot.move_to_pose(pose)
    rospy.sleep(2)

    robot.mover_pinza(0,7)
    robot.move_motors(t1_2)
    robot.move_motors(t1_1)
    robot.move_motors(t2_1)
    robot.move_motors(t2_2)

    pose = robot.get_pose()
    pose.position.z -= 0.056
    # # # pose.position.y += 0.105617977

    # # # pose.position.x += 0.2
    # # # pose.position.y += 0.2
    
    robot.move_to_pose(pose)
    robot.mover_pinza(100,7)


    
    #robot.move_motors([0.01643398404121399, -1.3876174551299592, -1.7439038753509521, -1.5793386898436488, 1.5755459070205688, 0.26713186502456665])


# Columna 1 pieza verde [-1.4413602987872522, -1.68092741588735, -1.7416102886199951, -1.3439886581948777, 1.6059073209762573, 0.22908173501491547] dentro del circulo
# Columna 1 pieza verde [-1.4413602987872522, -1.68092741588735, -1.7416102886199951, -1.3439886581948777, 1.6059073209762573, 0.22908173501491547] encima

# Point auroco [0.01643398404121399, -1.3876174551299592, -1.7439038753509521, -1.5793386898436488, 1.5755459070205688, 0.26713186502456665]
# Point auroco 2 [0.011877944692969322, -1.4035440546325226, -1.9582005739212036, -1.3497496408275147, 1.5766655206680298, 0.2636486291885376]


# HANOI
# COL 1 [-1.5410898367511194, -1.821970125237936, -1.1506967544555664, -1.7381869755186976, 1.5752625465393066, -0.40384132066835576]
# COL 2 [-1.3415525595294397, -1.7472197018065394, -1.243865966796875, -1.7197805843748988, 1.5753071308135986, -0.20414764085878545]
# COL 3 [-1.147170368825094, -1.7331439457335414, -1.2606544494628906, -1.717024942437643, 1.5752791166305542, -0.009780232106344044]
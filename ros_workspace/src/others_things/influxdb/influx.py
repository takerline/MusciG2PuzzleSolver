#!/usr/bin/env python3
import threading
from influxdb_client import InfluxDBClient, Point
from influxdb_client.client.write_api import SYNCHRONOUS
from datetime import datetime
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

# Configuración
url = "https://deusto-influxdb-001-v2qrmk5znqme3f.eu-west-1.timestream-influxdb.amazonaws.com:8086"  # URL del servidor InfluxDB
token = "KgD5Y20nFrV4pKgAm_YGTr01wZJsrheSoxtM5pQFrsWNWzynEJCJiIYY0Fq1U9oeQSu1E0s9GiIGQw0tjXLxzg=="  # Token de autenticación
org = "deusto"  # Organización (configurada en la interfaz web)
bucket = "Grupo_6"  # Bucket donde se insertarán los datos

# Conexión al cliente
client = InfluxDBClient(url=url, token=token, org=org)
write_api = client.write_api(write_options=SYNCHRONOUS)

# Variables compartidas
moving = False
joint_states_queue = []
traceability_code = 0

# Bloqueo para la cola de mensajes
queue_lock = threading.Lock()

def create_traceability_code():
    dt = datetime.now()
    linea = "006" # Número de grupo y de línea
    area = "01" # Número de área
    dia_juliano = dt.strftime('%j')  # Día juliano 
    hora = dt.strftime('%H')  # Horas (24h)
    minuto = dt.strftime('%M')  # Minutos
    segundo = dt.strftime('%S')  # Segundos
    return int(f"{linea}{area}{dia_juliano}{hora}{minuto}{segundo}")

def check_movement(msg):
    """
    Callback para actualizar el estado de movimiento.
    """
    global moving, traceability_code
    moving = msg.data
    traceability_code = create_traceability_code()

def joint_states_callback(msg):
    """
    Callback para procesar los datos del topic /joint_states y almacenarlos en una cola.
    """

    global traceability_code
    if moving:
        try:
            data_points = []
            for i, name in enumerate(msg.name):
                #dt = datetime.now()
                position = msg.position[i] if len(msg.position) > i else None
                velocity = msg.velocity[i] if len(msg.velocity) > i else None
                effort = msg.effort[i] if len(msg.effort) > i else None

                # Define un punto de datos
                point = (
                Point("joint_states")
                .tag("TraceabilityCode", traceability_code)
                .tag("joint_name", name)
                .field("rotationValues", position)
                #.field("RotationUnits", "rad")
                .field("velocityValues", velocity)
                #.field("VelocityUnits", "rad/s")
                .field("torqueValues", effort)
                #.field("TorqueUnits", "N·m")
                .time(int(datetime.now().timestamp() * 1e9))
                )
                data_points.append(point)
            # Agrega los puntos a la cola
            with queue_lock:
                joint_states_queue.extend(data_points)

            rospy.loginfo(f"Datos agregados a la cola: {len(data_points)} puntos")

        except Exception as e:
            rospy.logerr(f"Error al procesar los datos del topic /joint_states: {e}")

def read_topic_data():
    """
    Hilo para leer datos de los topics ROS.
    """
    rospy.Subscriber("/moving", Bool, check_movement)
    rospy.Subscriber("/joint_states", JointState, joint_states_callback)

    rospy.loginfo("Nodo ROS iniciado. Escuchando /moving y /joint_states.")
    rospy.spin()

def write_to_influxdb():
    """
    Hilo para escribir datos en InfluxDB.
    """
    while not rospy.is_shutdown():
        with queue_lock:
            if joint_states_queue:
                try:
                    # Escribe todos los datos de la cola en InfluxDB
                    write_api.write(bucket=bucket, org=org, record=joint_states_queue)
                    rospy.loginfo(f"{len(joint_states_queue)} puntos escritos en InfluxDB.")
                    joint_states_queue.clear()
                except Exception as e:
                    rospy.logerr(f"Error al escribir en InfluxDB: {e}")
        rospy.sleep(0.1)  # Evita el uso intensivo de la CPU

if __name__ == "__main__":
    try:
        rospy.init_node("joint_states_to_influx", anonymous=True)
        # Crea e inicia los hilos
        ros_thread = threading.Thread(target=read_topic_data)
        influx_thread = threading.Thread(target=write_to_influxdb)

        ros_thread.start()
        influx_thread.start()

        ros_thread.join()
        influx_thread.join()

    except rospy.ROSInterruptException:
        pass
    finally:
        client.close()
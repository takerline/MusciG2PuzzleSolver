import yaml
from math import pi

# Crear y guardar conf inicial.
conf_inicial = {
    "configuracion_inicial": [
        -0 * pi / 180,
        -90 * pi / 180,
        -90 * pi / 180,
        -90 * pi / 180,
        90 * pi / 180,
        0 * pi / 180
    ]
}

with open("/home/laboratorio/ros_workspace/src/proyecto/src/proyecto/confs_y_poses/initial.yaml", "w") as f:
    yaml.dump(conf_inicial, f, default_flow_style=False)

# Crear y guardar pose inicial.
'''
pose_inicial = {
    "pose_inicial": {
        "position": {
            "x": -0.164,
            "y": -0.480,
            "z": 0.3
        },
        "orientation": {
            "x": 0,
            "y": 0,
            "z": 0,
            "w": 1.0
        }
    }
}

with open("src/proyecto/src/proyecto/confs_y_poses/poses.yaml", "w") as f:
    yaml.dump(pose_inicial, f, default_flow_style=False)
    '''

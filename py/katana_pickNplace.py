# Importamos las librerias necesarias
import numpy as np
from queue import PriorityQueue
import pybullet as p
import pybullet_data
import time
# from pickNplace_lib import * # Descomentar si se quiere usar la libreria de funciones fuera del Jupiter Notebook

# Conectar al cliente de simulación
physicsClient = p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
useFixedBase = True  
p.setGravity(0, 0, -9.81)

# Cargar los URDFs
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("modelos/manipuladores/katana/katana.urdf", basePosition=[0, 0, 0], useFixedBase=useFixedBase)
football_pitch = p.loadURDF("modelos/manipuladores/katana/football_pitch.urdf", basePosition=[1, 1.08, 0], useFixedBase=True)
duck2 = p.loadURDF("modelos/manipuladores/katana/duck_orange.urdf", basePosition=[0.28, -0.2, 0],useFixedBase=True)
duck3 = p.loadURDF("modelos/manipuladores/katana/duck_vhacd.urdf", basePosition=[0.4, 0, 0],useFixedBase=True)
duck4 = p.loadURDF("modelos/manipuladores/katana/duck_orange.urdf", basePosition=[0.25, 0.18, 0],useFixedBase=True)
basket = p.loadURDF("modelos/manipuladores/katana/basket.urdf", basePosition=[0.1, -0.3, 0.15],useFixedBase=True)

# Rotar la canasta
angle= -0.785398
orientation= p.getQuaternionFromEuler([angle, 0, 0])
p.resetBasePositionAndOrientation(basket, [0.1, -0.3, 0.15], orientation)

# Obtener el número de articulaciones y filtrar las móviles
num_joints = p.getNumJoints(robotId) 
movable_joints = []
gripper_joints = []

# Iterar sobre cada articulación
for joint_index in range(num_joints):
    joint_info = p.getJointInfo(robotId, joint_index)
    joint_type = joint_info[2]
    joint_name = joint_info[1].decode('utf-8')
    
    # Filtrar articulaciones móviles (REVOLUTE o PRISMATIC)
    if joint_type in [p.JOINT_REVOLUTE, p.JOINT_PRISMATIC]:
        movable_joints.append(joint_index)
    
    # Filtrar articulaciones del gripper (por nombre o tipo específico)
    if 'gripper' in joint_name.lower():
        gripper_joints.append(joint_index)

# Imprimir resultados
print("Número total de articulaciones:", num_joints)
print("Articulaciones móviles:", movable_joints)
print("Articulaciones del gripper:", gripper_joints)

# Definir la posición de la cámara
lista_de_obstaculos = [duck2, duck3, duck4]
basket_obstacle = [basket]  
grid_size = 0.06 #en terminos de la grilla
world_size = 0.35 #en terminos de la grilla
obstacles_height = 0 #en terminos del mundo
tool_frame_link = 8

print("Lista de obstáculos:", lista_de_obstaculos)

# Obtener los índices de los links de los dedos del gripper
grid, obstaculos= grid_and_obstacles(grid_size, world_size, obstacles_height, lista_de_obstaculos)

print("obstaculos",obstaculos)

""""
Definir la función de movimiento que recibe las posiciones a las 
que debe moverse el robot, entre otros inputs, como robotId o endEffectorId
"""
# Definir la función de movimiento
def mover_robot(robotId, endEffectorId, path, tool_frame_link, grip_coordinates, movable_joints, showProgress=False):
    """
    Mueve el robot a lo largo de la trayectoria calculada por A*.
    params
    :param robotId: ID del robot en la simulación.
    :param endEffectorId: ID del end effector del robot.
    :param path: Lista de posiciones a las que debe moverse el robot.
    :param tool_frame_link: ID del link del end effector.
    :param grip_coordinates: Coordenadas a las que debe moverse el gripper.
    :param movable_joints: Lista de índices de las articulaciones móviles del robot.
    return
    :return: None
    """
    # Definir los límites de las articulaciones
    global lower_l, upper_l, joint_ranges, pos_descanso
    
    for punto in path:
        # Calcular la cinemática inversa para obtener las posiciones de las articulaciones
        joint_positions = p.calculateInverseKinematics(bodyIndex=robotId, endEffectorLinkIndex=endEffectorId, targetPosition=punto, lowerLimits=lower_l, upperLimits=upper_l, jointRanges=joint_ranges, restPoses=pos_descanso)
        
        # Igualamos las posiciones de las articulaciones a las posiciones calculadas
        p.setJointMotorControlArray(bodyIndex=robotId, jointIndices=movable_joints, controlMode=p.POSITION_CONTROL, targetPositions=joint_positions)

        # Mover el gripper a las coordenadas definidas
        grip_joint_positions = p.calculateInverseKinematics(bodyIndex=robotId, endEffectorLinkIndex=tool_frame_link, targetPosition=grip_coordinates)
        
        # Igualar las posiciones de las articulaciones del gripper a las posiciones calculadas
        p.setJointMotorControlArray(bodyIndex=robotId, jointIndices=movable_joints, controlMode=p.POSITION_CONTROL, targetPositions=grip_joint_positions)

        # Avanzar la simulación un paso
        progress = (path.index(punto) + 1) / len(path) * 100
        if showProgress: print(f"Progreso: {progress:.2f}%")

    if showProgress: print("El robot ha completado el movimiento.")
    
# Definir el punto de inicio y el objetivo
start_mundo = [0.4, -0.6, obstacles_height] # Se usan las coordenadas del mundo en 3D, elegir las coordenadas de inicio y goal
goal_mundo = [0.35, 0.36, obstacles_height]
grip_coordinates= [0.19, -0.31, 0.2] 
start=world_to_grid(start_mundo, grid_size, obstacles_height) # Esta función se puede modificar en picknplace_lib.py
goal=world_to_grid(goal_mundo, grid_size, obstacles_height)

print('start:',start)
print('start:',is_in_obstacle(start, obstacles_height, grid_size, lista_de_obstaculos))
print('goal:',goal)
print('goal:',is_in_obstacle(goal, obstacles_height, grid_size, lista_de_obstaculos))

# Calcular el camino con A*
"""
path es el camino que recorre en la cancha evitando obstáculos.
"""
path = a_star_with_obstacles(start, goal, grid, obstaculos, lista_de_obstaculos, grid_size, obstacles_height)

"""
Se definen los límites de las articulaciones y los rangos de las articulaciones.
Esta info se encuentra en el archivo katana.urdf
"""
lower_l=[-3.025528, -0.135228, -2.221804, -2.033309, -2.993240]
upper_l=[2.891097, 2.168572, 2.054223, 1.876133, 2.870985]
joint_ranges=[5.916625, 2.303800, 4.276027, 3.909442, 5.864225]
pos_descanso=[0.5, 0.9, -0.5, -1.7]

# Iniciar la simulación
# time.sleep(5) # Esperar 5 segundos antes de empezar a mover el robot, descomentar si es necesario
path_index = 0
max_steps = 1000
max_Steps = max_steps
showProgressWhile = True

# Mover el robot a lo largo de la trayectoria
while max_steps > 0:
    mover_robot(robotId, tool_frame_link, path, tool_frame_link, grip_coordinates, movable_joints, showProgress=False)
    p.stepSimulation()
    # time.sleep(1 / 240) # se puede ajustar la velocidad de la simulación, comentar para máxima velocidad
    
    # Progreso de la simulación
    progress = (max_Steps - max_steps) / max_Steps * 100
    if showProgressWhile: print(f"Progreso: {progress:.2f}%")
    
    # Incrementar el índice del camino
    max_steps -= 1
    
print(f"Progreso: 100.00%")
print("El robot ha completado el movimiento.")
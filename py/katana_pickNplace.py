import pybullet as p
import pybullet_data
import time
import numpy as np
from py.pickNplace_lib import *


# Conectar al cliente de simulación
physicsClient = p.connect(p.GUI)
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

#TODO: Obtener el número de articulaciones y filtrar las móviles
num_joints = p.getNumJoints(robotId) 
movable_joints = []
gripper_joints = []

#filtrar las articulaciones móviles 
for i in range(num_joints):
    joint_info = p.getJointInfo(robotId, i)
    
    # Filtrar articulaciones móviles (revolute o prismatic)
    if joint_info[2] == p.JOINT_REVOLUTE or joint_info[2] == p.JOINT_PRISMATIC:
        movable_joints.append(i)
    
    # Identificar los joints del gripper
    if "finger" in str(joint_info[12]).lower():  # Si el nombre del joint contiene "finger"
        gripper_joints.append(i)

print("Articulaciones móviles:", movable_joints)
print("Articulaciones del gripper:", gripper_joints)


lista_de_obstaculos = [duck2, duck3, duck4] 
basket_obstacle = [basket]  
grid_size = 0.06 #en terminos de la grilla
world_size = 0.35 #en terminos de la grilla
obstacles_height = 0 #en terminos del mundo
tool_frame_link = 8

print(lista_de_obstaculos)

# Obtener los índices de los links de los dedos del gripper
grid, obstaculos= grid_and_obstacles(grid_size, world_size, obstacles_height, lista_de_obstaculos)


print("obstaculos",obstaculos)


""""definir la función de movimiento que recibe las posiciones a las 
que debe moverse el robot, entre otros inputs, como robotId o endEffectorId"""


#mod 
def mover_robot(robotId, endEffectorId, path, tool_frame_link, grip_coordinates, movable_joints):
    """Mueve el robot a lo largo de la trayectoria calculada por A*."""
    for punto in path:
        # Calcular la cinemática inversa para obtener las posiciones de las articulaciones
        joint_positions = p.calculateInverseKinematics(robotId, endEffectorId, punto, lowerLimits=lower_l, upperLimits=upper_l, jointRanges=joint_ranges, restPoses=pos_descanso)

        # Mover las articulaciones del brazo a las posiciones calculadas
        p.setJointMotorControlArray(robotId, movable_joints, p.POSITION_CONTROL, targetPositions=joint_positions)

        # Mover el gripper a las coordenadas definidas
        grip_joint_positions = p.calculateInverseKinematics(robotId, tool_frame_link, grip_coordinates)
        p.setJointMotorControlArray(robotId, gripper_joints, p.POSITION_CONTROL, targetPositions=grip_joint_positions)

        # # Avanzar la simulación un paso
        # p.stepSimulation()
        # time.sleep(1 / 240)  # Ajustar la velocidad de la simulación

    print("El robot ha completado el movimiento.")


start_mundo = [0.4, -0.6, obstacles_height] #se usan las coordenadas del mundo en 3D, elegir las coordenadas de inicio y goal
goal_mundo = [0.35, 0.36, obstacles_height]
grip_coordinates= [0.19, -0.31, 0.2] 
start=world_to_grid(start_mundo, grid_size, obstacles_height) #esta función se puede modificar en picknplace_lib.py
goal=world_to_grid(goal_mundo, grid_size, obstacles_height)

# print('start:',start)
# print('start:',is_in_obstacle(start, obstacles_height, grid_size, obstaculos))
# print('goal:',goal)
# print('goal:',is_in_obstacle(goal, obstacles_height, grid_size, obstaculos))
# """path es el camino que recorre en la cancha evitando obstáculos"""

path = a_star_with_obstacles(start, goal, grid, obstaculos, grid_size, obstacles_height)

""" se definen los límites de las articulaciones y los rangos de las articulaciones
Esta info se encuentra en el archivo katana.urdf"""
lower_l=[-3.025528, -0.135228, -2.221804, -2.033309, -2.993240]
upper_l=[2.891097, 2.168572, 2.054223, 1.876133, 2.870985]
joint_ranges=[5.916625, 2.303800, 4.276027, 3.909442, 5.864225]
pos_descanso=[0.5, 0.9, -0.5, -1.7]


time.sleep(5) # esperar 5 segundos antes de empezar a mover el robot
path_index = 0




while True:
    #TODO: implementar el movimiento del robot en la simulacion
    mover_robot(robotId, tool_frame_link, path, tool_frame_link, grip_coordinates, movable_joints)
    p.stepSimulation()
    time.sleep(1 / 240) # se puede ajustar la velocidad de la simulación
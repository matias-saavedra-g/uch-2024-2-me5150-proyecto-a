# Importamos las librerias necesarias
import numpy as np
from queue import PriorityQueue
import pybullet as p
import pybullet_data
import time
# from pickNplace_lib import * # Descomentar si se quiere usar la libreria de funciones fuera del Jupiter Notebook

# Esta función verifica si un nodo está dentro de un obstáculo
def is_in_obstacle(node, obstacle_height, grid_size, lista_de_obstaculos, sub_grid_divisions=10):
    """
    Determina si un nodo está dentro de un obstáculo.
    param
    :param node: Coordenadas del nodo a verificar.
    :param obstacle_height: Altura del obstáculo.
    :param grid_size: Tamaño de la celda de la grilla.
    :param lista_de_obstaculos: Lista de IDs de los obstáculos en el entorno.
    :param sub_grid_divisions: Número de subdivisiones del nodo para verificar si está dentro de un obstáculo. Por defecto es 10.                                 
    return
    :return: True si el nodo está dentro de un obstáculo, False en caso contrario.
    """
    sub_grid_size = grid_size / sub_grid_divisions
    
    # Recorre la lista de obstáculos
    for obstacle_id in lista_de_obstaculos:
        # Obtener el AABB (Axis-Aligned Bounding Box) del obstáculo
        aabb_min, aabb_max = p.getAABB(obstacle_id)
        
        # Recorre las subdivisiones del nodo
        for i in range(sub_grid_divisions):
            for j in range(sub_grid_divisions):
                # Calcula las coordenadas del sub-nodo
                sub_node_x = node[0] + i * sub_grid_size
                sub_node_y = node[1] + j * sub_grid_size
                
                # Verifica si el sub-nodo está dentro del AABB del obstáculo
                if aabb_min[0] <= sub_node_x <= aabb_max[0] and aabb_min[1] <= sub_node_y <= aabb_max[1] and aabb_min[2] <= obstacle_height <= aabb_max[2]:
                    return True  # Si está dentro del obstáculo, retorna True
    
    return False  # Si no está dentro de ningún obstáculo, retorna False

# Esta función genera una grilla y una lista de obstáculos en el mundo
def grid_and_obstacles(grid_size, world_size, obstacles_height, lista_de_obstaculos):
    """
    Genera una grilla y una lista de obstáculos en el mundo.
    Esta función ya está implementada y no es necesario modificarla.
    param
    :param grid_size: Tamaño de cada celda de la grilla.
    :param world_size: Tamaño del mundo.
    :param obstacles_height: Altura de los obstáculos a considerar.
    :param lista_de_obstaculos: Lista de IDs de los obstáculos en el entorno.
    return
    :return: Una tupla con la grilla y la lista de obstáclos en el mundo.
    """
    grid = []
    obstacles= []
    for x in np.arange(0, world_size*2, grid_size):
        for y in np.arange(-world_size, world_size, grid_size):
            if not is_in_obstacle([x, y], obstacles_height, grid_size, lista_de_obstaculos):
                grid.append((x,y))
            else:
                obstacles.append((x,y, obstacles_height))
    return grid, obstacles

# Esta función reconstruye el camino desde el nodo inicial hasta el nodo actual
def reconstruct_path(came_from, current):
    """
    Reconstruye el camino desde el nodo inicial hasta el nodo actual.
    Esta función ya está implementada y no es necesario modificarla.
    param
    :param came_from: Diccionario que contiene los nodos visitados.
    :param current: Nodo actual.
    return
    :return: Lista con el camino reconstruido.
    """
    path = []
    while current in came_from:
        path.append(current)
        current = came_from[current]
    path.reverse() #invertir el camino para que vaya desde el inicio hasta el final
    return path

# Esta función obtiene los vecinos de un nodo en la grilla
def get_neighbors(node, grid, obstacles, grid_size, height_z):
    """"
    Obtiene los vecinos de un nodo en la grilla.
    Esta función ya está implementada y no es necesario modificarla.
    param
    :param node: Nodo actual.
    :param grid: Grilla que representa el entorno.
    :param obstacles: Lista de IDs de los obstáculos en el entorno.
    :param grid_size: Tamaño de cada celda de la grilla.
    :param height_z: Altura de los obstáculos a considerar.
    return
    :return: Lista con los vecinos del nodo actual.
    """
    directions = [(1, 0, 0), (0, 1, 0), (-1, 0, 0), (0, -1, 0)]  # Movimientos en 2D 
    scaled_directions = [(d[0] * grid_size, d[1] * grid_size, d[2] * grid_size) for d in directions]
    
    neighbors = []
    
    for direction in scaled_directions:
        neighbor = (node[0] + direction[0], node[1] + direction[1], height_z + direction[2])
        
        is_obstacle = False
        for obs in obstacles:
            if abs(neighbor[0] - obs[0]) < 0.09 and abs(neighbor[1] - obs[1]) < 0.09 and abs(neighbor[2] - obs[2]) < 0.09:#print("Obstáculo:", obs)
                is_obstacle = True
                break
        
        if not is_obstacle:
            for cell in grid:
                if abs(neighbor[0] - cell[0]) < 0.09 and abs(neighbor[1] - cell[1]) < 0.09 and abs(neighbor[2] - height_z) < 0.09:
                    neighbors.append(neighbor)
                    break

    return neighbors

# Esta función aplica el algoritmo de A* para encontrar el camino más corto entre dos puntos en un entorno con obstáculos
def a_star_with_obstacles(start, goal, grid, obstacles, grid_size, obstacles_height):
    """
    Aplica el algoritmo de A* para encontrar el camino más corto entre dos puntos en un entorno con obstáculos.
    param
    :param start: Punto de inicio (tuple con coordenadas x, y, z).
    :param goal: Punto de objetivo (tuple con coordenadas x, y, z).
    :param grid: Grilla que representa el entorno.
    :param obstacles: Lista de IDs de los obstáculos en el entorno.
    :param grid_size: Tamaño de cada celda de la grilla.
    :param obstacles_height: Altura de los obstáculos a considerar.
    return
    :return: Lista con el camino más corto entre el punto de inicio y el objetivo.
    """
    
    def heuristic(node, goal):
        """
        Calcula la distancia heurística (Euclidiana) entre dos puntos.
        param
        :param node: Nodo actual.
        :param goal: Nodo objetivo.
        return
        :return: Distancia heurística entre los dos nodos.
        """
        return np.linalg.norm(np.array(node) - np.array(goal))
    
    # Cola de prioridad y conjuntos de puntuaciones
    set_abierto = PriorityQueue()
    set_abierto.put((0, start))  # Iniciamos con el nodo de inicio
    came_from = {}  # Rastrea de dónde viene cada nodo
    g_score = {start: 0}  # Costo real desde el inicio
    f_score = {start: heuristic(start, goal)}  # Estimación del costo total (g + h)

    # Bucle principal
    while not set_abierto.empty():
        current = set_abierto.get()[1]  # Obtener el nodo con el menor costo estimado
        
        # Si el nodo actual es el objetivo, reconstruir el camino
        if abs(current[0] - goal[0]) < 0.09 and abs(current[1] - goal[1]) < 0.09 and abs(current[2] - goal[2]) < 0.09:
            return reconstruct_path(came_from, current)
        
        # Obtener vecinos del nodo actual
        print("Current:", current)
        neighbors = get_neighbors(current, grid_size, obstacles, grid, obstacles_height)
        
        for neighbor in neighbors:
            # Si el vecino está en un obstáculo, lo ignoramos
            if is_in_obstacle(neighbor, obstacles_height, grid_size, obstacles):
                continue
            
            # Costo tentativo desde el nodo actual hasta el vecino
            tentative_g_score = g_score[current] + np.linalg.norm(np.array(current) - np.array(neighbor))
            
            if tentative_g_score < g_score(neighbor,np.inf):
                # Este es un mejor camino, se actualizan los valores
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, goal)
                set_abierto.put((neighbor,f_score[neighbor]))
    
    # Si no se encuentra el camino
    raise ValueError("No se encontró un camino")

# Esta función convierte un punto del mundo de la simulación a coordenadas de la grilla
def world_to_grid(world_point, grid_size, obstacles_height):
    """
    Convierte un punto del mundo de la simulación a coordenadas de la grilla.
    De ser necesario, se puede modificar esta función, para obtener mejores resultados en la simulacion
    param
    :param world_point: Punto en el mundo de la simulación.
    :param grid_size: Tamaño de cada celda de la grilla.
    :param obstacles_height: Altura de los obstáculos a considerar.
    return
    :return: Punto en la grilla correspondiente al punto del mundo de la simulación.
    """
    x_world, y_world, z_world = world_point
    x_grid = x_world  / 0.1  
    x_grid = round(x_grid) * grid_size  
    
    y_grid = y_world / 0.1   
    y_grid = round(y_grid) * grid_size  
    
    z_grid = z_world
    
    return (x_grid, y_grid, z_grid)
    """
    convierte un punto del mundo de la simulación a coordenadas de la grilla.
    De ser necesario, se puede modificar esta función, para obtener mejores resultados en la simulacion
    """
    x_world, y_world, z_world = world_point
    x_grid = x_world  / 0.1  
    x_grid = round(x_grid) * grid_size  
    
    y_grid = y_world / 0.1   
    y_grid = round(y_grid) * grid_size  
    
    z_grid = z_world
    
    return (x_grid, y_grid, z_grid)

# Si se ejecuta este script, se ejecutará el siguiente código
if __name__ == "__main__":
    pass
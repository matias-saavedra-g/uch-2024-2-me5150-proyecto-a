# Universidad de Chile
## Facultad de Ciencias Físicas y Matemáticas
### Departamento de Ingeniería Mecánica

**ME 5150-1 Robótica**

## Proyecto A - Simulación de Pick & Place en Pybullet

**Profesor:** Harold Valenzuela Coloma  
**Auxiliar:** Leslie Cárdenas  
**Ayudantes:** Francisco Cáceres, Karen Quezada, Benjamín Rojas

### Brazo robótico Katana

El manipulador robótico Katana cuenta con una rotación en la base, tres rotaciones adicionales a lo largo del brazo, una rotación para orientar el gripper y, por último, un gripper de dos dedos (derecho e izquierdo).

**Link al repo original:** [aquí](https://github.com/uos/katana_driver/tree/kinetic).

El objetivo del proyecto es simular un manipulador robótico realizando una rutina de Pick & Place en una cancha de fútbol. La simulación debe mostrar cómo el manipulador recoge una pelota de una canasta elevada y la coloca en el arco contrario, evitando varios obstáculos en el recorrido. Para lograr esto, se empleará un algoritmo de path planning.

### Path planning: algoritmo A\*

Es un tipo de algoritmo de búsqueda en grafos de tipo heurístico. Encuentra el camino de menor costo entre un nodo de origen y uno objetivo, eligiendo en cada paso el nodo que minimiza la función de costo total estimado. La fórmula de este algoritmo es:

\[ f(n) = g(n) + h(n) \]

En donde \( f(n) \) es el costo total estimado, \( g(n) \) es el costo real desde el inicio hasta el nodo \( n \), y \( h(n) \) es la estimación heurística del costo desde el nodo \( n \) hasta el objetivo. En este último, se puede calcular el costo aproximado usando métodos matemáticos como la distancia Euclidiana, la distancia de Manhattan, distancia de Chebyshev, entre otras. A\* utiliza las colas de prioridad para seleccionar el nodo con el menor valor de \( f(n) \), para expandirlo en cada paso.

### Grilla creada para resolver el algoritmo

Para resolver el algoritmo A\* se creó una grilla en el mundo de la simulación. Es invisible (eventualmente podrían dibujarla usando `addUserDebugLine` y modificarla, visualizándola con esta herramienta). En la imagen se observa cómo es la grilla que se creó. Actualmente no está visible.

### Códigos a modificar

#### Código `picknplace_lib.py`

En la carpeta `tareas` se encuentra este código el cual debe ser modificado para aplicar el algoritmo de búsqueda, específicamente en la función `a_star_with_obstacles`. En este código también debe completarse la función `is_in_obstacle`, la que debe determinar si en un nodo de la grilla se encuentra un objeto en la simulación (un pato). Pueden haber varios métodos para esto, pero se recomienda usar funciones propias de Pybullet.

#### Código `katana_pick_and_place.py`

Este es el código principal de la simulación, en donde se han importado las funciones del código anterior, mediante la línea `from picknplace_lib import *`. Crear una función que mueva el robot desde un punto de inicio hasta un punto final (en este caso desde la canasta hasta el arco del lado contrario). Para lograrlo, es recomendable revisar la documentación de PyBullet, ya que proporciona funciones útiles como `calculateInverseKinematics2`, `setJointMotorControlArray` o `resetJointState` (cinemática directa) las cuales permiten controlar el movimiento y la posición del brazo robótico. Además, hay funciones como `getJointState` y `getLinkState` que proporcionan información sobre los estados de las articulaciones y los eslabones del brazo, lo que puede ser de gran utilidad para ajustar y monitorear su movimiento. La función debe recibir como uno de sus parámetros el `endEffectorId` (en este caso, definido como `tool_frame_link`), que es un enlace imaginario en el centro del gripper, utilizado como referencia para la cinemática inversa.

El movimiento debe considerar que el robot se desplace hacia la canasta, oriente el gripper (ajustando su rotación en cualquier ángulo) y cierre el gripper (utilizando las funciones de PyBullet). Posteriormente, el robot debe trasladarse a lo largo de la cancha, evitando los obstáculos y abrir el gripper al llegar al punto objetivo. Para que el brazo se introduzca en la canasta, se pueden emplear varios enfoques: usar una función predefinida sin recurrir al algoritmo A\*, crear varios puntos intermedios para guiar el movimiento, o eventualmente utilizar A\* si se estima conveniente. No es necesario que tome ningún objeto, basta con implementar la rutina de movimiento.

### Indicaciones generales

- En algunas funciones que deben completar hay líneas que dicen `raise NotImplementedError`, una vez que hagan sus funciones deben borrarlo, para que no tire error.
- Se recomienda fuertemente que printeen las variables o resultados de sus funciones para entender mejor lo que está pasando y puedan debuggear sus errores.
- El movimiento del robot debe ser lo más fluido posible, para ello pueden usar control de velocidad o posición, disponibles en la documentación.

### Entregable

Presentación en video explicando la metodología usada. Deben mostrar el resultado (la simulación), el análisis de lo que sucede (si se mueve bien o mal, por qué) y finalmente conclusiones de la experiencia.
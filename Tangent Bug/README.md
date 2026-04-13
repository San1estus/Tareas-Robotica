# Reporte de funcionamiento del código

## Simulador
Para realizar las simulaciones se uso OpenGL Legacy usando GLFW, para no meterme mucho en configuración de shaders y renderers.

## Estructuras definidas
Se utilizan 3 estructuras principales:

- Vec2: Almacena las coordenadas $(x,y)$ de los vertices de los poligonos en coordenadas del mundo/pantalla.
- Polygon: Almacena todos los puntos usados para crear el poligono al dar click.
- Robot: Contiene toda la lógica del algoritmo Tangent Bug, su posición actual, la trayectoria recorrida, estado del robot, detección de ciclos, manejo de mínimos locales

## Geometría usada
Además de la librería de cmath, en Vec2 se agregaron las operaciones básicas de suma, resta y multiplicación por escalar, además el producto punto, su norma y poder normalizar el vector.

Como el poligono se almacena como punto en el espacio, podemos determinar los segmentos que lo conforman considerando los vectores del vértice $i$ al vértice $i+1$, con esto cuando detectamos un segmento, podemos seguir dos direcciones tangentes, en base a cual de estas direcciones nos acerca más a la meta consideramos la dirección de seguimiento. Al seguir almacenamos la distancia mínima alcanzada, si en algún momento ocurre que la distancia aumenta de nuevo, hay que abandonar el seguimiento del obstáculo e intentar avanzar nuevamente. Lo mismo se repite con todos los polígonos que se alcancen.


## Algoritmo
El comportamiento del robot se basa en dos modos principales:

1. Movimiento hacia la meta

El robot avanza directamente hacia la meta si existe línea de vista libre o no se ha alcanzado un obstaculo.

2. Seguimiento de obstáculos

Cuando el robot encuentra un obstáculo calcula la normal al obstáculo más cercano, se desplaza tangencialmente al borde, mantiene una distancia constante (para evitar que se encierre). Mientras tanto, registramos los segmentos visitados del polígono, si en algún punto volvemos un segmento ya visitado, podemos concluir que se hizo (o hará) un ciclo, por lo que se detiene la simulación.

## Uso del simulador

El simulador tiene 2 modos, uno para dibujar y otro para correr. Dando click en la pantalla se añade un vértice, se pueden añadir los vértices que quieran, dando C se forma un polígono conectando el último vértice y el primero y se pueden seguir dibujando. Al terminar de dibujar los poligonos, presionar Enter para correr la simulación. Con R se puede reiniciar el entorno para intentar algo nuevo.

## Video demostración
En el primero se muestra un caso simple, un caso con múltiples obstáculos, un caso con un obstáculo concavo y un caso dónde no hay solución, en este caso el robot se detiene.



https://github.com/user-attachments/assets/2e8b5277-8299-423d-9a09-ba882069d47e


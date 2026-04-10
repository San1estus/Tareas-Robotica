# Reporte de funcionamiento del código

## Simulador
Para realizar las simulaciones se uso OpenGL Legacy usando GLFW, para no meterme mucho en configuración de shaders y renderers.

## Estructuras definidas
Se utilizan 3 estructuras principales:

- Point: Esta almacena las coordenadas $(x,y)$ de los vertices de los poligonos en coordenadas del mundo/pantalla.
- Segment: Se le asignan dos puntos, uno de inicio y uno de final, para indicar que es el segmento que une estos puntos.
- AngularPoint: Similar a point, pero ademas almacena el angulo respecto al observador (el punto de donde se calcula el polígono de visibilidad), y la distancia a este punto respecto al observador, el angulo sirve a la hora de calcular el polígono. Se usa el ángulo en sentido antihorario.

## Geometría usada
Además de la librería de cmath, se agregaron las funciones
crossProduct, que devuelve del producto cruz de dos vectores $\mathbf{x,y}\in \R^2$, es decir $$crossProduct(\mathbf{x,y})=x_1y_2-x_2y_1.$$

y getRaySegmentIntersection que identifica si hay una colisión con algún segmento y el punto donde colisiona, se asume rango infinito para la visibilidad. 

Para hacer esto tomamos el vector $\mathbf{s}$ que sale del punto inicial del segmento $\mathbf{a}$ al punto final del segmento $\mathbf{b}$ y además el vector $\mathbf{r}$ que va desde el observador $\mathbf{o}$ hacía el inicio del segmento. Podemos parametrizar estos vectores como
$$\mathbf{r}(t)=\mathbf{o}+t\cdot (\mathbf{a-o})=\mathbf{o}+t\cdot \mathbf{c}\quad\text{y}\quad \mathbf{s}(u) = \mathbf{a}+u\cdot (\mathbf{b-a}) = \mathbf{a}+u\cdot\mathbf{d},$$
donde $t,u$ son parametros que nos indican en que punto estan en el tiempo $t$ y $u$ respectivamente. Para encontrar el punto donde esto pasa, nos interesa resolver la ecuación
$$\mathbf{o}+t\cdot \mathbf{c}=\mathbf{a}+u\cdot\mathbf{d}\Leftrightarrow t\cdot\mathbf{c}-u\cdot\mathbf{d}=\mathbf{a-o},$$
pero antes podemos realizar el producto cruz entre $\mathbf{c}$ y $\mathbf{d}$, si este producto es cercano a $0$, entonces los vectores son paralelos. Si no es el caso, podemos despejar $t$ haciendo producto cruz con $\mathbf{d}$ en ambas ecuaciones recordando que el producto  de un vector consigo mismo es siempre $0$ y que este es bilineal, se obtiene 
$$(t\cdot\mathbf{c}-u\cdot\mathbf{d})\times\mathbf{d}=(\mathbf{a-o})\times\mathbf{d}=t\cdot \mathbf{c}\times\mathbf{d}=(\mathbf{a-o})\times d,$$
de manera analóga se obtiene $u$ pero haciendo producto cruz con $\mathbf{c}$. Si esto tiene solución, resta ver que el punto de impacto este en el segmento y ocurre enfrente del observador, para esto notamos que $\mathbf{s}(0)=a$ y $\mathbf{s}(1)=b$, por lo tanto se debe satisfacer que
$$0\leq u\leq 1\quad\text{y}\quad t>0,$$
si es el caso, regresa el tiempo $t$ que se encontro que nos indica en que parte del segmento se da el impacto.



## Algoritmo
Para calcular el polígono de visibilidad, tomamos todos los segmentos que hay en el mundo, y calculamos si hay un punto de impacto desde el observador hacía cada uno de ellos, para prevenir que se sigan al infinito, se añade una bounding box muy lejos en la escena para que al menos detecte esas colisiones. Además, como puede haber errores de punto flotante, se consideran 3 rayos, el rayo que va en la dirección a alguno de los vértices del segemento y uno con una ligera perturbación negativa y otro con perturbación positiva. De estos 3 rayos, consideramos el que nos dé el $t$ más pequeño como el punto de impacto, si todos se van a infinito no hubo ningún contacto. Por último, para obtener el polígono, ordenamos los puntos de colisión obtenidos respecto al ángulo en sentido antihorario y consideramos el poligono que se forma en esa dirección de los puntos. 

## Uso del simulador

El simulador tiene 2 modos, uno para dibujar y otro para "iluminar", dando click en la pantalla se añade un vértices, se pueden añadir los vértices que quieran, dando Enter se forma un polígono conectando el último vértice y el primero y se pueden seguir dibujando. Entre tanto se puede presionar el Espacio para cambiar de modo, cuando "ilumina" se muestra el polígono de visibilidad. Con $R$ se reinicia el entorno borrando todos los polígonos.


# Proyecto_final_robotica_y_sistemas_autonomos
En este repositorio se encontrará la documentación técnica asociada al proyecto final en un README.md, se encontrará un acceso a un video que evidencia el correcto funcionamiento del robot en Webots con la navegación seleccionada, además se encuentran los directorios del controlador del robot, el archivo de mundos de Webots junto con sus intrucciones de instalación.

## Integrantes :

Marcel Gutierrez

Mario Rojas

Martín Basulto

Benjamín Leiva

Joaquín Rojas



## Linea seleccionada : Planificación de rutas con A*

## Objetivos 

**Objetivo del proyecto :** Aprender a diseñar, implementar y evaluar en Webots un sistema de navegación autónoma para un
robot móvil diferencial, integrando control cinemático, percepción sensorial, estimación de
movimiento y toma de decisiones mediante una solución basada en planificación de rutas
o en SLAM/mapeo autónomo.

**Objetivo del equipo :** Nuestro objetivo como equipo es poder trabajar en conjunto para lograr intergrar la planificación de rutas A* a nuestro robot diferencial en Webots que hemos estado trabajando a lo largo de los laboratorios, dividiéndonos roles en el equipo e integrando todos nuestros avances para lograr el objetivo del proyecto asertivamente.

## Robot, sensores y actuadores :

**Descripción general del robot :** Utilizamos un robot diferencial con ruedas extraído de e-puck.wbt.

**Sensores utilizados :** Los sensores que le implementamos fueron encoders para las ruedas y sensores de distancia infrarrojos (IR) para estimar distancias.

**Actuadores utilizados :** Motores rotacionales izquierdo (left wheel motor) y derecho (right wheel motor)

## Descripción de los escenarios de prueba

Para probar el funcionamiento eficiente de nuestro robot, planteamos dos escenarios principales :

**Escenario simple :** Pocos obstáculos, ruta relativamente directa y baja complejidad. 

Para este escenario utilizaremos un cajas como obstáculos denominadas "WoodenBox" que impedirán un trayecto completamente despejado entre la posición de inicio y la meta, sin embargo, habrá espacio suficnete entre los obstáculos para estimar una trayectoria válida con suficiente facilidad, ya que las cajas están distanciadas entre sí, siempre habrán rutas posibles, sin embargo el objetivo ideal no es solo encontrar una ruta sino la más óptima posible.

**Escenario complejo :** Mayor cantidad de obstáculos, pasillos estrechos, curvas, zonas de bloqueo o rutas alternativas. 

Para este escenario utilizaremos estructuras de concreto como obstáculos denominadas "ConcreteWall" y barreras metálicas denominadas "MetalFence". Estos elementos se distribuirán de forma densa por todo el entorno, creando un laberinto con pasadizos muy ajustados y esquinas pronunciadas que forzarán al algoritmo a realizar giros constantes. Además, se configurarán zonas de bloqueo total (callejones sin salida) y bifurcaciones con rutas alternativas donde un camino parecerá prometedor al principio pero terminará cerrado, obligando al algoritmo a retroceder. En este escenario, la baja visibilidad de la meta y la alta densidad de "ConcreteWall" y "MetalFence" elevarán drásticamente la complejidad, desafiando al algoritmo A* a no quedarse atrapado en mínimos locales y a demostrar su verdadera eficiencia al encontrar la única ruta real y óptima entre la multitud de opciones falsas


## Explicación del algoritmo implementado 

***Explicación y utilidad del algoritmo seleccionado :** Nuestro algoritmo, es un algoritmo de planificación de rutas que tiene cómo propósito guiar a nuestro robot diferencial en la simulación hasta una meta final dividiéndose principalmente en dos componentes : el costo acumulado del trayecto actual (g(n)) y la predicción del costo desde nuestra posición actual recorrida hasta la meta denominada heurística (h(n)), formando un componente principal que es la combinación de estos dos componentes secundarios el cuál es el costo total del trayecto desde el inicio hasta la meta (f(n)), y así formamos la fórmula completa del algoritmo A* a continuación :

***Fórmula del Algoritmo A* :**  $$f(n) = g(n) + h(n)$$

La ecuación principal del algoritmo A* es $f(n) = g(n) + h(n)$, donde:

- **$f(n)$**: Costo total estimado del camino a través del nodo $n$.
- **$g(n)$**: Costo real acumulado desde el nodo de inicio hasta el nodo $n$.
- **$h(n)$**: Costo estimado (heurística) desde el nodo $n$ hasta la meta.

**Resultado entregado por nuestro algoritmo :** Este algoritmo nos entrega como resultado un Path o ruta con las coordenadas por las cuales debe transitar el robot para llegar a la meta.

**Ejemplo :**

Para el siguente mapa 

0 = Libre, 1 = Obstáculo

cuadricula = [
    [0,0,0,0,1,0,0,0,0,0],
    [0,1,1,0,1,0,1,1,1,0],
    [0,0,1,0,0,0,0,0,1,0],
    [0,0,1,1,1,1,1,0,1,0],
    [0,0,0,0,0,0,1,0,0,0],
    [0,1,1,1,1,0,1,1,1,0],
    [0,0,0,0,1,0,0,0,1,0],
    [1,1,1,0,1,1,1,0,1,0],
    [0,0,0,0,0,0,0,0,0,0],
    [0,0,1,1,1,1,1,1,1,0]
]

Aplicamos la lógica del algoritmo A* y obtenemos un camino eficiente :

```
Path: [(0,0), (1,0), (2,0), (3,0), (3,1), (3,2), (4,2), (5,2), (6,2), (7,2), (7,3), (7,4), (8,4), (9,4), (9,5), (9,6), (9,7), (9,8), (9,9)]
```



## Pseudocódigo de la solución

``` 
# Definiciones y estructura de nodos

Definir constantes: ANCHO = 10, ALTO = 10
Definir MATRIZ_OBSTÁCULOS (Cuadrícula de 10x10 donde 0 es libre y 1 es bloqueo)

Estructura NODO:
    Atributos:
        x, y: Coordenadas en el mapa
        g: Costo acumulado desde el inicio (comienza en Infinito)
        f: Costo total estimado (comienza en Infinito)
        h: Costo estimado restante hasta la meta (comienza en 0)
        padre: Enlace al nodo anterior (comienza vacío)
        abierto: Estado booleano (Falso)
        cerrado: Estado booleano (Falso)

Crear MATRIZ_DE_NODOS de tamaño ANCHO x ALTO llena de instancias de NODO

# Funciones necesarias previas al algoritmo A*

Función CALCULAR_HEURÍSTICA_MANHATTAN(x1, y1, x2, y2)
    Retornar (Valor absoluto de x1 - x2) + (Valor absoluto de y1 - y2)
Fin Función

Función INICIALIZAR_NODOS()
    Por cada nodo dentro de MATRIZ_DE_NODOS:
        Reiniciar g y f a Infinito
        Vaciar el nodo padre
        Marcar abierto y cerrado como Falso
Fin Función

Función OBTENER_NODO_MENOR_F()
    Definir mejor_nodo como Vacío
    Por cada nodo dentro de MATRIZ_DE_NODOS:
        Si el nodo está en la lista de ABIERTOS:
            Si mejor_nodo está Vacío O el f del nodo actual es menor que el f de mejor_nodo:
                mejor_nodo = nodo actual
    Retornar mejor_nodo
Fin Función

#Lógica del algoritmo A*

Función ALGORITMO_A_ESTRELLA(inicio_x, inicio_y, meta_x, meta_y)
    Llamar a INICIALIZAR_NODOS()
    
    Configurar nodo_inicio:
        g = 0
        h = CALCULAR_HEURÍSTICA_MANHATTAN(inicio_x, inicio_y, meta_x, meta_y)
        f = g + h
        abierto = Verdadero
    
    Repetir indefinidamente:
        nodo_actual = OBTENER_NODO_MENOR_F()
        
        Si nodo_actual está Vacío:
            Mostrar mensaje: "Atrapado, no hay camino posible"
            Retornar Vacío
            
        Si nodo_actual está en las coordenadas (meta_x, meta_y):
            Retornar nodo_actual  // ¡Éxito! Llegamos a la meta
            
        Marcar nodo_actual como abierto = Falso
        Marcar nodo_actual como cerrado = Verdadero
        
        Por cada dirección vecina (Derecha, Izquierda, Abajo, Arriba):
            Calcular coordenadas del vecino (nx, ny)
            
            Si (nx, ny) está dentro de los límites del mapa Y MATRIZ_OBSTÁCULOS[ny][nx] es libre (0):
                vecino = MATRIZ_DE_NODOS[ny][nx]
                
                Si vecino ya está cerrado:
                    Saltar al siguiente vecino
                    
                Calcular nuevo_g = g de nodo_actual + 1
                
                Si vecino NO está abierto O nuevo_g es menor que el g que ya tenía el vecino:
                    vecino.g = nuevo_g
                    vecino.h = CALCULAR_HEURÍSTICA_MANHATTAN(nx, ny, meta_x, meta_y)
                    vecino.f = vecino.g + vecino.h
                    vecino.padre = nodo_actual
                    vecino.abierto = Verdadero
Fin Función

# Reconstruir el camino para una nueva re-implementación del algoritmo A*

Función RECONSTRUIR_CAMINO(nodo_meta)
    Crear lista vacía llamada camino
    nodo_actual = nodo_meta
    
    Mientras nodo_actual NO esté Vacío:
        Añadir las coordenadas (x, y) de nodo_actual a la lista camino
        nodo_actual = nodo_actual.padre
        
    Invertir el orden de la lista camino (para que vaya desde el inicio al fin)
    Retornar camino
Fin Función

```

## Relación explícita con los laboratorios 1 y 2

Hay una relación progresivos, los laboratorios nos ayudaron en gran medida a formar los cimientos del robot : 

**Laboratorio 1 :** En el laboratorio 1 inicializamos un robot diferencial simple y abordamos la cinemática del robot para que se pueda mover en un escenario de simulación a través de la física.

**Laboratorio 2 :** El laboratorio 2 profundizó en el funcionamiento del robot implementando prácticas adicionales como la fusión de sensores, el muestreo, el filtrado simple, el filtro de Kalman y el gráfico de valores crudos, estimados y filtrados, además de la navegación reactiva,  todo esto con la intención de que nuestro robot ya no solo se moviera en el escenario sino que pueda ser capaz de frenar a tiempo y evitar chocar con un obstáculo.

**Proyecto final :** por último en nuestro proyecto final hemos realizado el último ajuste necesario para que el robot pueda moverse con certeza en el mapa : la planificación de rutas, la cual otorga al robot una guía clara de los movimientos que necesita hacer para llegar a una meta específica esquivando inteligentemente los obstáculos para llegar lo antes posible.

**Conclusión :** Por lo tanto, nuestro proyecto final no inició en esta entrega, inicio desde el laboratorio 1 y la completitud de los objetivos fueron avanzando progresivamente hasta lograr nuestro cometido final, no es posible lograr este proyecto final sin seguir con la consistencia gradual de las anteriores 2 entregas de avances.












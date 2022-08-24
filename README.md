# BelkanGame
## Miguel García López
### Práctica - Inteligencia Artificial

- Descripción: Pequeño juego que implementa varios algoritmos básicos de Pathfinding, tales como Búsqueda en profundidad, Búsqueda en anchura etc. Dispone de varios niveles:
  - Lv 0 -> nivel demo, sin dificultad alguna con algoritmo de pathfinding básico de búsqueda en profundidad sin consideración alguna sobre objetos, obstáculos o demás consideraciones.
  - Lv 1 -> se requiere un plan óptimo en número de acciones. Para este nivel se utiliza el algoritmo de búsqueda en anchura.
  - Lv 2 -> se requiere un plan óptimo con respecto a la batería gastada. Para ello hay que tener en cuenta el costo de cada casilla y distintos objetos que recucen el mismo. Se utiliza un algoritmo de costo uniforme.
  - Lv 3 -> igual que el nivel 2, pero con el aliciente de encontrar tres objetivos en vez de uno. Se sigue utilizando costo uniforme.
  - Lv 4 -> encontrar máximo número de objetivos posibles en un mapa sin explorar (no tenemos información de las casillas hasta que las exploremos).

- Para ejecutarlo:
  - ./install.sh para instalar paquetes necesarios
  - make para compilar
  - ./Belkan para ejecutar el programa

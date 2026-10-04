# Holograma

Etapa 3 de la ampliación del lab2 de ALED (Recursividad: cinemática directa de un exoesqueleto). Un cuerpo humano real, creado con MakeHuman, dibujado como un holograma de decenas de miles de puntos que se mueve con un esqueleto de 21 segmentos y la misma cinemática directa recursiva del lab2. El código está comentado a fondo para que sirva también como material de estudio.

![holograma](docs/holograma.png)

| Caminando | Esqueleto dentro del holograma | Captura de movimiento real (.bvh) |
|---|---|---|
| ![caminando](docs/holograma-caminando.png) | ![esqueleto](docs/esqueleto.png) | ![mocap](docs/holograma-mocap.png) |

## Las tres etapas

| Etapa | Repo | Qué es |
|---|---|---|
| 1 | [ALED-lab2](https://github.com/Ricardo4843/ALED-lab2) | La práctica: cinemática directa recursiva en 2D (15 segmentos, Swing) |
| 2 | [exoesqueleto](https://github.com/Ricardo4843/exoesqueleto) | El muñeco: la misma cinemática en 3D con matrices 4x4, cuerpo de 21 segmentos con armadura de tubos y benchmark recursivo vs iterativo |
| 3 | **holograma** (este) | El cuerpo real de MakeHuman como nube de puntos, con el esqueleto colocado automáticamente dentro |

## Cómo ejecutarlo

- **Eclipse:** File > Import > General > Existing Projects into Workspace, y seleccionar esta carpeta. Run sobre `holograma.gui.HologramApp`. JavaFX 23 ya viene incluido en `lib/`. Hace falta un JDK 21 o superior.
- **Terminal (PowerShell):** `.\run.ps1` compila y lanza el visor con `JAVA_HOME` o, si no está definido, con el JDK que trae Eclipse.

Controles:
- Arrastrar el ratón para girar la cámara y la rueda para el zoom.
- En el panel se elige una articulación y se mueve con los sliders.
- Botones para caminar y para volver a la postura de reposo.
- **Captura de movimiento:** "Reproducir" mueve el holograma con una grabación real de una persona (`.bvh`). Al arrancar se carga `animaciones/caminar.bvh`. Con "Cargar .bvh..." se elige otra (correr, saltar, bailar, artes marciales o cualquier `.bvh` descargado), y el slider cambia la velocidad.
- Se puede mostrar el holograma, el esqueleto o los dos.
- Hay un slider para el número de puntos (de 10.000 a 300.000).

## Cómo funciona

1. **El cuerpo.** Es un modelo exportado de [MakeHuman](http://www.makehumancommunity.org/) en `.obj`, que `ObjMesh` lee. MakeHuman deforma siempre la misma malla base de 13.380 vértices, así que sus pesos de skinning (`default_weights.mhw`, licencia CC0) valen para cualquier cuerpo exportado. Para leer ese fichero hay un lector de JSON escrito a mano por descenso recursivo (`JsonParser`).
2. **Las articulaciones.** No se colocan a mano. Alrededor de cada articulación hay un anillo de vértices que siguen a los dos huesos, y el centro de ese anillo es la articulación (`MakeHumanRig`). Con esas posiciones, `HumanSkeleton` calcula la longitud y la orientación de cada segmento. El esqueleto encaja en cualquier cuerpo que se exporte.
3. **Los puntos.** Se reparten al azar por la superficie, eligiendo cada triángulo con probabilidad proporcional a su área (suma acumulada + búsqueda binaria). Así la densidad es uniforme. Cada punto hereda los pesos de su triángulo mediante coordenadas baricéntricas (`PointCloud`).
4. **El movimiento.** La cinemática directa se calcula solo para los 21 segmentos, en microsegundos. Los puntos **no** son nodos del árbol: se mueven con *linear blend skinning*, `p = Σ wᵢ · Sᵢ · p₀` con `Sᵢ = Frame_actual · Frame_reposo⁻¹`.
5. **El aspecto.** Los puntos son tetraedros diminutos con material autoiluminado. El resplandor es el efecto `Bloom` de JavaFX, aplicado a la vista 3D.

6. **La captura de movimiento.** Un `.bvh` trae el esqueleto de la persona grabada (un árbol, leído con otro parser por descenso recursivo, `BvhMotion`) y los giros de cada articulación en cada fotograma. Su esqueleto no es el nuestro: tiene otros huesos, otras proporciones, otros ejes y otra postura de reposo (en "T" en vez de en "A"), así que no se pueden copiar los ángulos. `Retargeter` copia **direcciones**:
   - Cinemática directa del `.bvh` para saber dónde está cada articulación suya.
   - Para cada segmento nuestro, la orientación que debería tener en el mundo: el eje Z va de una articulación a otra (muslo = de `RightUpLeg` a `RightLeg`). El giro sobre su propio eje sale del plano de la rodilla o el codo, que son bisagras, y si no, de cuánto ha girado esa articulación del `.bvh` desde el primer fotograma.
   - Recorriendo el árbol desde la pelvis (recursivo), se pasa a ángulos locales, `local = (padre · base)⁻¹ · deseada`, y se descompone en ángulos de Euler (`Matrix4.eulerXYZ`). Los límites articulares recortan lo imposible, y cada hijo se calcula con la orientación real del padre, ya recortada, para que los errores no se acumulen.
   - Los nombres de las articulaciones se reconocen en los formatos más comunes (CMU, Mixamo, Bandai Namco...), y los ejes del fichero (Y o Z arriba) se deducen solos. El retargeting tarda unos 20 µs por fotograma.

### Rendimiento: de 5 a 60 FPS

La primera versión recalculaba en Java cada punto en cada fotograma. Calcularlos costaba unos 4 ms, pero JavaFX tarda mucho en reprocesar una malla que cambia, y con 65.000 puntos iba a **5 FPS**.

La solución sale de la propia fórmula. Con dos huesos, `w·Sa·p + (1−w)·Sb·p = (w·Sa + (1−w)·Sb)·p`: la media ponderada de dos matrices es otra matriz. Por eso los puntos se agrupan por pareja de huesos y por peso (redondeado a 1/16). Cada grupo es una malla quieta con una única transformación, que aplica la tarjeta gráfica. En cada fotograma solo se calculan unas 650 matrices.

Resultado medido caminando (`--fpstest=8`):

| Puntos | FPS antes | FPS ahora |
|---|---|---|
| 65.000 | 5 | ~59 |
| 162.500 | 2 | ~59 |
| 325.000 | – | ~42 |

## Estructura

- `src/holograma/kinematics/`: `Matrix4`, `Vec3`, `Segment`, `Node3D`, `ForwardKinematics3D` (recursiva + iterativa), `HumanSkeleton` (esqueleto construido a partir de las articulaciones)
- `src/holograma/mocap/`: `BvhMotion` (lector de `.bvh` y su cinemática directa), `Retargeter` (pasa el movimiento del esqueleto del `.bvh` al nuestro)
- `src/holograma/body/`: `ObjMesh` (lector de .obj), `JsonParser` (JSON por descenso recursivo), `MakeHumanRig` (pesos y articulaciones), `PointCloud` (holograma)
- `src/holograma/gui/HologramApp.java`: visor JavaFX. Tiene también un modo captura (`--snapshot=f.png [--walk=s] [--yaw=grados] [--show=cloud,skeleton] [--points=n] [--bvh=f.bvh --bvhtime=s]`) y una prueba de rendimiento (`--fpstest=segundos [--nowalk=1] [--bvh=f.bvh]`).
- `modelo/`: el cuerpo exportado de MakeHuman (`cuerpo.obj`) y los pesos de la malla base (`default_weights.mhw`, CC0, de MakeHuman 1.3.0)
- `animaciones/`: capturas de movimiento de la base de datos de CMU (caminar, correr, saltar, bailar, artes marciales)
- `lib/`: JavaFX 23.0.2 (jars para Windows de Maven Central)

Para usar otro cuerpo: en MakeHuman, exportar como Wavefront obj en centímetros, sin ropa ni pelo, y guardarlo como `modelo/cuerpo.obj`.

## Ideas para seguir

- Cinemática inversa: dar la posición de la mano o del pie y calcular los ángulos (CCD o jacobiano).
- Dinámica real (masas, pares en los motores, interacción con el cuerpo): eso ya no es para JavaFX. Habría que pasar a OpenSim (tiene API Java) o a MuJoCo (Python).

Para más animaciones: la [base de datos de CMU](http://mocap.cs.cmu.edu/) tiene más de 2.500 en `.bvh` (conversión de cgspeed; hay copias en GitHub, por ejemplo [una-dinosauria/cmu-mocap](https://github.com/una-dinosauria/cmu-mocap)), y también funcionan las de Mixamo y las del dataset de Bandai Namco.

Créditos: modelo y pesos generados con MakeHuman (makehumancommunity.org). Los modelos exportados y `default_weights.mhw` tienen licencia CC0. Las animaciones de `animaciones/` son de la CMU Graphics Lab Motion Capture Database (mocap.cs.cmu.edu), creada con la financiación de la NSF EIA-0196217, en la conversión a BVH de cgspeed (sujetos 02_01, 09_01, 49_02, 05_01 y 135_01).

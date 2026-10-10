package holograma.kinematics;

import java.util.ArrayList;
import java.util.List;

/**
 * Segmento del exoesqueleto en 3D (un "hueso"). Es el Segment del lab2
 * ampliado:
 *
 * <ul>
 * <li>Lab2: longitud + 1 ángulo + lista de hijos.</li>
 * <li>Aquí: longitud + rotación base fija + 3 ángulos de articulación (uno
 * por eje) con sus límites + lista de hijos + referencia al padre.</li>
 * </ul>
 *
 * Igual que en el lab2, Segment es un TIPO RECURSIVO: contiene una lista de
 * objetos de su mismo tipo (children). Por eso un único Segment raíz
 * representa el árbol entero, y los algoritmos que lo recorren son recursivos
 * de forma natural.
 *
 * <h2>Los dos tipos de rotación</h2>
 * 1. Rotación BASE (fija): hacia dónde apunta el segmento respecto a su padre
 * en la postura de reposo. Por ejemplo, la clavícula sale hacia el lado y el
 * muslo hacia abajo. No cambia nunca.
 *
 * 2. Ángulos de la ARTICULACIÓN (variables): lo que mueve el usuario o la
 * animación. Hay uno por eje local (X, Y, Z), y cada uno tiene un mínimo y un
 * máximo, como las articulaciones reales. Un eje con min == max está
 * bloqueado: la rodilla solo tiene libre el X (flexión), mientras que el
 * hombro tiene los tres.
 *
 * <h2>Convenio</h2>
 * Cada segmento se extiende a lo largo de su eje Z local, desde la
 * articulación donde empieza hasta su extremo. Unidades: cm y radianes.
 */
public class Segment {

	// ---- Datos cinemáticos ----
	private final String name;
	private final double length; // en cm
	private final Matrix4 base; // rotación fija respecto al padre

	// Arrays de 3 posiciones: índice 0 = eje X, 1 = eje Y, 2 = eje Z.
	// Se usan arrays en vez de 3 variables (angleX, angleY, angleZ) para poder
	// recorrer los ejes con un bucle for en vez de repetir el código 3 veces.
	private final double[] angles = new double[3]; // ángulos actuales (radianes)
	private final double[] min = new double[3]; // límite inferior de cada eje
	private final double[] max = new double[3]; // límite superior de cada eje
	private final String[] axisNames = new String[3]; // "Flexión", "Abducción"...
	// Si el último setAngle de cada eje tuvo que recortarse: el movimiento pedía
	// más de lo que deja la articulación (la interfaz pinta la esfera en rojo)
	private final boolean[] clamped = new boolean[3];
	// Margen para no marcar como recortado un redondeo: 1º en radianes
	private static final double CLAMP_TOLERANCE = Math.toRadians(1);

	// ---- Estructura del árbol ----
	private final List<Segment> children = new ArrayList<>();
	// Referencia al padre: no estaba en el lab2. Hace falta para el skinning,
	// porque los vértices cerca de la articulación se mezclan con el hueso padre.
	private Segment parent;

	/**
	 * Constructor completo.
	 *
	 * @param name   Nombre para mostrar en la interfaz.
	 * @param length Longitud en cm.
	 * @param base   Rotación fija respecto al padre en la postura de reposo.
	 */
	public Segment(String name, double length, Matrix4 base) {
		this.name = name;
		this.length = length;
		this.base = base;
		// Los arrays angles, min y max empiezan llenos de 0 (valor por defecto
		// de double en Java), así que todos los ejes nacen bloqueados en 0.
	}

	/**
	 * Segmento sin rotación base: sigue en la misma dirección que su padre (como
	 * la tibia, que continúa al muslo). Llama al otro constructor con this(...),
	 * así no se repite código.
	 */
	public Segment(String name, double length) {
		this(name, length, Matrix4.identity());
	}

	/**
	 * Libera un eje de la articulación y le pone límites.
	 *
	 * Devuelve {@code this} para poder encadenar llamadas (patrón "fluent" o
	 * "builder"):
	 * {@code new Segment(...).joint(0, ...).joint(1, ...)}. Así el
	 * esqueleto entero se define de forma muy compacta en HumanSkeleton.
	 *
	 * @param axis     0 = X, 1 = Y, 2 = Z (ejes locales del segmento).
	 * @param axisName Nombre anatómico del movimiento ("Flexión"...).
	 * @param minDeg   Límite inferior en GRADOS (más cómodo de escribir).
	 * @param maxDeg   Límite superior en grados.
	 */
	public Segment joint(int axis, String axisName, double minDeg, double maxDeg) {
		axisNames[axis] = axisName;
		// Internamente todo se guarda en radianes, que es lo que usan Math.sin/cos
		min[axis] = Math.toRadians(minDeg);
		max[axis] = Math.toRadians(maxDeg);
		return this;
	}

	/**
	 * Añade un hijo, como en el lab2: el contains evita duplicados en la cadena
	 * cinemática. Además guarda en el hijo quién es su padre (enlace doble:
	 * padre -> hijo con la lista, hijo -> padre con este atributo).
	 */
	public void addChild(Segment child) {
		if (!children.contains(child)) {
			children.add(child);
			child.parent = this;
		}
	}

	/**
	 * Transformación local del segmento: cómo está colocado respecto al FINAL
	 * de su padre.
	 *
	 * local = base * rotX(ángulo X) * rotY(ángulo Y) * rotZ(ángulo Z)
	 *
	 * Se lee de izquierda a derecha como "giros sobre los ejes ya girados": se
	 * parte de la orientación base, luego se gira sobre el X local, después sobre
	 * el Y ya girado y por último sobre el Z. Esto se llama ángulos de Euler (en
	 * orden X-Y-Z), y por eso el orden importa (las rotaciones 3D no conmutan,
	 * ver Matrix4).
	 *
	 * Este método sustituye a "link.getAngle()" del lab2.
	 */
	public Matrix4 localTransform() {
		return base.multiply(Matrix4.rotX(angles[0]))
				.multiply(Matrix4.rotY(angles[1]))
				.multiply(Matrix4.rotZ(angles[2]));
	}

	/**
	 * Cambia el ángulo de un eje, recortándolo a sus límites ("clamp"):
	 * Math.min(max, x) impide pasarse por arriba y Math.max(min, ...) por abajo.
	 * Así, por mucho que se le pida, la rodilla nunca se dobla hacia delante.
	 *
	 * Además apunta si ha hecho falta recortar: si el ángulo pedido y el que
	 * se queda se diferencian en más de CLAMP_TOLERANCE, el eje ha llegado a
	 * su límite.
	 */
	public void setAngle(int axis, double angle) {
		angles[axis] = Math.max(min[axis], Math.min(max[axis], angle));
		clamped[axis] = Math.abs(angle - angles[axis]) > CLAMP_TOLERANCE;
	}

	/**
	 * Nombre del primer eje que se ha quedado en su límite en el último
	 * setAngle, o null si ninguno. Los ejes bloqueados (min = max) no cuentan:
	 * ahí el recorte es justo lo que se quiere (la rodilla no gira de lado).
	 */
	public String limitAxis() {
		for (int i = 0; i < 3; i++)
			if (clamped[i] && isAxisFree(i))
				return axisNames[i];
		return null;
	}

	/** Ángulo actual de un eje, en radianes. */
	public double getAngle(int axis) {
		return angles[axis];
	}

	/** Vuelve a la postura de reposo (los tres ángulos a 0). */
	public void resetAngles() {
		for (int i = 0; i < 3; i++)
			setAngle(i, 0);
	}

	/** Un eje está libre si su rango no es un único punto. */
	public boolean isAxisFree(int axis) {
		return min[axis] != max[axis];
	}

	/**
	 * Devuelve todos los segmentos del subárbol (este incluido) en una lista
	 * plana. Así, para hacer algo con todos los segmentos basta con un for, sin
	 * tener que recorrer el árbol cada vez.
	 *
	 * El orden es PREORDEN: primero el nodo y luego sus hijos. Es el mismo
	 * recorrido que hace la cinemática directa.
	 */
	public List<Segment> flatten() {
		List<Segment> list = new ArrayList<>();
		collect(this, list);
		return list;
	}

	// Método recursivo auxiliar (mismo patrón que computePositions del lab2:
	// método público de "fachada" + método privado recursivo).
	// Caso base implícito: un segmento sin hijos no entra en el bucle.
	private static void collect(Segment s, List<Segment> list) {
		list.add(s);
		for (Segment c : s.children)
			collect(c, list);
	}

	// ---- Getters (solo lectura: desde fuera no se pueden cambiar estos datos) ----

	public String getName() {
		return name;
	}

	public double getLength() {
		return length;
	}

	public Matrix4 getBase() {
		return base;
	}

	public double getMin(int axis) {
		return min[axis];
	}

	public double getMax(int axis) {
		return max[axis];
	}

	public String getAxisName(int axis) {
		return axisNames[axis];
	}

	public List<Segment> getChildren() {
		return children;
	}

	public Segment getParent() {
		return parent;
	}

	/**
	 * toString se sobrescribe para que el desplegable (ComboBox) de la interfaz
	 * muestre el nombre del segmento en vez de algo como "Segment@1b6d3586".
	 */
	@Override
	public String toString() {
		return name;
	}
}

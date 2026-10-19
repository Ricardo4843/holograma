package holograma.body;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

import holograma.kinematics.Matrix4;
import holograma.kinematics.Vec3;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.image.WritableImage;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Box;
import javafx.scene.shape.CullFace;
import javafx.scene.shape.Cylinder;
import javafx.scene.shape.MeshView;
import javafx.scene.shape.TriangleMesh;
import javafx.scene.transform.Affine;
import javafx.scene.transform.Translate;

/**
 * Armadura tipo Iron Man hecha a medida del cuerpo de MakeHuman: una placa
 * rígida por segmento, sacada de la propia piel.
 *
 * <h2>1. De qué trozos de piel sale cada placa</h2>
 * Cada vértice de la malla sigue a uno o varios segmentos (sus pesos de
 * skinning). Un triángulo entra en la placa de un segmento si sus TRES
 * vértices siguen a ese segmento con un peso de al menos {@link #THRESHOLD}
 * ({@link #TRUNK_THRESHOLD} en el tronco).
 * Los triángulos de las zonas de mezcla (codos, rodillas, cintura...) no
 * entran en ninguna placa, y ahí quedan huecos, igual que en una armadura de
 * verdad: si una placa cubriese la piel que se dobla, se clavaría al moverse.
 * Por los huecos se sigue viendo el holograma.
 *
 * <h2>2. Separarla de la piel: la normal de cada vértice</h2>
 * La placa es la piel empujada hacia fuera {@link #OUTER} cm. "Hacia fuera"
 * es la NORMAL del vértice: la media de las normales de los triángulos que lo
 * tocan, cada una pesando según su área (el producto vectorial de dos lados
 * mide el doble del área y ya apunta en perpendicular, así que basta con
 * sumarlos sin normalizar).
 *
 * <h2>3. Darle grosor: el canto</h2>
 * Una superficie sola parece de papel. En los bordes de cada placa se añade
 * un canto que baja de la capa exterior a una interior (a {@link #INNER} cm de
 * la piel). Un borde es una arista que solo usa UN triángulo de la placa: si
 * la usasen dos, estaría por dentro. Para contarlas se usa un mapa cuya clave
 * es la arista (sus dos vértices, el menor primero, empaquetados en un long).
 *
 * <h2>4. Moverla</h2>
 * Cada placa sigue al 100% a un único segmento, así que se mueve de forma
 * rígida con su matriz de skinning, S = Frame_actual * Frame_reposo^-1. Es
 * una sola Affine por segmento, la aplica la tarjeta gráfica y no hay que
 * tocar ni un vértice en cada fotograma (como los cubos de PointCloud, pero
 * sin mezclar huesos).
 *
 * <h2>5. Los detalles</h2>
 * El reactor del pecho y los ojos son piezas que brillan por sí mismas,
 * colgadas del grupo del tórax y de la cabeza. La "máscara" dorada son los
 * triángulos de la cabeza que miran hacia delante y están por debajo de la
 * frente.
 */
public class Armor {

	/** Peso mínimo de los 3 vértices para que un triángulo entre en una placa. */
	private static final double THRESHOLD = 0.8;
	/**
	 * Umbral más bajo para el tronco y los hombros. Ahí los pesos se mezclan en
	 * zonas muy anchas (la columna se dobla poco a poco), y con 0,8 casi todo el
	 * pecho y el abdomen se quedaban sin placa. Con 0,5 basta con que los tres
	 * vértices tengan el mismo segmento dominante, y entre placa y placa queda
	 * solo una junta fina.
	 */
	private static final double TRUNK_THRESHOLD = 0.5;
	private static final Set<String> TRUNK = Set.of("Pelvis", "Lumbar", "Tórax", "Cuello", "Clavícula D",
			"Clavícula I");
	/** Distancia de la cara exterior de la placa a la piel (cm). */
	private static final double OUTER = 1.4;
	/** Distancia de la cara interior (donde acaba el canto) a la piel (cm). */
	private static final double INNER = 0.4;
	/** Segmentos dorados; el resto son rojos. */
	private static final Set<String> GOLD = Set.of("Lumbar", "Cuello", "Antebrazo D", "Antebrazo I", "Tibia D",
			"Tibia I");

	private final Group node = new Group();
	/** Una Affine por segmento, en el orden de MakeHumanRig.SEGMENTS (null si no tiene placa). */
	private final Affine[] affines = new Affine[MakeHumanRig.SEGMENTS.length];
	private final double[] vertices; // posición de cada vértice en reposo
	private final double[][] normals; // normal de cada vértice (unitaria, hacia fuera)
	private boolean flipped; // true si los triángulos del .obj giran al revés (ver vertexNormals)
	private int triangleCount;

	public Armor(MakeHumanRig rig) {
		ObjMesh m = rig.getMesh();
		double[][] w = rig.getWeights();
		vertices = m.vertices;
		int n = m.vertexCount();

		// Segmento dominante de cada vértice y su peso
		int[] dominant = new int[n];
		double[] weight = new double[n];
		for (int v = 0; v < n; v++)
			for (int s = 0; s < w[v].length; s++)
				if (w[v][s] > weight[v]) {
					weight[v] = w[v][s];
					dominant[v] = s;
				}

		normals = vertexNormals(m);

		// Ojos: los vértices de la malla aparte (grupo distinto de "base.obj").
		// Su centro dice dónde poner los ojos del casco y hasta dónde llega la
		// máscara (un poco por encima de ellos). [lado][x, y, z, cuántos]
		double[][] eyes = new double[2][4];
		for (int t = 0; t < m.triangleCount(); t++)
			if (!m.triangleGroup[t].equals("base.obj"))
				for (int k = 0; k < 3; k++) {
					int v = m.triangles[3 * t + k];
					double[] e = eyes[vertices[3 * v] > 0 ? 0 : 1]; // x > 0 = derecha
					for (int c = 0; c < 3; c++)
						e[c] += vertices[3 * v + c];
					e[3]++;
				}
		double[][] eyeCenter = new double[2][];
		for (int i = 0; i < 2; i++)
			eyeCenter[i] = eyes[i][3] > 0 ? Vec3.scale(new double[] { eyes[i][0], eyes[i][1], eyes[i][2] }, 1 / eyes[i][3])
					: null;
		double browZ = eyeCenter[0] != null ? eyeCenter[0][2] + 3.5 : Double.NEGATIVE_INFINITY;

		// 1) Repartir los triángulos entre las placas. Cada placa se parte en dos
		// listas (roja y dorada) porque una malla de JavaFX solo tiene un material.
		int head = MakeHumanRig.segmentIndex("Cabeza");
		List<List<int[]>> red = new ArrayList<>(), gold = new ArrayList<>();
		for (int s = 0; s < MakeHumanRig.SEGMENTS.length; s++) {
			red.add(new ArrayList<>());
			gold.add(new ArrayList<>());
		}
		for (int t = 0; t < m.triangleCount(); t++) {
			if (!m.triangleGroup[t].equals("base.obj"))
				continue; // los ojos no llevan placa
			int a = m.triangles[3 * t], b = m.triangles[3 * t + 1], c = m.triangles[3 * t + 2];
			int s = dominant[a];
			double min = TRUNK.contains(MakeHumanRig.SEGMENTS[s]) ? TRUNK_THRESHOLD : THRESHOLD;
			if (dominant[b] != s || dominant[c] != s || Math.min(weight[a], Math.min(weight[b], weight[c])) < min)
				continue; // zona de mezcla: hueco
			int[] tri = { a, b, c };
			boolean isGold = GOLD.contains(MakeHumanRig.SEGMENTS[s]) || s == head && isFace(tri, browZ);
			(isGold ? gold : red).get(s).add(tri);
		}

		PhongMaterial redMat = metal("#a3161c", "#ffc2b0");
		PhongMaterial goldMat = metal("#c99a2e", "#fff0c0");
		for (int s = 0; s < MakeHumanRig.SEGMENTS.length; s++) {
			if (red.get(s).isEmpty() && gold.get(s).isEmpty())
				continue; // segmento sin piel propia (las "caderas" de unión)
			// 3) Bordes de la placa entera (roja + dorada juntas): así no sale un
			// canto entre la máscara y el resto del casco
			List<int[]> all = new ArrayList<>(red.get(s));
			all.addAll(gold.get(s));
			Map<Long, Integer> edgeUses = new HashMap<>();
			for (int[] tri : all)
				for (int k = 0; k < 3; k++)
					edgeUses.merge(edgeKey(tri[k], tri[(k + 1) % 3]), 1, Integer::sum);

			affines[s] = new Affine();
			Group plate = new Group();
			plate.getTransforms().add(affines[s]);
			for (int g = 0; g < 2; g++) {
				List<int[]> tris = g == 0 ? red.get(s) : gold.get(s);
				if (tris.isEmpty())
					continue;
				MeshView view = new MeshView(plateMesh(tris, edgeUses));
				view.setMaterial(g == 0 ? redMat : goldMat);
				view.setCullFace(CullFace.NONE); // por los huecos se ve el interior de la placa
				plate.getChildren().add(view);
				triangleCount += tris.size();
			}
			node.getChildren().add(plate);
		}

		// 5) Detalles que brillan
		addReactor(rig, w);
		for (double[] eye : eyeCenter)
			if (eye != null)
				addEye(eye, head, dominant, weight);
	}

	/**
	 * ¿Es un triángulo de la máscara? Mira hacia delante (su normal tiene
	 * componente Y positiva clara) y está por debajo de la frente.
	 */
	private boolean isFace(int[] tri, double browZ) {
		double ny = 0, z = 0;
		for (int v : tri) {
			ny += normals[v][1];
			z += vertices[3 * v + 2];
		}
		return ny / 3 > 0.35 && z / 3 < browZ;
	}

	/**
	 * Normal de cada vértice: suma de los productos vectoriales de sus
	 * triángulos (ver punto 2 del javadoc), normalizada.
	 *
	 * El producto vectorial apunta hacia fuera o hacia dentro según el orden
	 * en que estén escritos los vértices del triángulo. Para no depender de
	 * eso, al final se comprueba: si de media las normales apuntan hacia el
	 * centro del cuerpo, se les da la vuelta a todas.
	 */
	private double[][] vertexNormals(ObjMesh m) {
		double[][] nrm = new double[m.vertexCount()][3];
		double[] v = m.vertices;
		for (int t = 0; t < m.triangleCount(); t++) {
			if (!m.triangleGroup[t].equals("base.obj"))
				continue;
			int a = m.triangles[3 * t], b = m.triangles[3 * t + 1], c = m.triangles[3 * t + 2];
			double[] pa = point(v, a);
			double[] cross = Vec3.cross(Vec3.sub(point(v, b), pa), Vec3.sub(point(v, c), pa));
			for (int i : new int[] { a, b, c })
				for (int k = 0; k < 3; k++)
					nrm[i][k] += cross[k];
		}
		// Centro del cuerpo y comprobación del sentido
		double[] center = new double[3];
		for (int i = 0; i < m.vertexCount(); i++)
			for (int k = 0; k < 3; k++)
				center[k] += v[3 * i + k] / m.vertexCount();
		double outward = 0;
		for (int i = 0; i < m.vertexCount(); i++)
			outward += Vec3.dot(nrm[i], Vec3.sub(point(v, i), center));
		flipped = outward < 0;
		double sign = flipped ? -1 : 1;
		for (int i = 0; i < nrm.length; i++) {
			double len = Vec3.norm(nrm[i]);
			nrm[i] = len > 0 ? Vec3.scale(nrm[i], sign / len) : new double[3];
		}
		return nrm;
	}

	private static double[] point(double[] v, int i) {
		return new double[] { v[3 * i], v[3 * i + 1], v[3 * i + 2] };
	}

	/** Clave de una arista sin importar el sentido: los dos índices, el menor delante. */
	private static long edgeKey(int a, int b) {
		return (long) Math.min(a, b) << 32 | Math.max(a, b);
	}

	/** Vértice desplazado d cm por su normal. */
	private double[] offset(int v, double d) {
		return Vec3.add(point(vertices, v), Vec3.scale(normals[v], d));
	}

	/**
	 * Malla de una placa: la capa exterior y, en cada borde, un canto de dos
	 * triángulos que baja hasta la capa interior.
	 *
	 * Los vértices de la malla original se renumeran (la placa solo usa unos
	 * pocos de los 13.380): los mapas outer e inner dan el índice nuevo de cada
	 * vértice en la capa exterior y en la interior.
	 */
	private TriangleMesh plateMesh(List<int[]> tris, Map<Long, Integer> edgeUses) {
		List<Float> pts = new ArrayList<>();
		List<Integer> faces = new ArrayList<>(), groups = new ArrayList<>();
		Map<Integer, Integer> outer = new HashMap<>(), inner = new HashMap<>();
		for (int[] t : tris) {
			// Que todos giren en sentido antihorario vistos desde fuera
			int[] tri = flipped ? new int[] { t[0], t[2], t[1] } : t;
			int[] o = new int[3];
			for (int k = 0; k < 3; k++)
				o[k] = index(tri[k], OUTER, outer, pts);
			addFace(faces, groups, o[0], o[1], o[2], 1);
			for (int k = 0; k < 3; k++) {
				int a = tri[k], b = tri[(k + 1) % 3];
				if (edgeUses.get(edgeKey(a, b)) != 1)
					continue; // arista interior
				// Canto del borde a -> b: exterior a, interior a, interior b, exterior
				// b. Con este orden su normal apunta hacia fuera de la placa (si el
				// triángulo gira en sentido antihorario visto desde fuera, el
				// interior queda a la izquierda de a -> b y el canto mira a la derecha).
				int ia = index(a, INNER, inner, pts), ib = index(b, INNER, inner, pts);
				addFace(faces, groups, o[k], ia, ib, 2);
				addFace(faces, groups, o[k], ib, o[(k + 1) % 3], 2);
			}
		}
		TriangleMesh mesh = new TriangleMesh();
		float[] p = new float[pts.size()];
		for (int i = 0; i < p.length; i++)
			p[i] = pts.get(i);
		mesh.getPoints().addAll(p);
		mesh.getTexCoords().addAll(0, 0);
		mesh.getFaces().addAll(faces.stream().mapToInt(Integer::intValue).toArray());
		// Grupos de suavizado: la capa exterior (1) se ve curva y lisa, y el canto
		// (2) aparte, para que la esquina entre los dos se vea viva y no redondeada
		mesh.getFaceSmoothingGroups().addAll(groups.stream().mapToInt(Integer::intValue).toArray());
		return mesh;
	}

	/** Índice del vértice v desplazado d cm; lo crea la primera vez que se pide. */
	private int index(int v, double d, Map<Integer, Integer> map, List<Float> pts) {
		return map.computeIfAbsent(v, key -> {
			double[] p = offset(key, d);
			for (double c : p)
				pts.add((float) c);
			return pts.size() / 3 - 1;
		});
	}

	/** Añade un triángulo (vértice, textura) x 3, con la textura siempre 0. */
	private static void addFace(List<Integer> faces, List<Integer> groups, int a, int b, int c, int group) {
		faces.addAll(List.of(a, 0, b, 0, c, 0));
		groups.add(group);
	}

	/**
	 * Reactor del pecho: un disco que brilla, con un aro de metal detrás, en
	 * el esternón (en el centro, unos 14 cm por debajo de la base del cuello).
	 * Se coloca en el vértice del tórax más adelantado de esa zona.
	 */
	private void addReactor(MakeHumanRig rig, double[][] w) {
		int chest = MakeHumanRig.segmentIndex("Tórax");
		if (affines[chest] == null)
			return;
		double targetZ = rig.getJoints().get("neck_base")[2] - 14;
		int best = -1;
		for (int v = 0; v < w.length; v++) {
			double x = vertices[3 * v], z = vertices[3 * v + 2];
			if (w[v][chest] > 0.5 && Math.abs(x) < 2.5 && Math.abs(z - targetZ) < 2.5
					&& (best < 0 || vertices[3 * v + 1] > vertices[3 * best + 1]))
				best = v;
		}
		if (best < 0)
			return;
		double[] nrm = normals[best];
		Cylinder ring = new Cylinder(3.6, 0.8);
		ring.setMaterial(metal("#8a8f99", "#ffffff"));
		orient(ring, offset(best, OUTER + 0.2), nrm);
		Cylinder core = new Cylinder(2.5, 0.8);
		core.setMaterial(glowing(Color.web("#bff4ff")));
		orient(core, offset(best, OUTER + 0.5), nrm);
		plateOf(chest).getChildren().addAll(ring, core);
	}

	/**
	 * Ojo del casco: una ranura que brilla delante del ojo, a la altura de la
	 * cara exterior del casco en esa zona.
	 */
	private void addEye(double[] eye, int head, int[] dominant, double[] weight) {
		if (affines[head] == null)
			return;
		double front = eye[1];
		for (int v = 0; v < dominant.length; v++)
			if (dominant[v] == head && weight[v] >= THRESHOLD && Math.abs(vertices[3 * v] - eye[0]) < 2
					&& Math.abs(vertices[3 * v + 2] - eye[2]) < 2)
				front = Math.max(front, offset(v, OUTER)[1]);
		Box slit = new Box(3.0, 1.6, 0.9); // ancho (x), fondo (y), alto (z)
		slit.setMaterial(glowing(Color.web("#e6fbff")));
		slit.getTransforms().add(new Translate(eye[0], front - 0.6, eye[2]));
		plateOf(head).getChildren().add(slit);
	}

	/** Grupo de la placa del segmento s (el que lleva su Affine). */
	private Group plateOf(int s) {
		for (Node child : node.getChildren())
			if (child.getTransforms().contains(affines[s]))
				return (Group) child;
		throw new IllegalStateException("Sin placa: " + MakeHumanRig.SEGMENTS[s]);
	}

	/**
	 * Coloca un cilindro de JavaFX (orientado según su eje Y local) con su eje
	 * a lo largo de "axis" y centrado en "center". Las columnas de la matriz
	 * son los ejes nuevos: Y = axis y X, Z dos perpendiculares cualesquiera.
	 */
	private static void orient(Cylinder c, double[] center, double[] axis) {
		double[] y = Vec3.normalize(axis);
		double[] ref = Math.abs(y[2]) < 0.9 ? new double[] { 0, 0, 1 } : new double[] { 1, 0, 0 };
		double[] x = Vec3.normalize(Vec3.cross(y, ref));
		double[] z = Vec3.cross(x, y);
		double[] v = Matrix4.translation(center[0], center[1], center[2]).multiply(Matrix4.fromAxes(x, y, z)).toArray();
		c.getTransforms().add(new Affine(v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8], v[9], v[10], v[11]));
	}

	/** Metal pintado: color difuso más un brillo especular claro. */
	private static PhongMaterial metal(String diffuse, String specular) {
		PhongMaterial m = new PhongMaterial(Color.web(diffuse));
		m.setSpecularColor(Color.web(specular));
		m.setSpecularPower(28);
		return m;
	}

	/** Material que brilla por sí mismo (igual que los puntos del holograma). */
	private static PhongMaterial glowing(Color color) {
		WritableImage pixel = new WritableImage(1, 1);
		pixel.getPixelWriter().setColor(0, 0, color);
		PhongMaterial m = new PhongMaterial(Color.BLACK);
		m.setSpecularColor(Color.BLACK);
		m.setSelfIlluminationMap(pixel);
		return m;
	}

	/**
	 * Pone la armadura en la postura actual.
	 *
	 * @param skin matriz de skinning de cada segmento, en el orden de
	 *             MakeHumanRig.SEGMENTS (12 doubles por filas)
	 */
	public void update(double[][] skin) {
		for (int s = 0; s < affines.length; s++)
			if (affines[s] != null) {
				double[] m = skin[s];
				affines[s].setToTransform(m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8], m[9], m[10], m[11]);
			}
	}

	public Group getNode() {
		return node;
	}

	/** Triángulos de la capa exterior de todas las placas. */
	public int getTriangleCount() {
		return triangleCount;
	}
}

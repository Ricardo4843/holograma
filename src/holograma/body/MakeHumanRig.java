package holograma.body;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * "Rig" del cuerpo de MakeHuman: une la malla 3D con el esqueleto.
 *
 * Hace dos cosas:
 * 1. Lee los PESOS de skinning de MakeHuman (qué cantidad de cada vértice
 *    sigue a cada hueso) y los agrupa en los 21 segmentos de HumanSkeleton.
 * 2. A partir de esos pesos, DEDUCE dónde están las articulaciones dentro de
 *    este cuerpo concreto. Así el esqueleto encaja aunque el cuerpo sea más
 *    alto, más bajo o más ancho.
 *
 * <h2>Por qué funciona con cualquier cuerpo de MakeHuman</h2>
 * MakeHuman deforma siempre la MISMA malla base (13.380 vértices) con sus
 * sliders: cambia dónde está cada vértice, pero nunca su número de orden. El
 * vértice 4849 es siempre el mismo punto de la nariz, sea cual sea el cuerpo.
 * Por eso los pesos de default_weights.mhw (licencia CC0), escritos para esa
 * malla base, sirven para cualquier cuerpo exportado.
 *
 * <h2>Cómo se encuentra una articulación</h2>
 * Alrededor del codo hay un anillo de vértices que siguen un poco al brazo y
 * un poco al antebrazo (por eso la piel se dobla suave). El centro de ese
 * anillo es el centro del codo. Se calcula como una media ponderada:
 *
 * <pre>
 * codo = suma(min(peso_brazo, peso_antebrazo) * posición) / suma(min(...))
 * </pre>
 *
 * Los vértices que solo siguen a uno de los dos huesos tienen min = 0 y no
 * cuentan.
 */
public class MakeHumanRig {

	/**
	 * Columnas de la tabla de pesos: un segmento de HumanSkeleton por columna.
	 * Deben coincidir con los nombres que se usan allí.
	 */
	public static final String[] SEGMENTS = { "Pelvis", "Lumbar", "Tórax", "Cuello", "Cabeza",
			"Clavícula D", "Brazo D", "Antebrazo D", "Mano D", "Cadera D", "Muslo D", "Tibia D", "Pie D",
			"Clavícula I", "Brazo I", "Antebrazo I", "Mano I", "Cadera I", "Muslo I", "Tibia I", "Pie I" };

	private final ObjMesh mesh;
	/** weights[v][s] = cuánto sigue el vértice v al segmento SEGMENTS[s] (cada fila suma 1). */
	private final double[][] weights;
	/** Articulaciones encontradas (mismos nombres que HumanSkeleton.defaultJoints). */
	private final Map<String, double[]> joints = new HashMap<>();

	private MakeHumanRig(ObjMesh mesh, double[][] weights) {
		this.mesh = mesh;
		this.weights = weights;
	}

	public ObjMesh getMesh() {
		return mesh;
	}

	public double[][] getWeights() {
		return weights;
	}

	public Map<String, double[]> getJoints() {
		return joints;
	}

	/** Índice de un segmento en la tabla de pesos (búsqueda lineal: solo hay 21). */
	public static int segmentIndex(String name) {
		for (int i = 0; i < SEGMENTS.length; i++)
			if (SEGMENTS[i].equals(name))
				return i;
		throw new IllegalArgumentException("Segmento desconocido: " + name);
	}

	/**
	 * Carga la malla y los pesos y calcula las articulaciones.
	 *
	 * @param objFile     cuerpo exportado de MakeHuman (.obj)
	 * @param weightsFile pesos de la malla base de MakeHuman (.mhw, JSON)
	 */
	@SuppressWarnings("unchecked") // el JSON devuelve Object y hay que convertirlo (cast)
	public static MakeHumanRig load(Path objFile, Path weightsFile) throws IOException {
		ObjMesh mesh = ObjMesh.load(objFile);
		int n = mesh.vertexCount();
		double[][] w = new double[n][SEGMENTS.length];

		// Vértices del cuerpo: los que usan las caras del grupo "base.obj"
		// (13.380). En la malla base de MakeHuman, los índices siguientes son
		// geometría auxiliar que no se exporta, y en el .obj exportado esos
		// mismos números los ocupan los ojos. Por eso los pesos con índice >=
		// bodyVertices hay que ignorarlos: no son de este vértice.
		int bodyVertices = 0;
		for (int t = 0; t < mesh.triangleCount(); t++)
			if (mesh.triangleGroup[t].equals("base.obj"))
				for (int k = 0; k < 3; k++)
					bodyVertices = Math.max(bodyVertices, mesh.triangles[3 * t + k] + 1);

		// Para colocar el final de manos y pies se usa el centro de las yemas de
		// los dedos y de los dedos de los pies: [lado][x, y, z, peso total]
		double[][] fingertips = new double[2][4], toes = new double[2][4];

		// 1) Leer el JSON: { "weights": { "nombreHueso": [[vértice, peso], ...], ... } }
		Map<String, Object> json = (Map<String, Object>) JsonParser.parse(Files.readString(weightsFile));
		Map<String, Object> bones = (Map<String, Object>) json.get("weights");
		for (Map.Entry<String, Object> e : bones.entrySet()) {
			String bone = e.getKey();
			int seg = segmentIndex(segmentFor(bone));
			int side = bone.endsWith(".R") ? 0 : 1; // 0 = derecha, 1 = izquierda
			for (Object pair : (List<Object>) e.getValue()) {
				List<Object> p = (List<Object>) pair;
				int v = ((Double) p.get(0)).intValue();
				double weight = (Double) p.get(1);
				if (v >= bodyVertices) // geometría auxiliar (ver arriba)
					continue;
				w[v][seg] += weight;
				if (bone.startsWith("finger") && bone.contains("-3."))
					accumulate(fingertips[side], mesh, v, weight);
				if (bone.startsWith("toe"))
					accumulate(toes[side], mesh, v, weight);
			}
		}

		// 2) Los ojos son una malla aparte (grupo distinto de "base.obj") sin
		// pesos: siguen al 100% a la cabeza
		int head = segmentIndex("Cabeza");
		for (int t = 0; t < mesh.triangleCount(); t++)
			if (!mesh.triangleGroup[t].equals("base.obj"))
				for (int k = 0; k < 3; k++) {
					double[] row = w[mesh.triangles[3 * t + k]];
					java.util.Arrays.fill(row, 0);
					row[head] = 1;
				}

		// 3) Normalizar: que los pesos de cada vértice sumen exactamente 1 (en
		// el fichero suman entre 0,95 y 1,06 por redondeos)
		for (double[] row : w) {
			double sum = 0;
			for (double x : row)
				sum += x;
			if (sum == 0)
				row[head] = 1; // por si acaso: un vértice sin pesos sigue a la cabeza
			else
				for (int s = 0; s < row.length; s++)
					row[s] /= sum;
		}

		MakeHumanRig rig = new MakeHumanRig(mesh, w);
		rig.findJoints(fingertips, toes);
		return rig;
	}

	/**
	 * Traduce un hueso de MakeHuman (tiene 139, incluidos los de la cara y cada
	 * falange de los dedos) al segmento del esqueleto que lo engloba. En
	 * MakeHuman ".L" es el lado izquierdo del personaje y ".R" el derecho.
	 */
	static String segmentFor(String bone) {
		String side = bone.endsWith(".L") ? " I" : bone.endsWith(".R") ? " D" : "";
		String b = bone.replaceAll("\\.[LR]$", ""); // nombre sin el sufijo de lado
		if (b.equals("root") || b.equals("spine05") || b.equals("pelvis"))
			return "Pelvis";
		if (b.equals("spine04") || b.equals("spine03"))
			return "Lumbar";
		if (b.equals("spine02") || b.equals("spine01") || b.equals("breast"))
			return "Tórax";
		if (b.startsWith("neck"))
			return "Cuello";
		if (b.equals("clavicle") || b.equals("shoulder01"))
			return "Clavícula" + side;
		if (b.startsWith("upperarm"))
			return "Brazo" + side;
		if (b.startsWith("lowerarm"))
			return "Antebrazo" + side;
		if (b.equals("wrist") || b.startsWith("metacarpal") || b.startsWith("finger"))
			return "Mano" + side;
		if (b.startsWith("upperleg"))
			return "Muslo" + side;
		if (b.startsWith("lowerleg"))
			return "Tibia" + side;
		if (b.equals("foot") || b.startsWith("toe"))
			return "Pie" + side;
		return "Cabeza"; // head, jaw, eye, lengua y todos los músculos de la cara
	}

	/** Calcula todas las articulaciones a partir de los pesos (ver javadoc de la clase). */
	private void findJoints(double[][] fingertips, double[][] toes) {
		joints.put("pelvis", blendCenter("Pelvis", "Lumbar"));
		joints.put("lumbar_end", blendCenter("Lumbar", "Tórax"));
		joints.put("neck_base", blendCenter("Tórax", "Cuello"));
		joints.put("head_base", blendCenter("Cuello", "Cabeza"));

		// Coronilla: encima de la base de la cabeza, a la altura del vértice más
		// alto de la cabeza
		double[] hb = joints.get("head_base");
		double top = Double.NEGATIVE_INFINITY;
		int head = segmentIndex("Cabeza");
		for (int v = 0; v < weights.length; v++)
			if (weights[v][head] > 0.5)
				top = Math.max(top, mesh.vertices[3 * v + 2]);
		joints.put("head_top", new double[] { hb[0], hb[1], top });

		String[][] sides = { { " D", "_D" }, { " I", "_I" } };
		for (int i = 0; i < 2; i++) {
			String n = sides[i][0], j = sides[i][1];
			joints.put("shoulder" + j, blendCenter("Clavícula" + n, "Brazo" + n));
			joints.put("elbow" + j, blendCenter("Brazo" + n, "Antebrazo" + n));
			joints.put("wrist" + j, blendCenter("Antebrazo" + n, "Mano" + n));
			joints.put("hand_end" + j, average(fingertips[i]));
			joints.put("hip" + j, blendCenter("Pelvis", "Muslo" + n));
			joints.put("knee" + j, blendCenter("Muslo" + n, "Tibia" + n));
			joints.put("ankle" + j, blendCenter("Tibia" + n, "Pie" + n));
			joints.put("foot_end" + j, average(toes[i]));
		}
	}

	/** Centro del anillo de vértices compartidos por dos segmentos. */
	private double[] blendCenter(String a, String b) {
		int ia = segmentIndex(a), ib = segmentIndex(b);
		double[] acc = new double[4];
		for (int v = 0; v < weights.length; v++) {
			double k = Math.min(weights[v][ia], weights[v][ib]);
			if (k > 0)
				accumulate(acc, mesh, v, k);
		}
		if (acc[3] == 0)
			throw new IllegalStateException("No hay vértices compartidos entre " + a + " y " + b);
		return average(acc);
	}

	/** Suma la posición de un vértice multiplicada por k, y k al total (acc[3]). */
	private static void accumulate(double[] acc, ObjMesh mesh, int v, double k) {
		acc[0] += k * mesh.vertices[3 * v];
		acc[1] += k * mesh.vertices[3 * v + 1];
		acc[2] += k * mesh.vertices[3 * v + 2];
		acc[3] += k;
	}

	/** Media ponderada: suma de posiciones / suma de pesos. */
	private static double[] average(double[] acc) {
		return new double[] { acc[0] / acc[3], acc[1] / acc[3], acc[2] / acc[3] };
	}
}

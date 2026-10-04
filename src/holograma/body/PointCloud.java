package holograma.body;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;

import javafx.scene.Group;
import javafx.scene.paint.Material;
import javafx.scene.shape.CullFace;
import javafx.scene.shape.MeshView;
import javafx.scene.shape.TriangleMesh;
import javafx.scene.transform.Affine;

/**
 * Nube de puntos sobre la superficie del cuerpo, que se mueve con el
 * esqueleto (efecto "holograma").
 *
 * <h2>1. Repartir los puntos por la piel</h2>
 * Los vértices de la malla no están repartidos de forma uniforme (la cara y
 * las manos tienen muchos más que la espalda). Para que los puntos cubran el
 * cuerpo por igual, se colocan al azar sobre los triángulos, pero eligiendo
 * cada triángulo con probabilidad proporcional a su ÁREA: un triángulo el
 * doble de grande recibe, de media, el doble de puntos.
 *
 * Para elegir el triángulo se usa la suma acumulada de áreas y una BÚSQUEDA
 * BINARIA: se tira un número al azar entre 0 y el área total y se busca en
 * qué tramo cae, en O(log n) en vez de O(n).
 *
 * <h2>2. Coordenadas baricéntricas</h2>
 * Un punto dentro de un triángulo (A, B, C) se escribe como a*A + b*B + c*C
 * con a + b + c = 1 y todos positivos. Con esos mismos coeficientes se
 * mezclan también los pesos de skinning de A, B y C, así que cada punto sabe
 * a qué huesos seguir.
 *
 * <h2>3. Moverlos rápido: que trabaje la tarjeta gráfica</h2>
 * La primera versión recalculaba en Java la posición de cada punto en cada
 * fotograma y se la pasaba a JavaFX. El cálculo era rápido (unos 4 ms), pero
 * JavaFX tarda muchísimo en reprocesar una malla que cambia: con 65.000
 * puntos iba a 5 FPS. En cambio, una malla QUIETA con una transformación
 * (Affine) se dibuja a 60 FPS, porque la transformación la aplica la tarjeta
 * gráfica.
 *
 * El truco está en la fórmula del linear blend skinning con dos huesos:
 *
 * <pre>
 * p = w * (S_a * p0) + (1 - w) * (S_b * p0) = (w * S_a + (1 - w) * S_b) * p0
 * </pre>
 *
 * La media ponderada de dos matrices es OTRA MATRIZ (afín), así que todos los
 * puntos con los mismos dos huesos y el mismo peso w se mueven con una única
 * matriz. Por eso se agrupan en "cubos" (buckets): cada cubo es una malla
 * quieta con su Affine, y en cada fotograma solo se calculan unas pocas
 * centenas de matrices en vez de mover decenas de miles de puntos.
 *
 * Para que haya pocos cubos, cada punto se queda con sus 2 huesos de más
 * peso y w se redondea a múltiplos de 1/16. El error es menor de 1/32 de la
 * separación entre las dos posiciones posibles (milímetros), invisible en una
 * nube de puntos.
 *
 * <h2>4. Dibujar un punto</h2>
 * JavaFX 3D solo sabe dibujar triángulos. Cada punto se dibuja como un
 * TETRAEDRO diminuto (4 vértices y 4 caras), que se ve igual desde cualquier
 * ángulo.
 */
public class PointCloud {

	// Las 4 esquinas de un tetraedro regular centrado en el origen y sus 4 caras
	private static final double[][] TETRA = { { 1, 1, 1 }, { 1, -1, -1 }, { -1, 1, -1 }, { -1, -1, 1 } };
	private static final int[][] TETRA_FACES = { { 0, 1, 2 }, { 0, 3, 1 }, { 0, 2, 3 }, { 1, 3, 2 } };
	/** Niveles en que se redondea el peso w (ver punto 3 del javadoc). */
	private static final int LEVELS = 16;

	/**
	 * Un cubo: los puntos que siguen a los mismos dos huesos (a y b) con el
	 * mismo peso (level / LEVELS para a, el resto para b). Es un "record" porque
	 * solo agrupa datos.
	 */
	private record Bucket(int a, int b, int level, Affine affine) {
	}

	private final Group node = new Group(); // todas las mallas de la nube
	private final List<Bucket> buckets = new ArrayList<>();
	private final int count;

	/**
	 * @param rig      malla del cuerpo con sus pesos
	 * @param count    número de puntos
	 * @param size     radio de cada punto (cm)
	 * @param seed     semilla del azar (misma semilla = mismos puntos)
	 * @param material color/aspecto de los puntos
	 */
	public PointCloud(MakeHumanRig rig, int count, double size, long seed, Material material) {
		this.count = count;
		ObjMesh m = rig.getMesh();
		double[][] w = rig.getWeights();
		double[] cumulative = cumulativeAreas(m);
		double total = cumulative[cumulative.length - 1];
		Random rnd = new Random(seed);
		double[] mixed = new double[MakeHumanRig.SEGMENTS.length];

		// Mientras se reparten los puntos, cada cubo acumula sus coordenadas en
		// una lista. La clave del mapa identifica el cubo: a, b y nivel en un int.
		Map<Integer, List<Float>> pointsByBucket = new HashMap<>();

		for (int i = 0; i < count; i++) {
			// Elegir triángulo según su área (búsqueda binaria en la suma acumulada).
			// Si no encuentra el valor exacto, binarySearch devuelve
			// -(punto de inserción) - 1, y de ahí se recupera la posición.
			int t = Arrays.binarySearch(cumulative, rnd.nextDouble() * total);
			if (t < 0)
				t = -t - 1;
			int ia = m.triangles[3 * t], ib = m.triangles[3 * t + 1], ic = m.triangles[3 * t + 2];

			// Coordenadas baricéntricas uniformes. La raíz cuadrada es necesaria:
			// sin ella los puntos se amontonarían cerca de la esquina A.
			double r1 = Math.sqrt(rnd.nextDouble()), r2 = rnd.nextDouble();
			double a = 1 - r1, b = r1 * (1 - r2), c = r1 * r2;
			double[] p = new double[3];
			for (int k = 0; k < 3; k++)
				p[k] = a * m.vertices[3 * ia + k] + b * m.vertices[3 * ib + k] + c * m.vertices[3 * ic + k];

			// Pesos del punto = mezcla de los pesos de las 3 esquinas
			for (int s = 0; s < mixed.length; s++)
				mixed[s] = a * w[ia][s] + b * w[ib][s] + c * w[ic][s];

			// Los dos huesos de más peso y el peso relativo del primero, redondeado
			int first = argMax(mixed, -1), second = argMax(mixed, first);
			int level = (int) Math.round(LEVELS * mixed[first] / (mixed[first] + mixed[second]));
			if (level == LEVELS)
				second = first; // sigue solo al primero: da igual cuál sea el segundo
			int key = (first * 100 + second) * 100 + level; // "empaqueta" los 3 datos en un int

			// computeIfAbsent: si el cubo aún no existe, crea su lista vacía
			List<Float> list = pointsByBucket.computeIfAbsent(key, k -> new ArrayList<>());
			for (double[] corner : TETRA)
				for (int k = 0; k < 3; k++)
					list.add((float) (p[k] + corner[k] * size / Math.sqrt(3)));
		}

		// Crear una malla quieta por cubo. Sus puntos están en la posición de
		// REPOSO; la Affine del cubo los llevará a la postura actual.
		for (Map.Entry<Integer, List<Float>> e : pointsByBucket.entrySet()) {
			int key = e.getKey();
			Bucket bucket = new Bucket(key / 10000, (key / 100) % 100, key % 100, new Affine());
			MeshView view = new MeshView(tetraMesh(e.getValue()));
			view.setMaterial(material);
			// Que se dibujen las dos caras de cada triángulo (por defecto JavaFX
			// oculta las que miran hacia atrás para ahorrar trabajo)
			view.setCullFace(CullFace.NONE);
			view.getTransforms().add(bucket.affine());
			buckets.add(bucket);
			node.getChildren().add(view);
		}
	}

	/** Índice del mayor valor del array, sin contar la posición "skip". */
	private static int argMax(double[] values, int skip) {
		int best = skip == 0 ? 1 : 0;
		for (int i = 0; i < values.length; i++)
			if (i != skip && values[i] > values[best])
				best = i;
		return best;
	}

	/** Suma acumulada de las áreas: cumulative[t] = área de los triángulos 0..t. */
	private static double[] cumulativeAreas(ObjMesh m) {
		double[] acc = new double[m.triangleCount()];
		double sum = 0;
		for (int t = 0; t < acc.length; t++) {
			int a = 3 * m.triangles[3 * t], b = 3 * m.triangles[3 * t + 1], c = 3 * m.triangles[3 * t + 2];
			double[] v = m.vertices;
			// Área = la mitad del módulo del producto vectorial de dos lados
			double ux = v[b] - v[a], uy = v[b + 1] - v[a + 1], uz = v[b + 2] - v[a + 2];
			double wx = v[c] - v[a], wy = v[c + 1] - v[a + 1], wz = v[c + 2] - v[a + 2];
			double cx = uy * wz - uz * wy, cy = uz * wx - ux * wz, cz = ux * wy - uy * wx;
			sum += 0.5 * Math.sqrt(cx * cx + cy * cy + cz * cz);
			acc[t] = sum;
		}
		return acc;
	}

	/** Malla con los tetraedros de una lista de esquinas (4 esquinas x 3 coordenadas por punto). */
	private static TriangleMesh tetraMesh(List<Float> corners) {
		TriangleMesh mesh = new TriangleMesh();
		float[] pts = new float[corners.size()];
		for (int i = 0; i < pts.length; i++)
			pts[i] = corners.get(i);
		int n = pts.length / 12; // número de puntos (tetraedros)
		int[] faces = new int[n * 4 * 6];
		int f = 0;
		for (int i = 0; i < n; i++)
			for (int[] tf : TETRA_FACES) {
				// (vértice, textura) x 3; la textura siempre es la 0
				faces[f] = 4 * i + tf[0];
				faces[f + 2] = 4 * i + tf[1];
				faces[f + 4] = 4 * i + tf[2];
				f += 6;
			}
		mesh.getPoints().addAll(pts);
		mesh.getTexCoords().addAll(0, 0);
		mesh.getFaces().addAll(faces);
		return mesh;
	}

	/**
	 * Pone la nube en la postura actual: para cada cubo, su matriz es la media
	 * ponderada de las matrices de skinning de sus dos huesos.
	 *
	 * @param skin matriz de skinning de cada segmento, en el orden de
	 *             MakeHumanRig.SEGMENTS (12 doubles por filas)
	 */
	public void update(double[][] skin) {
		for (Bucket b : buckets) {
			double wa = (double) b.level() / LEVELS, wb = 1 - wa;
			double[] sa = skin[b.a()], sb = skin[b.b()];
			double[] mm = new double[12];
			for (int k = 0; k < 12; k++)
				mm[k] = wa * sa[k] + wb * sb[k];
			b.affine().setToTransform(mm[0], mm[1], mm[2], mm[3], mm[4], mm[5], mm[6], mm[7], mm[8], mm[9],
					mm[10], mm[11]);
		}
	}

	/** Nodo de JavaFX con toda la nube (para añadirlo a la escena). */
	public Group getNode() {
		return node;
	}

	public int getCount() {
		return count;
	}

	/** Número de cubos (mallas) en que se ha repartido la nube. */
	public int getBucketCount() {
		return buckets.size();
	}
}

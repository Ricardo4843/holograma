package holograma.body;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

/**
 * Malla 3D leída de un fichero Wavefront .obj (el formato que exporta
 * MakeHuman).
 *
 * <h2>El formato .obj</h2>
 * Es texto plano, una cosa por línea:
 *
 * <pre>
 * v  -3.58 171.34 15.38        vértice: coordenadas x y z
 * vt 0.51 0.73                 coordenada de textura (aquí no se usa)
 * g  base.obj                  empieza un grupo (una parte del modelo)
 * f  4849/1 1/2 2/3 4848/4     cara: índices de sus vértices (/textura)
 * </pre>
 *
 * Detalles:
 * - Los índices de las caras empiezan en 1, no en 0 como en Java.
 * - Las caras de MakeHuman son cuadriláteros. Se parten en 2 triángulos, que
 *   es con lo que trabaja todo lo demás.
 *
 * <h2>Cambio de ejes</h2>
 * MakeHuman usa Y hacia arriba, Z hacia delante y X hacia la IZQUIERDA del
 * personaje. Este proyecto usa X = derecha, Y = delante, Z = arriba. La
 * conversión es (x, y, z) -> (-x, z, y). Es una rotación válida (no un
 * reflejo), así que no cambia el sentido de giro de los triángulos.
 */
public class ObjMesh {

	/** Coordenadas de los vértices ya convertidas: x0, y0, z0, x1, y1, z1... */
	public final double[] vertices;
	/** Triángulos: 3 índices de vértice cada uno (empezando en 0). */
	public final int[] triangles;
	/** Nombre del grupo .obj al que pertenece cada triángulo ("base.obj", "high-poly.obj"...). */
	public final String[] triangleGroup;

	private ObjMesh(double[] vertices, int[] triangles, String[] triangleGroup) {
		this.vertices = vertices;
		this.triangles = triangles;
		this.triangleGroup = triangleGroup;
	}

	public int vertexCount() {
		return vertices.length / 3;
	}

	public int triangleCount() {
		return triangles.length / 3;
	}

	/** Lee el fichero línea a línea. Lanza IOException si no existe o no se puede leer. */
	public static ObjMesh load(Path file) throws IOException {
		List<Double> v = new ArrayList<>();
		List<Integer> t = new ArrayList<>();
		List<String> groups = new ArrayList<>();
		String group = "";

		for (String line : Files.readAllLines(file)) {
			// split("\\s+") parte por uno o más espacios/tabuladores
			String[] p = line.trim().split("\\s+");
			switch (p[0]) {
			case "v":
				double x = Double.parseDouble(p[1]), y = Double.parseDouble(p[2]), z = Double.parseDouble(p[3]);
				v.add(-x); // cambio de ejes (ver javadoc)
				v.add(z);
				v.add(y);
				break;
			case "g":
				group = p.length > 1 ? p[1] : "";
				break;
			case "f":
				// Índice de cada esquina: lo que hay antes de la primera '/', menos 1
				int[] idx = new int[p.length - 1];
				for (int i = 1; i < p.length; i++)
					idx[i - 1] = Integer.parseInt(p[i].split("/")[0]) - 1;
				// Triangulación "en abanico": un polígono de n esquinas a, b, c, d...
				// se parte en (a,b,c), (a,c,d)... Para un cuadrilátero, 2 triángulos.
				for (int i = 1; i + 1 < idx.length; i++) {
					t.add(idx[0]);
					t.add(idx[i]);
					t.add(idx[i + 1]);
					groups.add(group);
				}
				break;
			default:
				// Comentarios (#), texturas (vt), materiales... no hacen falta
			}
		}
		// De listas (cómodas para ir añadiendo) a arrays (rápidos y compactos)
		double[] verts = new double[v.size()];
		for (int i = 0; i < verts.length; i++)
			verts[i] = v.get(i);
		int[] tris = t.stream().mapToInt(Integer::intValue).toArray();
		return new ObjMesh(verts, tris, groups.toArray(new String[0]));
	}
}

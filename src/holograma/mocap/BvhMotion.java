package holograma.mocap;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import holograma.kinematics.Matrix4;

/**
 * Animación de captura de movimiento leída de un fichero BVH (Biovision
 * Hierarchy), el formato más usado para compartir datos de mocap.
 *
 * <h2>Cómo es un .bvh</h2>
 * Tiene dos partes:
 *
 * <pre>
 * HIERARCHY
 * ROOT Hips
 * {
 *   OFFSET 0 0 0
 *   CHANNELS 6 Xposition Yposition Zposition Zrotation Yrotation Xrotation
 *   JOINT LeftUpLeg
 *   {
 *     OFFSET 1.65 -1.80 0.62
 *     CHANNELS 3 Zrotation Yrotation Xrotation
 *     ...
 *     End Site
 *     {
 *       OFFSET 0 -0.5 2.1
 *     }
 *   }
 *   ...
 * }
 * MOTION
 * Frames: 344
 * Frame Time: 0.0083333
 * 9.37 17.82 -17.36 -3.29 -5.58 -6.82 0 0 0 ... (una línea por fotograma)
 * </pre>
 *
 * 1. HIERARCHY: el esqueleto de la persona grabada, como un ÁRBOL. Cada JOINT
 * tiene un OFFSET (dónde está respecto a su padre), unos CHANNELS (qué valores
 * trae cada fotograma para él: giros y, en la raíz, también la posición) y sus
 * hijos entre llaves. "End Site" marca el final de una rama (la punta de un
 * pie, la coronilla...): solo tiene OFFSET.
 *
 * 2. MOTION: una línea de números por fotograma. Cada línea trae los valores
 * de todos los canales, en el orden en que aparecen en la jerarquía.
 *
 * <h2>Es otra vez el lab2</h2>
 * La jerarquía es un árbol recursivo (un JOINT contiene JOINTs), así que se
 * lee con un parser por DESCENSO RECURSIVO, como el de JSON (JsonParser). Y
 * para saber dónde está cada articulación en un fotograma se usa la misma
 * CINEMÁTICA DIRECTA recursiva que con nuestro esqueleto: la matriz de cada
 * joint es la de su padre por su transformación local.
 *
 * El esqueleto del .bvh NO es el nuestro: tiene otros huesos, otras
 * proporciones y otra postura de reposo. Pasar el movimiento de uno a otro lo
 * hace la clase Retargeter.
 */
public class BvhMotion {

	/**
	 * Un nodo de la jerarquía. Tipo recursivo, como Segment: tiene una lista de
	 * hijos de su mismo tipo. Los "End Site" también se guardan como Joint (sin
	 * canales), con el nombre del padre seguido de "_End".
	 */
	public static final class Joint {
		final String name;
		final int index; // posición en la lista "joints" (orden de lectura = preorden)
		final double[] offset = new double[3];
		String[] channels = new String[0]; // "Xposition", "Zrotation"...
		int firstChannel; // dónde empiezan sus valores dentro de cada línea de MOTION
		final List<Joint> children = new ArrayList<>();
		Joint parent;

		Joint(String name, int index) {
			this.name = name;
			this.index = index;
		}

		public String getName() {
			return name;
		}

		public int getIndex() {
			return index;
		}

		public List<Joint> getChildren() {
			return children;
		}

		public Joint getParent() {
			return parent;
		}

		@Override
		public String toString() {
			return name;
		}
	}

	private final String name;
	private final Joint root;
	private final List<Joint> joints = new ArrayList<>(); // todos, en preorden
	private int channelCount; // canales totales = números por fotograma
	private double frameTime; // segundos entre fotogramas
	private double[][] frames; // frames[f][c] = valor del canal c en el fotograma f

	// ---- Estado del parser (solo se usa mientras se lee) ----
	private String[] tokens;
	private int pos;

	/** Lee un fichero .bvh. */
	public static BvhMotion load(Path file) throws IOException {
		String fileName = file.getFileName().toString();
		return new BvhMotion(fileName.replaceFirst("\\.[bB][vV][hH]$", ""), Files.readString(file));
	}

	/**
	 * Lee el texto de un .bvh.
	 *
	 * Primero se trocea en "tokens" (palabras separadas por espacios, tabuladores
	 * o saltos de línea). En BVH todo está separado por espacios, incluidas las
	 * llaves, así que con un split basta y el parser trabaja palabra a palabra.
	 */
	public BvhMotion(String name, String text) {
		this.name = name;
		// "\\s+" = uno o más espacios en blanco de cualquier tipo
		tokens = text.trim().split("\\s+");
		expect("HIERARCHY");
		expect("ROOT");
		root = parseJoint(null);
		parseMotion();
		tokens = null; // ya no hace falta: que lo libere el recolector de basura
	}

	// ================================================================ parser

	/**
	 * joint := nombre '{' OFFSET x y z [CHANNELS n canal*n] { JOINT joint | End Site endSite } '}'
	 *
	 * Método recursivo: al encontrar "JOINT" dentro de las llaves se llama a sí
	 * mismo para leer el hijo. Caso base: un joint cuyas llaves solo contienen
	 * End Site (o nada más).
	 */
	private Joint parseJoint(Joint parent) {
		Joint j = new Joint(next(), joints.size());
		register(j, parent);
		expect("{");
		parseOffset(j);
		if (peek().equals("CHANNELS")) {
			pos++;
			int n = Integer.parseInt(next());
			j.channels = new String[n];
			j.firstChannel = channelCount;
			for (int i = 0; i < n; i++)
				j.channels[i] = next();
			channelCount += n;
		}
		// Hijos hasta encontrar la llave que cierra este joint
		while (!peek().equals("}")) {
			String word = next();
			if (word.equals("JOINT"))
				parseJoint(j); // caso recursivo
			else if (word.equals("End")) {
				expect("Site");
				Joint end = new Joint(j.name + "_End", joints.size());
				register(end, j);
				expect("{");
				parseOffset(end);
				expect("}");
			} else
				throw error("se esperaba JOINT, End Site o }, y hay '" + word + "'");
		}
		expect("}");
		return j;
	}

	/** Añade el joint a la lista plana y lo cuelga de su padre (enlace doble, como Segment). */
	private void register(Joint j, Joint parent) {
		joints.add(j);
		j.parent = parent;
		if (parent != null)
			parent.children.add(j);
	}

	private void parseOffset(Joint j) {
		expect("OFFSET");
		for (int i = 0; i < 3; i++)
			j.offset[i] = number();
	}

	/** MOTION Frames: n Frame Time: t, y luego n líneas de números. */
	private void parseMotion() {
		expect("MOTION");
		expect("Frames:");
		int n = (int) number();
		expect("Frame");
		expect("Time:");
		frameTime = number();
		// Algunos ficheros traen 0 o un valor absurdo: en ese caso, 30 fotogramas/s
		if (!(frameTime > 0 && frameTime < 1))
			frameTime = 1 / 30.0;
		frames = new double[n][channelCount];
		for (int f = 0; f < n; f++)
			for (int c = 0; c < channelCount; c++) {
				if (pos >= tokens.length)
					throw error("el fichero acaba en el fotograma " + f + " de " + n);
				frames[f][c] = number();
			}
	}

	private String peek() {
		if (pos >= tokens.length)
			throw error("el fichero se acaba antes de tiempo");
		return tokens[pos];
	}

	private String next() {
		String t = peek();
		pos++;
		return t;
	}

	private double number() {
		String t = next();
		try {
			return Double.parseDouble(t);
		} catch (NumberFormatException e) {
			throw error("se esperaba un número y hay '" + t + "'");
		}
	}

	private void expect(String word) {
		String t = next();
		if (!t.equals(word))
			throw error("se esperaba '" + word + "' y hay '" + t + "'");
	}

	private IllegalArgumentException error(String msg) {
		return new IllegalArgumentException("BVH no válido (palabra " + pos + "): " + msg);
	}

	// ================================================================ cinemática

	/**
	 * Cinemática directa del esqueleto del .bvh en un fotograma: devuelve la
	 * matriz (posición + orientación en el mundo) de cada joint, en un array
	 * indexado por Joint.index.
	 *
	 * Mismo esquema que ForwardKinematics3D: método "fachada" + método privado
	 * recursivo que pasa la matriz del padre a los hijos.
	 */
	public Matrix4[] pose(int frame) {
		Matrix4[] world = new Matrix4[joints.size()];
		pose(root, Matrix4.identity(), frames[frame], world);
		return world;
	}

	/**
	 * Transformación local de un joint en BVH:
	 * local = traslación(OFFSET) * [traslación(canales de posición)] * giros
	 *
	 * Los giros se aplican en el orden en que aparecen en CHANNELS (por ejemplo
	 * "Zrotation Yrotation Xrotation" = rotZ * rotY * rotX). Cada fichero puede
	 * traer un orden distinto, por eso se lee el nombre de cada canal. Los
	 * ángulos vienen en grados.
	 */
	private static void pose(Joint j, Matrix4 parentWorld, double[] values, Matrix4[] world) {
		Matrix4 local = Matrix4.translation(j.offset[0], j.offset[1], j.offset[2]);
		for (int i = 0; i < j.channels.length; i++) {
			double v = values[j.firstChannel + i];
			double rad = Math.toRadians(v);
			// switch sobre String (Java 7+); la forma con "->" (Java 14+) no
			// necesita break
			Matrix4 t = switch (j.channels[i].toLowerCase()) {
			case "xposition" -> Matrix4.translation(v, 0, 0);
			case "yposition" -> Matrix4.translation(0, v, 0);
			case "zposition" -> Matrix4.translation(0, 0, v);
			case "xrotation" -> Matrix4.rotX(rad);
			case "yrotation" -> Matrix4.rotY(rad);
			case "zrotation" -> Matrix4.rotZ(rad);
			default -> throw new IllegalArgumentException("Canal BVH desconocido: " + j.channels[i]);
			};
			local = local.multiply(t);
		}
		Matrix4 w = parentWorld.multiply(local);
		world[j.index] = w;
		// Caso base implícito: los End Site no tienen hijos
		for (Joint c : j.children)
			pose(c, w, values, world);
	}

	// ================================================================ getters

	public String getName() {
		return name;
	}

	public Joint getRoot() {
		return root;
	}

	public List<Joint> getJoints() {
		return joints;
	}

	public int getFrameCount() {
		return frames.length;
	}

	public double getFrameTime() {
		return frameTime;
	}

	/** Duración total en segundos. */
	public double getDuration() {
		return frames.length * frameTime;
	}
}

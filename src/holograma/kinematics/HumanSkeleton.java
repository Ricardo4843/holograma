package holograma.kinematics;

import java.util.HashMap;
import java.util.IdentityHashMap;
import java.util.Map;

// "import static": permite escribir sub(a, b) en vez de Vec3.sub(a, b)
import static holograma.kinematics.Vec3.*;

/**
 * Esqueleto humano de cuerpo completo (21 segmentos) con límites articulares
 * aproximados. Sustituye al fichero de texto que se leía en el lab2.
 *
 * <h2>Se construye a partir de las ARTICULACIONES</h2>
 * En vez de escribir a mano la longitud y la orientación de cada segmento, se
 * da la posición 3D de cada articulación en la postura de reposo (hombro,
 * codo, muñeca...) y el código deduce:
 * <ul>
 * <li>Longitud de cada segmento = distancia entre sus dos articulaciones.</li>
 * <li>Rotación base = la que hace que su eje Z apunte de una articulación a
 * la siguiente.</li>
 * </ul>
 * Así el mismo código sirve para dos casos:
 * <ul>
 * <li>{@link #defaultJoints()}: un cuerpo genérico de 1,75 m con los brazos
 * pegados al cuerpo (lo que se usa si no hay modelo 3D).</li>
 * <li>Articulaciones detectadas en el modelo de MakeHuman (MakeHumanRig): el
 * esqueleto encaja exactamente dentro de ese cuerpo, sea cual sea su altura o
 * complexión, y con su postura de reposo (brazos en "A").</li>
 * </ul>
 *
 * <h2>Ejes del mundo</h2>
 * X = lateral (hacia la derecha del sujeto), Y = hacia delante, Z = arriba.
 * El suelo es z = 0.
 *
 * <h2>Árbol</h2>
 *
 * <pre>
 * Pelvis (raíz, longitud 0)
 * ├── Lumbar ── Tórax ──┬── Cuello ── Cabeza
 * │                     ├── Clavícula D ── Brazo D ── Antebrazo D ── Mano D
 * │                     └── Clavícula I ── Brazo I ── Antebrazo I ── Mano I
 * ├── Cadera D ── Muslo D ── Tibia D ── Pie D
 * └── Cadera I ── Muslo I ── Tibia I ── Pie I
 * </pre>
 *
 * Es un árbol (y no una simple cadena como un brazo robótico) porque el
 * cuerpo se ramifica: de la pelvis salen el tronco y las dos piernas, y del
 * tórax salen el cuello y los dos brazos.
 */
public class HumanSkeleton {

	private final Segment root;
	private final double[] origin; // posición de la pelvis (articulación raíz)
	// Ángulo (grados) que separa cada brazo de la vertical en reposo. En la pose
	// en "A" de MakeHuman son unos 45º; la animación de caminar lo usa para
	// bajar los brazos.
	private final double armRestAngle;

	private HumanSkeleton(Segment root, double[] origin, double armRestAngle) {
		this.root = root;
		this.origin = origin;
		this.armRestAngle = armRestAngle;
	}

	public Segment getRoot() {
		return root;
	}

	/** Origen para la cinemática directa: dónde está la pelvis en el mundo. */
	public double[] getOrigin() {
		return origin.clone();
	}

	public double getArmRestAngle() {
		return armRestAngle;
	}

	/**
	 * Articulaciones de un cuerpo genérico de ~1,75 m, de pie y con los brazos
	 * rectos hacia abajo. Los nombres terminados en _D/_I son derecho/izquierdo.
	 */
	public static Map<String, double[]> defaultJoints() {
		Map<String, double[]> j = new HashMap<>();
		j.put("pelvis", new double[] { 0, 0, 95 });
		j.put("lumbar_end", new double[] { 0, 0, 115 });
		j.put("neck_base", new double[] { 0, 0, 143 });
		j.put("head_base", new double[] { 0, 0, 153 });
		j.put("head_top", new double[] { 0, 0, 176 });
		for (int side : new int[] { 1, -1 }) {
			String s = side == 1 ? "_D" : "_I";
			// side = +1 a la derecha (x positiva) y -1 a la izquierda
			j.put("shoulder" + s, new double[] { side * 21, 0, 143 });
			j.put("elbow" + s, new double[] { side * 21, 0, 113 });
			j.put("wrist" + s, new double[] { side * 21, 0, 86 });
			j.put("hand_end" + s, new double[] { side * 21, 0, 68 });
			j.put("hip" + s, new double[] { side * 9, 0, 95 });
			j.put("knee" + s, new double[] { side * 9, 0, 50 });
			j.put("ankle" + s, new double[] { side * 9, 0, 7 });
			j.put("foot_end" + s, new double[] { side * 9, 20, 7 });
		}
		return j;
	}

	/**
	 * Construye el esqueleto a partir de las posiciones de las articulaciones.
	 * Cada línea dice: segmento, padre, articulación donde empieza y
	 * articulación donde acaba. Después vienen sus ejes libres (.joint), con
	 * el patrón fluent de Segment.
	 * Ángulos en grados: (eje, nombre, mínimo, máximo).
	 */
	public static HumanSkeleton fromJoints(Map<String, double[]> joints) {
		Builder b = new Builder(joints);

		// ---------------- Tronco ----------------
		// Raíz: longitud 0 (empieza y acaba en la pelvis). Es el punto de unión
		// del tronco y las piernas. Su eje Z (Giro) rota todo el cuerpo.
		Segment pelvis = b.add(null, "Pelvis", "pelvis", "pelvis", false)
				.joint(0, "Inclinación", -30, 30)
				.joint(1, "Lateral", -30, 30)
				.joint(2, "Giro", -180, 180);
		Segment lumbar = b.add(pelvis, "Lumbar", "pelvis", "lumbar_end", false)
				.joint(0, "Flexión", -20, 45)
				.joint(1, "Lateral", -25, 25)
				.joint(2, "Giro", -30, 30);
		Segment chest = b.add(lumbar, "Tórax", "lumbar_end", "neck_base", false)
				.joint(0, "Flexión", -15, 30)
				.joint(1, "Lateral", -15, 15)
				.joint(2, "Giro", -30, 30);
		Segment neck = b.add(chest, "Cuello", "neck_base", "head_base", false)
				.joint(0, "Flexión", -40, 50)
				.joint(1, "Lateral", -35, 35)
				.joint(2, "Giro", -70, 70);
		// Sin .joint(1, ...): la cabeza no tiene inclinación lateral propia (eje
		// Y bloqueado; esa inclinación la hace el cuello)
		b.add(neck, "Cabeza", "head_base", "head_top", false)
				.joint(0, "Flexión", -20, 20)
				.joint(2, "Giro", -20, 20);

		// ---------------- Extremidades ----------------
		// Un bucle para los dos lados: side = 1 (derecho) y -1 (izquierdo). Los
		// límites de abducción se reflejan como en un espejo: en los dos lados
		// el ángulo positivo es "hacia la derecha", así que separar la pierna
		// derecha es positivo y separar la izquierda es negativo.
		for (int side : new int[] { 1, -1 }) {
			String n = side == 1 ? " D" : " I"; // sufijo del nombre del segmento
			String j = side == 1 ? "_D" : "_I"; // sufijo del nombre de la articulación
			boolean right = side == 1;

			// --- Pierna ---
			// Cadera: segmento "de unión" de la pelvis a la articulación de la
			// cadera. Sin ejes libres (los movimientos de cadera los
			// hace el muslo). true = segmento lateral (ver Builder.add).
			Segment hip = b.add(pelvis, "Cadera" + n, "pelvis", "hip" + j, true);
			Segment thigh = b.add(hip, "Muslo" + n, "hip" + j, "knee" + j, false)
					.joint(0, "Flexión", -30, 120)
					.joint(1, "Abducción", right ? -20 : -45, right ? 45 : 20)
					.joint(2, "Rotación", -40, 40);
			// La rodilla es una bisagra: solo el eje X, y solo hacia atrás
			// (negativo). Los límites son anatómicos (0 = pierna recta) y se
			// corrigen con lo que ya viene doblada en reposo (ver restFlexion).
			Segment shin = b.add(thigh, "Tibia" + n, "knee" + j, "ankle" + j, false);
			double knee0 = restFlexion(shin);
			shin.joint(0, "Flexión", -140 - knee0, 0 - knee0);
			b.add(shin, "Pie" + n, "ankle" + j, "foot_end" + j, false)
					.joint(0, "Flexión", -30, 45)
					.joint(1, "Inversión", -20, 20);

			// --- Brazo ---
			Segment clavicle = b.add(chest, "Clavícula" + n, "neck_base", "shoulder" + j, true)
					.joint(0, "Elevación", -10, 30);
			// El hombro es la articulación con más rango del cuerpo. La abducción
			// permite bajar el brazo 60º desde la pose en "A" hasta pegarlo al cuerpo.
			Segment arm = b.add(clavicle, "Brazo" + n, "shoulder" + j, "elbow" + j, false)
					.joint(0, "Flexión", -60, 180)
					.joint(1, "Abducción", right ? -60 : -150, right ? 150 : 60)
					.joint(2, "Rotación", -90, 90);
			// Codo: flexión (X) y pronación/supinación del antebrazo (Z, girar la
			// muñeca alrededor del propio antebrazo). Igual que en la rodilla, los
			// límites anatómicos se corrigen con la flexión de reposo: en la pose
			// en "A" de MakeHuman el codo ya está doblado unos 40º.
			Segment forearm = b.add(arm, "Antebrazo" + n, "elbow" + j, "wrist" + j, false);
			double elbow0 = restFlexion(forearm);
			forearm.joint(0, "Flexión", -10 - elbow0, 145 - elbow0)
					.joint(2, "Pronación", -80, 80);
			b.add(forearm, "Mano" + n, "wrist" + j, "hand_end" + j, false)
					.joint(0, "Flexión", -70, 80)
					.joint(1, "Desviación", -30, 30);
		}

		// Ángulo del brazo derecho con la vertical: arcocoseno de la componente
		// vertical de su dirección (producto escalar con el vector "abajo")
		double[] d = Vec3.normalize(Vec3.sub(joints.get("elbow_D"), joints.get("shoulder_D")));
		double armAngle = Math.toDegrees(Math.acos(-d[2]));
		return new HumanSkeleton(pelvis, joints.get("pelvis"), armAngle);
	}

	/**
	 * Cuánto está ya doblada una articulación en la postura de reposo (grados,
	 * sobre el eje X). Es el giro X de su rotación base (Matrix4.eulerXYZ).
	 *
	 * Hace falta porque los ángulos de Segment son RELATIVOS al reposo (0 =
	 * como venga el modelo), mientras que los límites anatómicos se miden desde
	 * la extremidad recta. Si el codo ya viene doblado 40º, el límite "-10º
	 * desde recto" es "-50º desde el reposo": límite relativo = anatómico -
	 * reposo. Sin esta corrección, el brazo nunca podría estirarse del todo
	 * (se notaba al reproducir capturas de movimiento).
	 */
	private static double restFlexion(Segment s) {
		return Math.toDegrees(s.getBase().eulerXYZ()[0]);
	}

	/**
	 * Clase auxiliar que crea cada segmento a partir de dos articulaciones.
	 * "static" = no necesita una instancia de HumanSkeleton; "private" = solo
	 * se usa aquí dentro.
	 */
	private static final class Builder {
		private final Map<String, double[]> joints;
		// Rotación de reposo de cada segmento en coordenadas del MUNDO. Hace falta
		// para calcular la rotación base de sus hijos (que es relativa al padre).
		private final Map<Segment, Matrix4> worldRotation = new IdentityHashMap<>();

		Builder(Map<String, double[]> joints) {
			this.joints = joints;
		}

		/**
		 * Crea un segmento que va de la articulación "from" a la "to".
		 *
		 * Pasos:
		 * 1. Dirección d = to - from; longitud = |d|.
		 * 2. Ejes del segmento en el mundo: Z = d normalizado (hacia dónde apunta).
		 *    X = el eje lateral del mundo, "enderezado" para que sea perpendicular
		 *    a Z (método de Gram-Schmidt). Y = Z x X (producto vectorial), que es
		 *    perpendicular a los dos. Así, en piernas y brazos, el X local siempre
		 *    es el eje lateral y girar sobre él es flexión hacia delante.
		 * 3. Rotación base = Rotación_padre^-1 * Rotación_mundo. Es decir, la
		 *    rotación del hijo vista desde el padre.
		 *
		 * @param lateral Para segmentos que apuntan hacia el lado (clavícula,
		 *                cadera) el X del mundo es casi paralelo a Z y
		 *                Gram-Schmidt fallaría (dividiría entre casi 0). Para
		 *                ellos se usa el eje Y del mundo como referencia, con el
		 *                signo elegido para que "Elevación" positiva suba el
		 *                hombro en los dos lados.
		 */
		Segment add(Segment parent, String name, String from, String to, boolean lateral) {
			double[] a = joints.get(from), b = joints.get(to);
			double[] d = sub(b, a);
			double length = norm(d);

			Matrix4 rotation;
			if (length < 1e-9) {
				// Segmento de longitud 0 (la pelvis): no tiene dirección, se deja
				// alineado con el mundo
				rotation = Matrix4.identity();
			} else {
				double[] z = scale(d, 1 / length);
				double[] ref = lateral ? new double[] { 0, -Math.signum(d[0]), 0 } : new double[] { 1, 0, 0 };
				// Gram-Schmidt: quitar a ref su componente en la dirección z
				double[] x = normalize(sub(ref, scale(z, dot(ref, z))));
				double[] y = cross(z, x);
				rotation = Matrix4.fromAxes(x, y, z);
			}
			// La inversa de una rotación pura es su transpuesta (ver Matrix4.rigidInverse)
			Matrix4 base = parent == null ? rotation : worldRotation.get(parent).rigidInverse().multiply(rotation);

			Segment s = new Segment(name, length, base);
			worldRotation.put(s, rotation);
			if (parent != null)
				parent.addChild(s);
			return s;
		}
	}
}

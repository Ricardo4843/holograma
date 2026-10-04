package holograma.mocap;

import static holograma.kinematics.Vec3.*;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;

import holograma.kinematics.ForwardKinematics3D;
import holograma.kinematics.HumanSkeleton;
import holograma.kinematics.Matrix4;
import holograma.kinematics.Segment;

/**
 * "Retargeting": pasa el movimiento de un esqueleto (el del .bvh) a otro
 * distinto (el nuestro, de 21 segmentos).
 *
 * <h2>Por qué no basta con copiar los ángulos</h2>
 * Los dos esqueletos son distintos en casi todo:
 * <ul>
 * <li>Huesos: el de CMU tiene 31 (con dedos, dos vértebras de cuello...) y el
 * nuestro 21.</li>
 * <li>Postura de reposo: los .bvh suelen estar en "T" (brazos en cruz) y el
 * nuestro en "A" (brazos a 45º).</li>
 * <li>Ejes: cada fichero usa los suyos (Y arriba casi siempre; nosotros Z
 * arriba) y su propio orden de giros.</li>
 * <li>Proporciones: la persona grabada no mide lo mismo que el modelo.</li>
 * </ul>
 * Si se copiasen los ángulos tal cual, un ángulo 0 del .bvh (brazo en
 * cruz) dejaría nuestro brazo a 45º, y todo saldría torcido.
 *
 * <h2>La idea: copiar DIRECCIONES, no ángulos</h2>
 * Para cada uno de nuestros segmentos se busca qué articulaciones del .bvh
 * hacen de inicio y fin (por ejemplo, el muslo va de "RightUpLeg" a
 * "RightLeg"). En cada fotograma:
 * <ol>
 * <li>Cinemática directa del .bvh (BvhMotion.pose): dónde está cada una de
 * sus articulaciones.</li>
 * <li>Para cada segmento nuestro, la orientación que DEBERÍA tener en el
 * mundo: su eje Z (hacia dónde apunta) = dirección inicio -> fin en el .bvh.
 * Su eje X (el giro sobre sí mismo) se elige con reglas que se explican en
 * cada caso.</li>
 * <li>Recorriendo nuestro árbol desde la pelvis (RECURSIVO, como la
 * cinemática directa), se pasa esa orientación del mundo a ángulos LOCALES:
 * local = (orientación del padre * base)^-1 * deseada. Y de la matriz local se
 * sacan los tres ángulos de Euler (Matrix4.eulerXYZ).</li>
 * </ol>
 * Como solo se copian direcciones, da igual cuánto midan los huesos del .bvh
 * o cuál sea su postura de reposo.
 *
 * <h2>Límites articulares</h2>
 * Los ángulos pasan por Segment.setAngle, que los recorta a los límites. Si
 * el .bvh pide algo imposible para nuestro esqueleto (una rodilla con giro
 * lateral, por ejemplo), se queda en lo más parecido que sí puede hacer. Por
 * eso cada hijo se calcula a partir de la orientación REAL de su padre (ya
 * recortada) y no de la deseada: así los errores no se acumulan.
 */
public class Retargeter {

	// ---- Nombres de las articulaciones en los .bvh ----
	// Cada programa los llama de una forma: "RightUpLeg" (CMU, Mixamo),
	// "UpperLeg_R" (Bandai Namco), "rThigh" (Poser)... Se prueban varios
	// patrones. %1$s se sustituye por "right"/"left" y %2$s por "r"/"l". Los
	// nombres se comparan en minúsculas y sin símbolos (ver cleanName).
	private static final Map<String, String[]> PATTERNS = new HashMap<>();
	static {
		// Bloque "static": se ejecuta una sola vez, al cargar la clase
		PATTERNS.put("hips", new String[] { "hips", "hip", "pelvis" });
		PATTERNS.put("neck", new String[] { "neck", "neck1", "lowerneck" });
		PATTERNS.put("head", new String[] { "head" });
		PATTERNS.put("upleg", new String[] { "%1$supleg", "%1$supperleg", "%1$sthigh", "upperleg%2$s", "thigh%2$s",
				"upleg%2$s", "%2$sthigh", "%2$supleg", "%1$ship", "hip%2$s" });
		PATTERNS.put("knee", new String[] { "%1$sleg", "%1$slowerleg", "%1$sshin", "%1$sknee", "lowerleg%2$s",
				"shin%2$s", "knee%2$s", "leg%2$s", "%2$sshin", "%2$sknee", "%2$sleg" });
		PATTERNS.put("ankle", new String[] { "%1$sfoot", "%1$sankle", "foot%2$s", "ankle%2$s", "%2$sfoot",
				"%2$sankle" });
		PATTERNS.put("toe", new String[] { "%1$stoebase", "%1$stoe", "%1$stoes", "toes%2$s", "toe%2$s",
				"toebase%2$s", "%2$stoe", "%2$stoes" });
		PATTERNS.put("clav", new String[] { "%1$sshoulder", "%1$scollar", "%1$sclavicle", "shoulder%2$s",
				"clavicle%2$s", "collar%2$s", "%2$scollar", "%2$sclavicle" });
		PATTERNS.put("shoulder", new String[] { "%1$sarm", "%1$supperarm", "upperarm%2$s", "arm%2$s",
				"%2$sshldr", "%2$supperarm", "%2$sarm" });
		PATTERNS.put("elbow", new String[] { "%1$sforearm", "%1$slowerarm", "%1$selbow", "lowerarm%2$s",
				"forearm%2$s", "elbow%2$s", "%2$sforearm", "%2$selbow" });
		PATTERNS.put("wrist", new String[] { "%1$shand", "%1$swrist", "hand%2$s", "wrist%2$s", "%2$shand",
				"%2$swrist" });
	}

	// Por debajo de este ángulo de flexión el plano de la rodilla/codo no es
	// fiable, y por encima del segundo se usa solo él (ver bendFrame)
	private static final double BEND_MIN = Math.sin(Math.toRadians(5));
	private static final double BEND_FULL = Math.sin(Math.toRadians(25));

	private final BvhMotion motion;
	private final Segment root;
	private final double[] origin; // pelvis de nuestro esqueleto en reposo
	// Articulación del .bvh para cada clave nuestra ("hips", "knee_D"...)
	private final Map<String, BvhMotion.Joint> joints = new HashMap<>();
	private final List<BvhMotion.Joint> spine; // camino de la cadera al cuello
	private final double lumbarFraction; // dónde acaba nuestra lumbar dentro de ese camino (0-1)
	private final Matrix4 toOurs; // giro que pasa vectores de los ejes del .bvh a los nuestros
	private final Matrix4[] firstPose; // cinemática del .bvh en el fotograma 0
	// Orientación de cada segmento en el fotograma 0 (ver twistReference)
	private final Map<Segment, Matrix4> firstRot = new IdentityHashMap<>();
	// Para la altura de la pelvis (ver apply y findFloor)
	private final double scale;
	private final double[] lift; // cuánto hay que subir o bajar la pelvis en cada fotograma
	private final Segment[] feet; // Pie D y Pie I
	// Corrección de la inclinación de cada pie (ver footPitchOffset)
	private final Map<String, Double> footOffset = new HashMap<>();

	/**
	 * Prepara el retargeting: busca las articulaciones, deduce los ejes del
	 * .bvh, calcula la escala y "calibra" con el primer fotograma.
	 *
	 * @throws IllegalArgumentException si al .bvh le falta alguna articulación
	 *                                  imprescindible.
	 */
	public Retargeter(BvhMotion motion, HumanSkeleton skeleton) {
		this.motion = motion;
		this.root = skeleton.getRoot();
		this.origin = skeleton.getOrigin();
		Map<String, Segment> byName = new HashMap<>();
		for (Segment s : root.flatten())
			byName.put(s.getName(), s);

		// 1) Buscar las articulaciones del .bvh
		find("hips", "", "");
		find("neck", "", "");
		find("head", "", "");
		for (String side : new String[] { "_D", "_I" }) {
			String longName = side.equals("_D") ? "right" : "left";
			String shortName = side.equals("_D") ? "r" : "l";
			for (String key : new String[] { "upleg", "knee", "ankle", "shoulder", "elbow", "wrist" })
				find(key + side, longName, shortName);
			// Opcionales: si no están se apaña sin ellos
			findOptional("toe" + side, longName, shortName);
			findOptional("clav" + side, longName, shortName);
		}
		spine = pathBetween(joints.get("hips"), joints.get("neck"));
		double lumbar = byName.get("Lumbar").getLength(), chest = byName.get("Tórax").getLength();
		lumbarFraction = lumbar / (lumbar + chest);

		// 2) Ejes del .bvh, deducidos de la postura del primer fotograma
		firstPose = motion.pose(0);
		toOurs = detectAxes(firstPose);

		// 3) Escala para la altura de la pelvis y corrección de los pies. Se
		// recorre la animación entera una vez (cinemática del .bvh en cada
		// fotograma) y se guarda lo que hace falta.
		int n = motion.getFrameCount();
		Map<String, double[]> ankleHeight = new HashMap<>(), footPitch = new HashMap<>();
		for (String side : new String[] { "_D", "_I" }) {
			ankleHeight.put(side, new double[n]);
			footPitch.put(side, new double[n]);
		}
		for (int f = 0; f < n; f++) {
			Frame fr = new Frame(motion.pose(f));
			for (String side : new String[] { "_D", "_I" }) {
				ankleHeight.get(side)[f] = fr.pos("ankle" + side)[2];
				footPitch.get(side)[f] = pitch(sub(fr.toe(side), fr.pos("ankle" + side)));
			}
		}
		Frame f0 = new Frame(firstPose);
		double bvhLeg = norm(sub(f0.pos("upleg_D"), f0.pos("knee_D"))) + norm(sub(f0.pos("knee_D"), f0.pos("ankle_D")));
		double ourLeg = byName.get("Muslo D").getLength() + byName.get("Tibia D").getLength();
		scale = ourLeg / bvhLeg;
		feet = new Segment[] { byName.get("Pie D"), byName.get("Pie I") };
		double[] foot = restFoot(feet[0]);
		for (String side : new String[] { "_D", "_I" })
			footOffset.put(side, foot[1] - footPitchOffset(ankleHeight.get(side), footPitch.get(side)));

		// 4) Calibración (ver twistReference). Se guardan y restauran los ángulos
		// para no cambiar la postura que hubiera en pantalla.
		Map<Segment, double[]> saved = saveAngles();
		place(root, Matrix4.identity(), f0, true);

		// 5) Contacto con el suelo (ver findFloor): se pone el esqueleto en
		// cada fotograma, con la pelvis a la altura que dice el .bvh, y se mira
		// dónde queda el punto más bajo de NUESTROS pies.
		double[] lowest = new double[n];
		for (int f = 0; f < n; f++) {
			Frame fr = new Frame(motion.pose(f));
			place(root, Matrix4.identity(), fr, false);
			lowest[f] = lowestFootPoint(rawHeight(fr));
		}
		double[] floor = findFloor(lowest, (int) Math.round(0.4 / motion.getFrameTime()));
		// Hay que mover la pelvis para que ese suelo quede a la altura que tienen
		// nuestros pies en reposo
		lift = new double[n];
		for (int f = 0; f < n; f++)
			lift[f] = foot[0] - floor[f];
		restoreAngles(saved);
	}

	/**
	 * Pone nuestro esqueleto en la postura del fotograma indicado.
	 *
	 * @return Dónde va la pelvis (origen para ForwardKinematics3D). En
	 *         horizontal se queda quieta en su sitio, como en una cinta de
	 *         correr: si se copiase el desplazamiento, al caminar el holograma
	 *         se saldría de la pantalla. En vertical sí se copia (escalada a
	 *         nuestras piernas), para que se note el rebote al correr o saltar.
	 */
	public double[] apply(int frame) {
		Frame f = new Frame(motion.pose(frame));
		place(root, Matrix4.identity(), f, false);
		return new double[] { origin[0], origin[1], rawHeight(f) + lift[frame] };
	}

	/** Altura de la cadera del .bvh, escalada a nuestras piernas (sin corregir el suelo). */
	private double rawHeight(Frame f) {
		return f.pos("hips")[2] * scale;
	}

	/**
	 * Altura del punto más bajo de nuestros pies (tobillos y puntas) con la
	 * postura actual y la pelvis a la altura indicada.
	 */
	private double lowestFootPoint(double pelvisHeight) {
		Map<Segment, Matrix4> frames = ForwardKinematics3D
				.frames(ForwardKinematics3D.computePositions(root, origin[0], origin[1], pelvisHeight));
		double low = Double.MAX_VALUE;
		for (Segment foot : feet) {
			Matrix4 ankle = frames.get(foot);
			double toe = ankle.transformPoint(new double[] { 0, 0, foot.getLength() })[2];
			low = Math.min(low, Math.min(ankle.tz(), toe));
		}
		return low;
	}

	/** Fotograma que toca a los t segundos, en bucle (al acabar vuelve a empezar). */
	public int frameAt(double seconds) {
		int n = motion.getFrameCount();
		int f = (int) Math.floor(seconds / motion.getFrameTime()) % n;
		return f < 0 ? f + n : f;
	}

	public BvhMotion getMotion() {
		return motion;
	}

	// ================================================================ recursión

	/**
	 * Coloca un segmento y, recursivamente, todo lo que cuelga de él.
	 *
	 * @param parentRot Orientación REAL del padre en el mundo (ya con sus
	 *                  ángulos recortados a los límites). Para la pelvis, la
	 *                  identidad.
	 * @param calibrate true solo en la calibración (fotograma 0).
	 */
	private void place(Segment s, Matrix4 parentRot, Frame f, boolean calibrate) {
		// Orientación que tendría el segmento con sus tres ángulos a 0
		Matrix4 neutral = parentRot.multiply(s.getBase());
		Matrix4 target = target(s, f, neutral, calibrate);
		if (target == null)
			s.resetAngles(); // segmento sin equivalente en el .bvh (cadera): se queda en reposo
		else {
			// target = neutral * rotX * rotY * rotZ  =>  rotX * rotY * rotZ = neutral^-1 * target
			double[] angles = neutral.rigidInverse().multiply(target).eulerXYZ();
			// setAngle recorta a los límites; en los ejes bloqueados (min = max =
			// 0) deja 0, así que no hace falta mirar cuáles están libres
			for (int i = 0; i < 3; i++)
				s.setAngle(i, angles[i]);
		}
		// Orientación real, con los ángulos ya recortados
		Matrix4 rot = parentRot.multiply(s.localTransform());
		if (calibrate)
			firstRot.put(s, rot);
		// Caso recursivo: los hijos parten de la orientación real de este.
		// Caso base implícito: una mano o un pie no tienen hijos.
		for (Segment c : s.getChildren())
			place(c, rot, f, calibrate);
	}

	/**
	 * Orientación que debería tener el segmento en el mundo (null si no hay de
	 * dónde sacarla). Para casi todos: eje Z = de una articulación a otra del
	 * .bvh, y eje X según la regla que mejor funciona en cada parte del cuerpo.
	 */
	private Matrix4 target(Segment s, Frame f, Matrix4 neutral, boolean calibrate) {
		String name = s.getName();
		// "Brazo D" -> base "Brazo" y lado "_D"
		String side = name.endsWith(" D") ? "_D" : name.endsWith(" I") ? "_I" : "";
		String base = side.isEmpty() ? name : name.substring(0, name.length() - 2);
		double[] neutralX = neutral.axis(0);

		switch (base) {
		case "Pelvis": {
			// La pelvis mide 0: no tiene dirección. Su eje X es la línea que une
			// las dos caderas, y su Z la vertical (enderezada para que sea
			// perpendicular a X). Así copia el giro y la caída lateral de la
			// cadera; la inclinación del tronco la hacen la lumbar y el tórax.
			double[] x = normalize(sub(f.pos("upleg_D"), f.pos("upleg_I")));
			double[] z = normalize(perpendicular(new double[] { 0, 0, 1 }, x));
			return Matrix4.fromAxes(x, cross(z, x), z);
		}
		case "Lumbar":
			// Eje X = línea de las caderas
			return frame(sub(f.spinePoint(lumbarFraction), f.pos("hips")),
					sub(f.pos("upleg_D"), f.pos("upleg_I")), neutralX);
		case "Tórax":
			// Eje X = línea de los hombros
			return frame(sub(f.pos("neck"), f.spinePoint(lumbarFraction)),
					sub(f.pos("shoulder_D"), f.pos("shoulder_I")), neutralX);
		case "Cuello":
			return frame(sub(f.pos("head"), f.pos("neck")), twistReference(s, "neck", f, neutralX, calibrate), neutralX);
		case "Cabeza":
			return frame(sub(f.endOf("head"), f.pos("head")), twistReference(s, "head", f, neutralX, calibrate),
					neutralX);
		case "Clavícula":
			// Aquí NO se copia la dirección: cada esqueleto pone el inicio de la
			// clavícula en un sitio (en CMU, en el centro del pecho y bastante
			// más abajo que el nuestro), y copiarla subiría el hombro de más. Se
			// copia solo cuánto ha GIRADO desde el fotograma 0, como en
			// twistReference.
			if (!joints.containsKey("clav" + side))
				return null;
			return calibrate ? neutral : f.delta("clav" + side).multiply(firstRot.get(s));
		case "Muslo":
			// La rodilla se dobla hacia atrás (+Y local del muslo)
			return bendFrame(f.pos("upleg" + side), f.pos("knee" + side), f.pos("ankle" + side), 1,
					twistReference(s, "upleg" + side, f, neutralX, calibrate), neutralX);
		case "Tibia":
			return frame(sub(f.pos("ankle" + side), f.pos("knee" + side)), neutralX, neutralX);
		case "Pie":
			// La dirección del pie se corrige con footOffset (ver footPitchOffset)
			return frame(tilt(sub(f.toe(side), f.pos("ankle" + side)), footOffset.get(side)),
					twistReference(s, "ankle" + side, f, neutralX, calibrate), neutralX);
		case "Brazo":
			// El codo se dobla hacia delante (-Y local del brazo)
			return bendFrame(f.pos("shoulder" + side), f.pos("elbow" + side), f.pos("wrist" + side), -1,
					twistReference(s, "shoulder" + side, f, neutralX, calibrate), neutralX);
		case "Antebrazo":
			return frame(sub(f.pos("wrist" + side), f.pos("elbow" + side)),
					twistReference(s, "elbow" + side, f, neutralX, calibrate), neutralX);
		case "Mano":
			return frame(sub(f.endOf("wrist" + side), f.pos("wrist" + side)),
					twistReference(s, "wrist" + side, f, neutralX, calibrate), neutralX);
		default: // Cadera D/I: segmento de unión sin ejes libres
			return null;
		}
	}

	/**
	 * Orientación con el eje Z en la dirección {@code dir} y el eje X lo más
	 * parecido posible a {@code xRef} (Gram-Schmidt, como en
	 * HumanSkeleton.Builder). Si xRef es paralelo a Z no sirve y se usa
	 * {@code fallbackX}.
	 */
	private static Matrix4 frame(double[] dir, double[] xRef, double[] fallbackX) {
		if (norm(dir) < 1e-9)
			return null; // dos articulaciones en el mismo sitio: sin dirección
		double[] z = normalize(dir);
		double[] x = perpendicular(xRef, z);
		if (norm(x) < 1e-6)
			x = perpendicular(fallbackX, z);
		x = normalize(x);
		return Matrix4.fromAxes(x, cross(z, x), z);
	}

	/**
	 * Para el muslo y el brazo, el giro sobre sí mismos se deduce de la rodilla
	 * o el codo, que son bisagras: solo se doblan en un plano (el de su eje X
	 * local). Por eso el eje X del muslo tiene que ser perpendicular al plano
	 * que forman muslo y tibia. Si no, la rodilla no podría seguir a la tibia
	 * del .bvh.
	 *
	 * Con la pierna casi recta ese plano no está definido (los dos huesos son
	 * casi paralelos), así que se mezcla con la referencia de giro: hasta 5º de
	 * flexión solo cuenta la referencia, a partir de 25º solo el plano, y entre
	 * medias una mezcla proporcional, para que no pegue saltos.
	 *
	 * @param sign +1 si la articulación se dobla hacia el +Y local (rodilla) y
	 *             -1 si hacia el -Y (codo).
	 */
	private static Matrix4 bendFrame(double[] a, double[] b, double[] c, int sign, double[] twistRef, double[] fallbackX) {
		double[] z = normalize(sub(b, a)); // dirección del muslo/brazo
		double[] next = normalize(sub(c, b)); // dirección de la tibia/antebrazo
		double[] bend = perpendicular(next, z); // hacia dónde se dobla, visto desde el muslo
		double amount = norm(bend); // = seno del ángulo de flexión
		double[] xRef = perpendicular(twistRef, z);
		if (norm(xRef) < 1e-6)
			xRef = perpendicular(fallbackX, z);
		xRef = normalize(xRef);

		double w = Math.max(0, Math.min(1, (amount - BEND_MIN) / (BEND_FULL - BEND_MIN)));
		if (w > 0) {
			// El eje Y local debe apuntar hacia donde se dobla (con el signo de
			// la articulación) y X = Y x Z
			double[] xBend = cross(scale(normalize(bend), sign), z);
			// Si con poca flexión el plano dice lo contrario que la referencia
			// (una rodilla un pelín hiperextendida), manda la referencia
			if (dot(xBend, xRef) >= 0 || w == 1)
				xRef = normalize(add(scale(xBend, w), scale(xRef, 1 - w)));
		}
		return frame(sub(b, a), xRef, fallbackX);
	}

	/**
	 * Referencia para el giro de un segmento sobre su propio eje (pronación del
	 * antebrazo, girar la cabeza...). Las posiciones de las articulaciones no lo
	 * dicen: un antebrazo apunta igual con la palma arriba o abajo. Hay que
	 * mirar cómo GIRA la articulación del .bvh:
	 * <ul>
	 * <li>En la calibración (fotograma 0) se supone que no hay giro: la
	 * referencia es el eje X neutro (el del padre).</li>
	 * <li>Después, el eje X del fotograma 0 se gira lo mismo que ha girado esa
	 * articulación del .bvh desde el fotograma 0: delta = G(t) * G(0)^-1.</li>
	 * </ul>
	 */
	private double[] twistReference(Segment s, String key, Frame f, double[] neutralX, boolean calibrate) {
		if (calibrate)
			return neutralX;
		return f.delta(key).transformDirection(firstRot.get(s).axis(0));
	}

	// ================================================================ preparación

	/** Busca una articulación imprescindible; si no está, error. */
	private void find(String key, String longSide, String shortSide) {
		if (!findOptional(key, longSide, shortSide))
			throw new IllegalArgumentException("El .bvh no tiene una articulación reconocible para '" + key
					+ "'. Nombres que se buscan: " + String.join(", ", PATTERNS.get(key.replaceAll("_[DI]$", ""))));
	}

	/** Busca una articulación probando los patrones de nombre. Devuelve si la encontró. */
	private boolean findOptional(String key, String longSide, String shortSide) {
		for (String pattern : PATTERNS.get(key.replaceAll("_[DI]$", ""))) {
			String wanted = String.format(pattern, longSide, shortSide);
			for (BvhMotion.Joint j : motion.getJoints())
				// Los End Site no tienen canales y no cuentan
				if (!j.getName().endsWith("_End") && cleanName(j.getName()).equals(wanted)) {
					joints.put(key, j);
					return true;
				}
		}
		return false;
	}

	/**
	 * "mixamorig:RightUpLeg" -> "rightupleg", "UpperLeg_R" -> "upperlegr":
	 * minúsculas, sin prefijo (lo que va antes de ':') y solo letras y números.
	 */
	private static String cleanName(String name) {
		String n = name.substring(name.lastIndexOf(':') + 1).toLowerCase();
		return n.replaceAll("[^a-z0-9]", "");
	}

	/**
	 * Camino en el árbol desde {@code from} hasta {@code to} (que cuelga de
	 * él): se sube de to hasta from por los padres y se da la vuelta a la lista.
	 */
	private static List<BvhMotion.Joint> pathBetween(BvhMotion.Joint from, BvhMotion.Joint to) {
		List<BvhMotion.Joint> path = new ArrayList<>();
		for (BvhMotion.Joint j = to; j != null; j = j.getParent()) {
			path.add(j);
			if (j == from) {
				Collections.reverse(path);
				return path;
			}
		}
		throw new IllegalArgumentException("En el .bvh el cuello no cuelga de la cadera");
	}

	/**
	 * Deduce los ejes del .bvh mirando el cuerpo en el primer fotograma:
	 * <ul>
	 * <li>Arriba: el eje del .bvh (X, Y o Z, con su signo) más parecido a la
	 * dirección cadera -> cuello.</li>
	 * <li>Derecha: de la cadera izquierda a la derecha, quitándole la parte
	 * vertical.</li>
	 * <li>Delante: arriba x derecha (para que los tres formen un sistema de mano
	 * derecha, como el nuestro: X derecha, Y delante, Z arriba).</li>
	 * </ul>
	 * Así da igual que el fichero use Y o Z hacia arriba o hacia dónde mire la
	 * persona al empezar: en nuestro mundo siempre arranca mirando hacia
	 * delante.
	 *
	 * Devuelve la matriz que pasa vectores del .bvh a nuestros ejes. Sus FILAS
	 * son nuestros ejes vistos desde el .bvh, que es la transpuesta (= inversa)
	 * de fromAxes.
	 */
	private Matrix4 detectAxes(Matrix4[] pose) {
		double[] hips = position(pose, joints.get("hips"));
		double[] trunk = sub(position(pose, joints.get("neck")), hips);
		double[] up = new double[3];
		int best = 0;
		for (int i = 1; i < 3; i++)
			if (Math.abs(trunk[i]) > Math.abs(trunk[best]))
				best = i;
		up[best] = Math.signum(trunk[best]);
		double[] right = sub(position(pose, joints.get("upleg_D")), position(pose, joints.get("upleg_I")));
		double[] x = normalize(perpendicular(right, up));
		double[] y = cross(up, x);
		return Matrix4.fromAxes(x, y, up).rigidInverse();
	}

	private static double[] position(Matrix4[] pose, BvhMotion.Joint j) {
		Matrix4 m = pose[j.getIndex()];
		return new double[] { m.tx(), m.ty(), m.tz() };
	}

	/**
	 * Nuestro pie en reposo (de pie, apoyado en el suelo): devuelve {altura
	 * del punto más bajo (tobillo o punta), inclinación en radianes}. Se hace
	 * la cinemática directa con todo a 0 y se restauran los ángulos.
	 */
	private double[] restFoot(Segment foot) {
		Map<Segment, double[]> saved = saveAngles();
		for (Segment s : saved.keySet())
			s.resetAngles();
		Matrix4 ankle = ForwardKinematics3D
				.frames(ForwardKinematics3D.computePositions(root, origin[0], origin[1], origin[2])).get(foot);
		double toe = ankle.transformPoint(new double[] { 0, 0, foot.getLength() })[2];
		restoreAngles(saved);
		return new double[] { Math.min(ankle.tz(), toe), pitch(ankle.axis(2)) };
	}

	/**
	 * Altura del suelo en cada fotograma: el punto más bajo de nuestros pies
	 * en una ventana de +-0,4 segundos alrededor de ese fotograma.
	 *
	 * Se mide con nuestros pies y no con los del .bvh porque no son los mismos
	 * puntos (su pie acaba en la base de los dedos y el nuestro en la punta).
	 * Y no se usa un único suelo para toda la animación (el punto más bajo de
	 * todas) porque hay grabaciones que "derivan": en la de caminar de CMU los
	 * pies van subiendo unos 6 cm a lo largo del clip (el suelo del
	 * laboratorio o la calibración no son perfectos), y el holograma acababa
	 * flotando. Con una ventana el suelo sigue esa deriva.
	 *
	 * El tamaño de la ventana es un compromiso. Al caminar o correr siempre
	 * hay un pie apoyado en menos de 0,4 s, así que en cada fotograma el pie
	 * de apoyo toca el suelo. Un salto normal está en el aire menos de 0,8 s:
	 * en el punto más alto la ventana todavía llega al despegue o al
	 * aterrizaje, y se ve como un salto. Con una ventana más grande se
	 * mezclarían posturas distintas (por ejemplo, el principio en "T" con las
	 * piernas rectas y la marcha con las rodillas dobladas) y los pies
	 * flotarían.
	 *
	 * Coste: O(n·w), con n fotogramas y w el tamaño de la ventana. Hay formas
	 * de hacerlo en O(n) (con una cola doble, "mínimo en ventana deslizante"),
	 * pero solo se hace una vez al cargar y no merece la pena complicarlo.
	 */
	private static double[] findFloor(double[] lowest, int halfWindow) {
		int n = lowest.length;
		double[] floor = new double[n];
		for (int f = 0; f < n; f++) {
			floor[f] = Double.MAX_VALUE;
			for (int g = Math.max(0, f - halfWindow); g <= Math.min(n - 1, f + halfWindow); g++)
				floor[f] = Math.min(floor[f], lowest[g]);
		}
		return floor;
	}

	/**
	 * Inclinación media del pie del .bvh cuando está APOYADO (los fotogramas en
	 * que el tobillo está más bajo: el 20% más bajo de todos).
	 *
	 * Hace falta porque nuestro pie y el del .bvh no son el mismo hueso: el
	 * nuestro va del tobillo a la punta de los dedos (baja unos 30º), y el de
	 * CMU del tobillo a la base de los dedos (baja menos). Si se copiase la
	 * dirección tal cual, nuestro pie iría siempre con la punta levantada.
	 * Pero cuando el pie está plano en el suelo, los dos deben tener su
	 * inclinación de reposo. La diferencia entre las dos es la corrección que
	 * se aplica en todos los fotogramas.
	 */
	private static double footPitchOffset(double[] ankleHeight, double[] pitch) {
		double[] sorted = ankleHeight.clone();
		java.util.Arrays.sort(sorted);
		double limit = sorted[sorted.length / 5]; // percentil 20
		double sum = 0;
		int count = 0;
		for (int f = 0; f < ankleHeight.length; f++)
			if (ankleHeight[f] <= limit) {
				sum += pitch[f];
				count++;
			}
		return sum / count;
	}

	/** Inclinación de un vector respecto al suelo, en radianes (positivo = hacia arriba). */
	private static double pitch(double[] v) {
		return Math.asin(Math.max(-1, Math.min(1, v[2] / norm(v))));
	}

	/**
	 * Cambia la inclinación de un vector en {@code angle} radianes sin cambiar
	 * hacia dónde apunta en horizontal: se separa en su dirección horizontal
	 * y su ángulo con el suelo, se suma el ángulo y se vuelve a montar.
	 */
	private static double[] tilt(double[] v, double angle) {
		double[] horizontal = { v[0], v[1], 0 };
		if (norm(horizontal) < 1e-9)
			return v; // vertical: no hay "hacia dónde" horizontal
		double p = pitch(v) + angle;
		double[] h = normalize(horizontal);
		return new double[] { h[0] * Math.cos(p), h[1] * Math.cos(p), Math.sin(p) };
	}

	private Map<Segment, double[]> saveAngles() {
		Map<Segment, double[]> saved = new IdentityHashMap<>();
		for (Segment s : root.flatten())
			saved.put(s, new double[] { s.getAngle(0), s.getAngle(1), s.getAngle(2) });
		return saved;
	}

	private static void restoreAngles(Map<Segment, double[]> saved) {
		saved.forEach((s, a) -> {
			for (int i = 0; i < 3; i++)
				s.setAngle(i, a[i]);
		});
	}

	// ================================================================ fotograma

	/**
	 * Un fotograma del .bvh visto con NUESTROS ejes. Es una clase interna (no
	 * static): puede usar los atributos del Retargeter que la crea (joints,
	 * toOurs...).
	 */
	private final class Frame {
		private final Matrix4[] pose; // cinemática del .bvh en este fotograma

		Frame(Matrix4[] pose) {
			this.pose = pose;
		}

		/** Posición de una articulación del .bvh, con nuestros ejes. */
		double[] pos(String key) {
			return pos(joints.get(key));
		}

		double[] pos(BvhMotion.Joint j) {
			return toOurs.transformDirection(position(pose, j));
		}

		/**
		 * Dónde acaba el hueso que empieza en esta articulación: la media de sus
		 * hijos (en la mano, por ejemplo, la base de los dedos y el pulgar).
		 */
		double[] endOf(String key) {
			BvhMotion.Joint j = joints.get(key);
			List<double[]> ends = new ArrayList<>();
			collectEnds(j, pos(j), ends);
			if (ends.isEmpty())
				return pos(j);
			double[] sum = new double[3];
			for (double[] e : ends)
				sum = add(sum, e);
			return scale(sum, 1.0 / ends.size());
		}

		/**
		 * Recoge la posición de los hijos de j. Si un hijo está en el mismo
		 * punto que el inicio (en CMU la mano tiene hijos con OFFSET 0), no
		 * sirve para dar una dirección y se baja a SUS hijos (recursivo).
		 */
		private void collectEnds(BvhMotion.Joint j, double[] start, List<double[]> ends) {
			for (BvhMotion.Joint c : j.getChildren()) {
				double[] p = pos(c);
				if (norm(sub(p, start)) > 1e-6)
					ends.add(p);
				else
					collectEnds(c, start, ends);
			}
		}

		/** Punta del pie: la articulación de los dedos si existe y, si no, el final del tobillo. */
		double[] toe(String side) {
			return joints.containsKey("toe" + side) ? endOf("toe" + side) : endOf("ankle" + side);
		}

		/**
		 * Punto a una fracción t (0-1) del camino de la columna, de la cadera al
		 * cuello, medida en longitud. El .bvh puede tener 2, 3 o 4 vértebras y
		 * nosotros solo 2 (lumbar y tórax), así que se reparte por distancias.
		 */
		double[] spinePoint(double t) {
			double total = 0;
			for (int i = 1; i < spine.size(); i++)
				total += norm(sub(pos(spine.get(i)), pos(spine.get(i - 1))));
			double target = t * total;
			for (int i = 1; i < spine.size(); i++) {
				double[] a = pos(spine.get(i - 1)), b = pos(spine.get(i));
				double len = norm(sub(b, a));
				if (target <= len && len > 0)
					return add(a, scale(sub(b, a), target / len)); // interpolación lineal
				target -= len;
			}
			return pos(spine.get(spine.size() - 1));
		}

		/**
		 * Cuánto ha girado una articulación del .bvh desde el fotograma 0,
		 * expresado con nuestros ejes: C * G(t) * G(0)^-1 * C^-1, donde G es la
		 * orientación en el mundo del .bvh y C = toOurs. (Para pasar un giro de
		 * unos ejes a otros hay que "entrar" con C^-1 y "salir" con C.)
		 */
		Matrix4 delta(String key) {
			int i = joints.get(key).getIndex();
			Matrix4 change = pose[i].rotationOnly().multiply(firstPose[i].rotationOnly().rigidInverse());
			return toOurs.multiply(change).multiply(toOurs.rigidInverse());
		}
	}
}

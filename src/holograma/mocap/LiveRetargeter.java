package holograma.mocap;

import static holograma.kinematics.Vec3.*;

import java.util.Map;

import holograma.kinematics.ForwardKinematics3D;
import holograma.kinematics.HumanSkeleton;
import holograma.kinematics.Matrix4;
import holograma.kinematics.Segment;

/**
 * Retargeting EN DIRECTO desde MediaPipe Pose (webcam o vídeo).
 *
 * Es la misma idea que Retargeter (copiar DIRECCIONES de una articulación a
 * otra y pasarlas a ángulos recorriendo el árbol desde la pelvis), pero con
 * dos diferencias:
 * <ul>
 * <li>MediaPipe solo da POSICIONES de 33 puntos (hombros, codos, caderas,
 * orejas, talones...), no giros. Lo que no se puede deducir de las
 * posiciones (por ejemplo, girar el antebrazo sobre sí mismo) se queda
 * neutro.</li>
 * <li>Los datos llegan de uno en uno, sin saber el futuro, así que no hay
 * calibración con toda la animación: el suelo se pone en cada fotograma
 * haciendo que el pie más bajo lo toque.</li>
 * </ul>
 *
 * <h2>Ejes de MediaPipe</h2>
 * Los "world landmarks" están en metros, con el origen entre las caderas: x
 * hacia la derecha de la IMAGEN, y hacia ABAJO y z hacia el fondo (más
 * pequeño = más cerca de la cámara). Para una persona mirando a la cámara, su
 * derecha está a la izquierda de la imagen, así que en nuestros ejes:
 *
 * <pre>
 * X (derecha de la persona) = -x     Y (delante, hacia la cámara) = -z     Z (arriba) = -y
 * </pre>
 *
 * <h2>Modo espejo</h2>
 * Si el holograma copia tu brazo derecho con SU brazo derecho, al mirarlo de
 * frente lo ves moverse al otro lado, como otra persona delante de ti. En
 * modo espejo se cambian los puntos de izquierda por los de derecha y se da
 * la vuelta a la x: tu derecha mueve su izquierda, como en un espejo.
 */
public class LiveRetargeter {

	// Índices de los puntos de MediaPipe Pose. Pares {izquierdo, derecho}.
	private static final int NOSE = 0;
	private static final int[] EAR = { 7, 8 }, SHOULDER = { 11, 12 }, ELBOW = { 13, 14 }, WRIST = { 15, 16 },
			PINKY = { 17, 18 }, INDEX = { 19, 20 }, HIP = { 23, 24 }, KNEE = { 25, 26 }, ANKLE = { 27, 28 },
			FOOT = { 31, 32 };
	/** Por debajo de esta visibilidad, el punto se considera fuera de la imagen. */
	private static final double VISIBLE = 0.5;

	private final Segment root;
	private final double[] origin;
	private final double lumbarFraction;
	private final Segment[] feet;
	private final double soleHeight; // altura del punto más bajo del pie en reposo (cm)
	private double[][] smooth; // puntos filtrados (ver apply)
	private double smoothing = 0.5;
	private boolean mirror = true;
	private boolean legsVisible;

	public LiveRetargeter(HumanSkeleton skeleton) {
		root = skeleton.getRoot();
		origin = skeleton.getOrigin();
		Segment lumbar = null, chest = null, footD = null, footI = null;
		for (Segment s : root.flatten())
			switch (s.getName()) {
			case "Lumbar" -> lumbar = s;
			case "Tórax" -> chest = s;
			case "Pie D" -> footD = s;
			case "Pie I" -> footI = s;
			default -> {
			}
			}
		lumbarFraction = lumbar.getLength() / (lumbar.getLength() + chest.getLength());
		feet = new Segment[] { footD, footI };
		// Altura de la planta en reposo: pie más bajo con todos los ángulos a 0
		double[][] saved = new double[root.flatten().size()][];
		int i = 0;
		for (Segment s : root.flatten()) {
			saved[i++] = new double[] { s.getAngle(0), s.getAngle(1), s.getAngle(2) };
			s.resetAngles();
		}
		soleHeight = lowestFoot(origin[2]);
		i = 0;
		for (Segment s : root.flatten()) {
			for (int k = 0; k < 3; k++)
				s.setAngle(k, saved[i][k]);
			i++;
		}
	}

	public void setMirror(boolean mirror) {
		this.mirror = mirror;
		smooth = null; // los puntos filtrados estaban en los ejes de antes: se empieza de cero
	}

	/** 0 = sin filtro; cuanto más cerca de 1, más suave (y más retraso). */
	public void setSmoothing(double s) {
		smoothing = s;
	}

	/** Si en la última postura se veían las piernas. */
	public boolean legsVisible() {
		return legsVisible;
	}

	/**
	 * Pone el esqueleto en la postura recibida.
	 *
	 * Filtro contra el temblor: la detección de MediaPipe "vibra" unos
	 * milímetros de un fotograma a otro. Se usa una media móvil EXPONENCIAL:
	 * filtrado = s · anterior + (1 - s) · nuevo. Cada punto nuevo cuenta un
	 * (1 - s) y el pasado se va olvidando poco a poco. Con s = 0,5 el temblor
	 * casi desaparece y el retraso es de unos 2 fotogramas.
	 *
	 * @param landmarks [33][4] de LiveReceiver (x, y, z, visibilidad)
	 * @return dónde va la pelvis (origen para ForwardKinematics3D)
	 */
	public double[] apply(double[][] landmarks) {
		if (smooth == null)
			smooth = new double[landmarks.length][];
		for (int i = 0; i < landmarks.length; i++) {
			double[] p = toOurs(landmarks[i]);
			smooth[i] = smooth[i] == null ? p : add(scale(smooth[i], smoothing), scale(p, 1 - smoothing));
		}
		// Piernas: si no se ven (sentado delante del ordenador, por ejemplo), se
		// quedan en reposo en vez de hacer cosas raras
		legsVisible = true;
		for (int[] pair : new int[][] { KNEE, ANKLE })
			for (int i : pair)
				if (landmarks[i][3] < VISIBLE)
					legsVisible = false;

		place(root, Matrix4.identity());

		// Suelo: el punto más bajo de los pies a la altura de la planta en reposo
		double height = origin[2] + soleHeight - lowestFoot(origin[2]);
		return new double[] { origin[0], origin[1], height };
	}

	/** Un punto de MediaPipe (metros) en nuestros ejes (cm). Ver javadoc de la clase. */
	private double[] toOurs(double[] p) {
		double x = mirror ? p[0] : -p[0];
		return new double[] { x * 100, -p[2] * 100, -p[1] * 100 };
	}

	/**
	 * Punto de un par {izquierdo, derecho} para un lado nuestro ("_D" o
	 * "_I"). En modo espejo, nuestro lado derecho usa el punto izquierdo de la
	 * persona y al revés.
	 */
	private double[] pt(int[] pair, String side) {
		boolean right = side.equals("_D") != mirror;
		return smooth[right ? pair[1] : pair[0]];
	}

	private double[] mid(int[] pair) {
		return scale(add(smooth[pair[0]], smooth[pair[1]]), 0.5);
	}

	/** Lateral (de izquierda a derecha de NUESTRO esqueleto) entre los dos puntos de un par. */
	private double[] lateral(int[] pair) {
		return sub(pt(pair, "_D"), pt(pair, "_I"));
	}

	/** Igual que Retargeter.place: recursivo desde la pelvis. */
	private void place(Segment s, Matrix4 parentRot) {
		Matrix4 neutral = parentRot.multiply(s.getBase());
		Matrix4 target = target(s, neutral);
		if (target == null)
			s.resetAngles();
		else {
			double[] angles = neutral.rigidInverse().multiply(target).eulerXYZ();
			for (int i = 0; i < 3; i++)
				s.setAngle(i, angles[i]);
		}
		Matrix4 rot = parentRot.multiply(s.localTransform());
		for (Segment c : s.getChildren())
			place(c, rot);
	}

	/** Orientación deseada de cada segmento a partir de los puntos de MediaPipe. */
	private Matrix4 target(Segment s, Matrix4 neutral) {
		String name = s.getName();
		String side = name.endsWith(" D") ? "_D" : name.endsWith(" I") ? "_I" : "";
		String base = side.isEmpty() ? name : name.substring(0, name.length() - 2);
		double[] nx = neutral.axis(0);
		double[] hips = mid(HIP), shoulders = mid(SHOULDER);
		// No hay puntos en la columna: se toma la recta de caderas a hombros y
		// se parte en dos (lumbar y tórax) en la misma proporción que las nuestras
		double[] spine = add(hips, scale(sub(shoulders, hips), lumbarFraction));
		boolean leg = base.equals("Muslo") || base.equals("Tibia") || base.equals("Pie");
		if (leg && !legsVisible)
			return null;

		switch (base) {
		case "Pelvis": {
			double[] x = normalize(lateral(HIP));
			double[] z = normalize(perpendicular(new double[] { 0, 0, 1 }, x));
			return Matrix4.fromAxes(x, cross(z, x), z);
		}
		case "Lumbar":
			return Retargeter.frame(sub(spine, hips), lateral(HIP), nx);
		case "Tórax":
			return Retargeter.frame(sub(shoulders, spine), lateral(SHOULDER), nx);
		case "Cuello":
			return Retargeter.frame(sub(mid(EAR), shoulders), lateral(EAR), nx);
		case "Cabeza": {
			// Ejes de la cabeza: X = de oreja a oreja, Y = hacia la nariz, Z = X × Y
			double[] x = normalize(lateral(EAR));
			double[] forward = perpendicular(sub(smooth[NOSE], mid(EAR)), x);
			if (norm(forward) < 1e-6)
				return null;
			double[] y = normalize(forward);
			return Matrix4.fromAxes(x, y, cross(x, y));
		}
		case "Clavícula":
			return null; // no hay puntos para la clavícula: en reposo
		case "Brazo":
			return Retargeter.bendFrame(pt(SHOULDER, side), pt(ELBOW, side), pt(WRIST, side), -1, nx, nx);
		case "Antebrazo":
			return Retargeter.frame(sub(pt(WRIST, side), pt(ELBOW, side)), nx, nx);
		case "Mano": {
			double[] hand = scale(add(pt(INDEX, side), pt(PINKY, side)), 0.5);
			return Retargeter.frame(sub(hand, pt(WRIST, side)), nx, nx);
		}
		case "Muslo":
			return Retargeter.bendFrame(pt(HIP, side), pt(KNEE, side), pt(ANKLE, side), 1, nx, nx);
		case "Tibia":
			return Retargeter.frame(sub(pt(ANKLE, side), pt(KNEE, side)), nx, nx);
		case "Pie":
			// Del tobillo a la punta del pie: el mismo hueso que el nuestro
			return Retargeter.frame(sub(pt(FOOT, side), pt(ANKLE, side)), nx, nx);
		default: // Cadera D/I
			return null;
		}
	}

	/** Altura del punto más bajo de los dos pies (tobillo o punta) con la pelvis a esa altura. */
	private double lowestFoot(double pelvisHeight) {
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
}

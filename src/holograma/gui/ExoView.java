package holograma.gui;

import java.util.HashMap;
import java.util.Map;

import holograma.dynamics.Exoskeleton;
import holograma.dynamics.InverseDynamics;
import holograma.kinematics.Matrix4;
import holograma.kinematics.Segment;
import holograma.kinematics.Vec3;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Cylinder;
import javafx.scene.transform.Affine;

/**
 * Dibujo 3D del exoesqueleto: un motor (cilindro) en cada articulación con
 * motor, por FUERA de la pierna, barras que bajan por el muslo y la pierna, y
 * un cinturón por detrás de la cadera.
 *
 * Color de los motores según su potencia: naranja cuando empujan (gastan
 * batería), verde cuando frenan (podrían recargarla) y gris cuando apenas
 * hacen nada.
 *
 * Todas las piezas son cilindros de JavaFX de altura 1 colocados con una
 * transformación (Affine) que los gira, los estira y los mueve (ver
 * placeBetween). Se crean una vez y en cada fotograma solo cambia su Affine.
 */
class ExoView {

	// Cuánto se separa cada pieza de la articulación hacia fuera (cm): lo
	// justo para que quede pegada por fuera de la pierna
	private static final double HIP_OUT = 15, KNEE_OUT = 8.5, ANKLE_OUT = 7;
	private static final double FULL_POWER = 60; // W con los que el motor se ve del todo naranja/verde
	private static final int LEVELS = 12;

	private final Group group = new Group();
	private final Map<String, Cylinder> parts = new HashMap<>(); // "motor Cadera D", "barra Muslo D"...
	private final PhongMaterial metal = new PhongMaterial(Color.web("#aab4c3"));
	private final PhongMaterial idle = new PhongMaterial(Color.web("#6b7280"));
	private final PhongMaterial[] pushing = new PhongMaterial[LEVELS], braking = new PhongMaterial[LEVELS];

	ExoView() {
		metal.setSpecularColor(Color.WHITE);
		for (int i = 0; i < LEVELS; i++) {
			double t = (i + 1.0) / LEVELS;
			pushing[i] = new PhongMaterial(Color.web("#6b7280").interpolate(Color.web("#ff8a1f"), t));
			braking[i] = new PhongMaterial(Color.web("#6b7280").interpolate(Color.web("#2fe08a"), t));
		}
		for (String side : new String[] { " D", " I" }) {
			for (String joint : new String[] { "Cadera", "Rodilla", "Tobillo" })
				add("motor " + joint + side, 5.5, idle);
			add("barra Muslo" + side, 1.3, metal);
			add("barra Tibia" + side, 1.2, metal);
		}
		add("cinturón", 1.6, metal);
	}

	private void add(String name, double radius, PhongMaterial material) {
		Cylinder c = new Cylinder(radius, 1);
		c.setMaterial(material);
		c.getTransforms().add(new Affine());
		parts.put(name, c);
		group.getChildren().add(c);
	}

	Node getNode() {
		return group;
	}

	void setVisible(boolean visible) {
		group.setVisible(visible);
	}

	/**
	 * Coloca las piezas en la postura actual.
	 *
	 * @param frames  cinemática directa (en cm)
	 * @param byName  segmentos por nombre
	 * @param exo     configuración (qué motores hay)
	 * @param result  dinámica inversa (para el color de los motores), o null
	 */
	void update(Map<Segment, Matrix4> frames, Map<String, Segment> byName, Exoskeleton exo,
			InverseDynamics.Result result) {
		Map<String, Double> power = new HashMap<>();
		if (result != null)
			for (InverseDynamics.Joint j : result.joints())
				power.put(j.name(), j.exoPower());

		double[][] hipMotors = new double[2][];
		int k = 0;
		for (String side : new String[] { " D", " I" }) {
			// En las piernas el eje X local apunta hacia la derecha del cuerpo en
			// los dos lados (ver HumanSkeleton.Builder), así que "hacia fuera" es
			// +X en la derecha y -X en la izquierda
			int out = side.equals(" D") ? 1 : -1;
			Matrix4 thigh = frames.get(byName.get("Muslo" + side));
			Matrix4 shin = frames.get(byName.get("Tibia" + side));
			Matrix4 foot = frames.get(byName.get("Pie" + side));
			double[] hip = outside(thigh, HIP_OUT * out), knee = outside(shin, KNEE_OUT * out),
					ankle = outside(foot, ANKLE_OUT * out);
			hipMotors[k++] = hip;

			motor("motor Cadera" + side, exo.hasHip(), hip, thigh.axis(0), power.get("Cadera" + side));
			motor("motor Rodilla" + side, exo.hasKnee(), knee, shin.axis(0), power.get("Rodilla" + side));
			motor("motor Tobillo" + side, exo.hasAnkle(), ankle, foot.axis(0), power.get("Tobillo" + side));
			bar("barra Muslo" + side, exo.hasHip() || exo.hasKnee(), hip, knee);
			bar("barra Tibia" + side, exo.hasKnee() || exo.hasAnkle(), knee, ankle);
		}
		// Cinturón: de un motor de cadera al otro, pasando 12 cm por detrás
		// (eje -Y de la pelvis)
		double[] back = Vec3.scale(frames.get(byName.get("Pelvis")).axis(1), -12);
		bar("cinturón", exo.hasHip(), Vec3.add(hipMotors[0], back), Vec3.add(hipMotors[1], back));
	}

	/** Punto de la articulación (origen del frame) desplazado hacia fuera por su eje X. */
	private static double[] outside(Matrix4 frame, double cm) {
		return frame.transformPoint(new double[] { cm, 0, 0 });
	}

	private void motor(String name, boolean on, double[] center, double[] axis, Double power) {
		Cylinder c = parts.get(name);
		c.setVisible(on);
		if (!on)
			return;
		// Un motor de 6 cm de largo con su eje en el de la articulación
		placeBetween(c, Vec3.add(center, Vec3.scale(axis, -3)), Vec3.add(center, Vec3.scale(axis, 3)));
		double p = power == null ? 0 : power;
		int i = (int) Math.min(LEVELS - 1, Math.abs(p) / FULL_POWER * LEVELS);
		c.setMaterial(Math.abs(p) < 2 ? idle : p > 0 ? pushing[i] : braking[i]);
	}

	private void bar(String name, boolean on, double[] a, double[] b) {
		Cylinder c = parts.get(name);
		c.setVisible(on);
		if (on)
			placeBetween(c, a, b);
	}

	/**
	 * Coloca un cilindro de altura 1 (que JavaFX centra en su origen y orienta
	 * según el eje Y) para que vaya del punto a al b.
	 *
	 * La matriz tiene como columnas: un eje X cualquiera perpendicular, el eje
	 * Y = la dirección de a a b MULTIPLICADA por la distancia (eso estira el
	 * cilindro hasta medir lo mismo), y Z = X × Y. La traslación es el punto
	 * medio. No es una transformación rígida (estira), pero JavaFX la acepta
	 * igual.
	 */
	private static void placeBetween(Cylinder c, double[] a, double[] b) {
		double[] d = Vec3.sub(b, a);
		double len = Vec3.norm(d);
		if (len < 1e-6) {
			c.setVisible(false);
			return;
		}
		double[] y = Vec3.scale(d, 1 / len);
		// Cualquier vector que no sea paralelo a y sirve de referencia para X
		double[] ref = Math.abs(y[2]) < 0.9 ? new double[] { 0, 0, 1 } : new double[] { 1, 0, 0 };
		double[] x = Vec3.normalize(Vec3.cross(y, ref));
		double[] z = Vec3.cross(x, y);
		double[] mid = Vec3.scale(Vec3.add(a, b), 0.5);
		Matrix4 m = Matrix4.translation(mid[0], mid[1], mid[2]).multiply(Matrix4.fromAxes(x, Vec3.scale(y, len), z));
		double[] v = m.toArray();
		((Affine) c.getTransforms().get(0)).setToTransform(v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8], v[9],
				v[10], v[11]);
	}
}

package holograma.dynamics;

import static holograma.kinematics.Vec3.*;

import java.util.ArrayList;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;

import holograma.kinematics.ForwardKinematics3D;
import holograma.kinematics.HumanSkeleton;
import holograma.kinematics.Matrix4;
import holograma.kinematics.Segment;

/**
 * Dinámica inversa: a partir de cómo se mueve el cuerpo, calcula el PAR
 * (momento, en N·m) que tiene que hacer cada articulación.
 *
 * <h2>Directa e inversa</h2>
 * <ul>
 * <li>Dinámica directa: pares de los músculos/motores -> movimiento (lo que
 * hace un simulador).</li>
 * <li>Dinámica INVERSA: movimiento -> pares. Es lo que se usa en biomecánica
 * para saber cuánto esfuerzo hace una rodilla al correr, y para dimensionar
 * los motores de un exoesqueleto: el motor de la rodilla tiene que poder dar
 * el par que sale aquí.</li>
 * </ul>
 *
 * <h2>Newton-Euler recursivo</h2>
 * Cada segmento es un sólido rígido con masa m, centro de masas c y tensor de
 * inercia I. Las leyes de Newton y Euler dicen qué fuerza y qué momento
 * netos necesita para moverse como se mueve:
 *
 * <pre>
 * fuerza neta  = m · a           (a = aceleración del centro de masas)
 * momento neto = I·α + ω × (I·ω) (ω = velocidad angular, α = aceleración angular)
 * </pre>
 *
 * Sobre un segmento actúan la gravedad, su padre (en la articulación de
 * arriba), sus hijos (en las de abajo) y, en los pies, el suelo. Si ya se
 * sabe lo que hacen los hijos, lo único que falta es lo del padre: se
 * despeja. Por eso el cálculo va DE LAS HOJAS A LA RAÍZ: primero las manos y
 * los pies, luego antebrazos y tibias... hasta la pelvis.
 *
 * Es el recorrido contrario al de la cinemática directa (que va de la raíz a
 * las hojas, en PREORDEN): aquí cada nodo necesita el resultado de sus hijos
 * antes de calcularse, así que es un recorrido en POSTORDEN. Con recursión
 * sale solo: primero se llama a los hijos y luego se hace la cuenta del nodo.
 *
 * <h2>El suelo y la "fuerza residual"</h2>
 * En un laboratorio de biomecánica, la fuerza del suelo se mide con
 * plataformas de fuerza. Aquí no hay, así que se estima: la fuerza total que
 * necesita el cuerpo entero (suma de m·(a - g) de todos los segmentos) la tiene
 * que dar el suelo, y se reparte entre los pies apoyados (ver groundForces).
 * Al llegar a la pelvis sobra algo de fuerza y de momento: lo que haría falta
 * de más para que todo cuadre. Es la "fuerza residual" (o "fuerza fantasma")
 * y mide lo buena que es la estimación: si fuese perfecta, valdría 0.
 *
 * <h2>Limitaciones (conscientes)</h2>
 * Las masas e inercias son las de una persona media (tablas de de Leva,
 * 1996) escaladas a la masa corporal, los pares máximos son orientativos (un
 * solo valor por articulación, cuando en realidad depende de la dirección y
 * del ángulo) y no se modelan músculos: es el par NETO de cada articulación,
 * la suma de lo que hacen todos los músculos que la cruzan.
 */
public class InverseDynamics {

	/** Gravedad en m/s², hacia abajo (eje Z del mundo hacia arriba). */
	private static final double[] GRAVITY = { 0, 0, -9.81 };
	/** Un punto del pie está apoyado si está a menos de esto del suelo (cm). */
	private static final double CONTACT = 3;
	/** Media anchura del pie (m), para la zona de apoyo. */
	private static final double FOOT_HALF_WIDTH = 0.045;
	private static final double CM = 0.01; // las posiciones vienen en cm y aquí se trabaja en metros

	/**
	 * Datos de cada tipo de segmento.
	 *
	 * @param mass      fracción de la masa corporal (de Leva 1996, hombre)
	 * @param com       dónde está el centro de masas, como fracción de la
	 *                  longitud desde la articulación de arriba
	 * @param radius    radio de giro transversal (fracción de la longitud): I =
	 *                  m · (radio · longitud)²
	 * @param radiusLong radio de giro alrededor del propio eje del segmento
	 * @param maxTorque par máximo aproximado de la articulación de arriba (N·m),
	 *                  para el mapa de esfuerzo. 0 = no se muestra.
	 * @param joint     nombre de la articulación de arriba en la interfaz
	 */
	private record Params(double mass, double com, double radius, double radiusLong, double maxTorque, String joint) {
	}

	/**
	 * Tabla por segmento (sin el " D"/" I"). La masa de la pelvis se reparte
	 * entre los dos segmentos "Cadera" (la pelvis mide 0 y no tiene centro de
	 * masas propio). Las fracciones suman 1.
	 */
	private static Params params(String base) {
		return switch (base) {
		case "Pelvis" -> new Params(0, 0, 0, 0, 0, null);
		case "Cadera" -> new Params(0.05585, 0.5, 0.3, 0.3, 0, null);
		case "Lumbar" -> new Params(0.1633, 0.45, 0.48, 0.47, 250, "Lumbar"); // par en L5/S1
		case "Tórax" -> new Params(0.1596, 0.5, 0.5, 0.46, 200, "Tórax");
		case "Cuello" -> new Params(0.01, 0.5, 0.3, 0.3, 40, "Cuello");
		case "Cabeza" -> new Params(0.0594, 0.5, 0.36, 0.38, 30, "Cabeza");
		case "Clavícula" -> new Params(0, 0, 0, 0, 0, null);
		case "Brazo" -> new Params(0.0271, 0.577, 0.285, 0.158, 80, "Hombro");
		case "Antebrazo" -> new Params(0.0162, 0.457, 0.276, 0.121, 70, "Codo");
		case "Mano" -> new Params(0.0061, 0.40, 0.30, 0.20, 15, "Muñeca");
		case "Muslo" -> new Params(0.1416, 0.4095, 0.329, 0.149, 200, "Cadera");
		case "Tibia" -> new Params(0.0433, 0.4459, 0.255, 0.103, 220, "Rodilla");
		case "Pie" -> new Params(0.0137, 0.40, 0.257, 0.124, 180, "Tobillo");
		default -> throw new IllegalArgumentException("Segmento sin datos: " + base);
		};
	}

	/**
	 * Resultado para una articulación (la de arriba de cada segmento).
	 *
	 * @param torque    módulo del par total que necesita la articulación (N·m)
	 * @param human     módulo del par que le queda a la persona después de la
	 *                  ayuda del exoesqueleto (sin exo, igual que torque)
	 * @param effort    human / par máximo de la articulación
	 * @param local     el par total en los ejes locales del segmento: X =
	 *                  flexión/extensión, Y = abducción/aducción (o lateral), Z
	 *                  = rotación sobre el propio hueso
	 * @param speed     velocidad angular de flexión/extensión de la
	 *                  articulación (rad/s, eje X local): la del hijo menos la
	 *                  del padre
	 * @param power     potencia de la articulación (W) = par · velocidad
	 *                  angular. Positiva: los músculos empujan a favor del
	 *                  movimiento y GENERAN energía (subir un escalón). Negativa:
	 *                  frenan el movimiento y la ABSORBEN (bajarlo).
	 * @param exoTorque par que da el motor del exo (N·m, sobre el eje X; 0 si
	 *                  no hay motor)
	 * @param exoPower  potencia del motor (W) = exoTorque · speed
	 */
	public record Joint(Segment segment, String name, double torque, double human, double effort, double[] local,
			double speed, double power, double exoTorque, double exoPower) {
	}

	/**
	 * Quién decide cuánto ayuda el exoesqueleto (lo implementa Exoskeleton).
	 * Es una INTERFAZ: InverseDynamics no sabe nada de motores, solo pregunta
	 * "para esta articulación, que necesita este par de flexión, ¿cuánto pones
	 * tú?". Así cualquier controlador nuevo vale sin tocar esta clase.
	 */
	public interface Assistance {
		/**
		 * @param s        segmento (su articulación de arriba)
		 * @param flexion  par de flexión/extensión que necesita (N·m)
		 * @return par que da el motor sobre el mismo eje (0 = sin motor)
		 */
		double torque(Segment s, double flexion);
	}

	/** Resultado completo de un cálculo. */
	public record Result(List<Joint> joints, Map<Segment, Double> effort, double[] residualForce,
			double[] residualMoment, String support, double[] groundForce) {
	}

	private final Segment root;
	private final List<Segment> segments;
	private final Map<Segment, Params> table = new IdentityHashMap<>();
	private final Segment[] feet;
	// Talón de cada pie en coordenadas locales del pie (ver constructor)
	private final double[][] heelLocal = new double[2][];
	private final double soleHeight; // altura de la planta en reposo (cm)
	private double bodyMass = 70; // kg
	// Masas añadidas (el exoesqueleto): segmento -> {kg, posición como fracción de su longitud}
	private final Map<Segment, List<double[]>> extraMass = new IdentityHashMap<>();
	private Assistance assistance; // null = sin exoesqueleto

	// ---- Datos del cálculo en curso (se rellenan en compute) ----
	private Map<Segment, double[]> com, acc, omega, alpha; // en metros y segundos
	private Map<Segment, double[]> force, moment; // lo que el padre ejerce sobre cada segmento
	private final Map<Segment, double[]> groundForce = new IdentityHashMap<>();
	private final Map<Segment, double[]> pressureCenter = new IdentityHashMap<>();
	private final Map<Segment, double[]> groundMoment = new IdentityHashMap<>(); // giro libre (rozamiento)

	public InverseDynamics(HumanSkeleton skeleton) {
		root = skeleton.getRoot();
		segments = root.flatten();
		Segment footD = null, footI = null;
		for (Segment s : segments) {
			table.put(s, params(baseName(s)));
			if (s.getName().equals("Pie D"))
				footD = s;
			if (s.getName().equals("Pie I"))
				footI = s;
		}
		feet = new Segment[] { footD, footI };

		// El talón no es una articulación del esqueleto: se define como el punto
		// de la planta que hay justo debajo del tobillo en reposo. Se guarda en
		// coordenadas LOCALES del pie, así que luego se mueve con él (cuando se
		// levanta el talón al andar, sube).
		double[] angles = new double[segments.size() * 3];
		for (int i = 0; i < segments.size(); i++)
			for (int k = 0; k < 3; k++) {
				angles[3 * i + k] = segments.get(i).getAngle(k);
				segments.get(i).setAngle(k, 0);
			}
		double[] o = skeleton.getOrigin();
		Map<Segment, Matrix4> rest = ForwardKinematics3D.frames(ForwardKinematics3D.computePositions(root, o[0], o[1], o[2]));
		double sole = Double.MAX_VALUE;
		for (int f = 0; f < 2; f++) {
			Matrix4 ankle = rest.get(feet[f]);
			double toeZ = ankle.transformPoint(new double[] { 0, 0, feet[f].getLength() })[2];
			double[] heel = { ankle.tx(), ankle.ty(), toeZ };
			heelLocal[f] = ankle.rigidInverse().transformPoint(heel);
			sole = Math.min(sole, toeZ);
		}
		soleHeight = sole;
		for (int i = 0; i < segments.size(); i++)
			for (int k = 0; k < 3; k++)
				segments.get(i).setAngle(k, angles[3 * i + k]);
	}

	public void setBodyMass(double kg) {
		bodyMass = kg;
	}

	public double getBodyMass() {
		return bodyMass;
	}

	/**
	 * Añade una masa puntual pegada a un segmento (un motor, una barra del
	 * exoesqueleto...), a una fracción de su longitud desde la articulación de
	 * arriba. Cuenta en el peso y en las aceleraciones, pero no en la inercia
	 * de giro (es pequeña comparada con la del cuerpo).
	 */
	public void addMass(Segment s, double kg, double fraction) {
		extraMass.computeIfAbsent(s, k -> new ArrayList<>()).add(new double[] { kg, fraction });
	}

	public void clearExtraMass() {
		extraMass.clear();
	}

	public void setAssistance(Assistance a) {
		assistance = a;
	}

	/** Masa del trozo de cuerpo (sin lo añadido). */
	private double bodyPartMass(Segment s) {
		return table.get(s).mass() * bodyMass;
	}

	/** Masa total del segmento: cuerpo + lo añadido. */
	private double mass(Segment s) {
		double m = bodyPartMass(s);
		for (double[] e : extraMass.getOrDefault(s, List.of()))
			m += e[0];
		return m;
	}

	/**
	 * Centro de masas del conjunto (cuerpo + añadidos), como fracción de la
	 * longitud: la media de las posiciones ponderada por las masas.
	 */
	private double comFraction(Segment s) {
		double m = bodyPartMass(s), sum = m * table.get(s).com();
		for (double[] e : extraMass.getOrDefault(s, List.of())) {
			m += e[0];
			sum += e[0] * e[1];
		}
		return m == 0 ? 0 : sum / m;
	}

	/**
	 * Calcula los pares de todas las articulaciones.
	 *
	 * Las velocidades y aceleraciones salen de tres posturas seguidas
	 * (DIFERENCIAS FINITAS, como en Métodos Numéricos): la de ahora y las de h
	 * segundos antes y después. Para una postura quieta se pasa la misma
	 * tres veces y todo sale 0 (solo cuenta la gravedad: es el caso estático).
	 *
	 * @param prev           frames (de ForwardKinematics3D.frames) h segundos antes
	 * @param cur            frames ahora
	 * @param next           frames h segundos después
	 * @param h              paso de tiempo en segundos
	 * @param alwaysStanding true: el pie más bajo está siempre apoyado (para
	 *                       mover el esqueleto a mano, con la pelvis fija).
	 *                       false: el suelo está a la altura de la planta en
	 *                       reposo, y si los dos pies están por encima, el
	 *                       cuerpo está en el aire (saltos, carrera).
	 */
	public Result compute(Map<Segment, Matrix4> prev, Map<Segment, Matrix4> cur, Map<Segment, Matrix4> next, double h,
			boolean alwaysStanding) {
		com = new IdentityHashMap<>();
		acc = new IdentityHashMap<>();
		omega = new IdentityHashMap<>();
		alpha = new IdentityHashMap<>();
		force = new IdentityHashMap<>();
		moment = new IdentityHashMap<>();

		// 1) Cinemática de cada segmento: centro de masas, aceleración,
		// velocidad y aceleración angulares
		for (Segment s : segments) {
			double[] c0 = comOf(s, prev), c1 = comOf(s, cur), c2 = comOf(s, next);
			com.put(s, c1);
			// Segunda derivada por diferencias centrales: (x(t+h) - 2x(t) + x(t-h)) / h²
			acc.put(s, scale(add(sub(c2, scale(c1, 2)), c0), 1 / (h * h)));
			// Giro entre fotogramas: R_siguiente * R_actual^-1 (en el mundo)
			Matrix4 r0 = prev.get(s).rotationOnly(), r1 = cur.get(s).rotationOnly(), r2 = next.get(s).rotationOnly();
			double[] wAfter = scale(r2.multiply(r1.rigidInverse()).rotationVector(), 1 / h);
			double[] wBefore = scale(r1.multiply(r0.rigidInverse()).rotationVector(), 1 / h);
			omega.put(s, scale(add(wAfter, wBefore), 0.5));
			alpha.put(s, scale(sub(wAfter, wBefore), 1 / h));
		}

		// 2) Fuerza del suelo
		String support = groundForces(cur, alwaysStanding);

		// 3) Newton-Euler de las hojas a la raíz
		solve(root, cur);

		// 4) Resultados por articulación
		List<Joint> joints = new ArrayList<>();
		Map<Segment, Double> effort = new IdentityHashMap<>();
		for (Segment s : segments) {
			Params p = table.get(s);
			if (p.joint() == null)
				continue;
			double[] m = moment.get(s);
			Matrix4 rot = cur.get(s).rotationOnly();
			double[] local = rot.rigidInverse().transformDirection(m);
			// Velocidad angular de la articulación: la del segmento menos la de su
			// padre (lo que gira uno respecto al otro), en ejes del segmento
			double[] parentOmega = s.getParent() == null ? new double[3] : omega.get(s.getParent());
			double[] speedLocal = rot.rigidInverse().transformDirection(sub(omega.get(s), parentOmega));
			double power = dot(m, sub(omega.get(s), parentOmega));

			// Ayuda del exo: un motor sobre el eje X (flexión/extensión). Lo que
			// le queda a la persona es el par total menos el del motor.
			double exo = assistance == null ? 0 : assistance.torque(s, local[0]);
			double[] human = sub(m, scale(rot.axis(0), exo));
			double e = norm(human) / p.maxTorque();
			effort.put(s, e);
			String side = s.getName().endsWith(" D") ? " D" : s.getName().endsWith(" I") ? " I" : "";
			joints.add(new Joint(s, p.joint() + side, norm(m), norm(human), e, local, speedLocal[0], power, exo,
					exo * speedLocal[0]));
		}
		// Las regiones sin articulación propia toman el color de la de al lado
		for (Segment s : segments)
			if (!effort.containsKey(s))
				effort.put(s, effort.getOrDefault(neighbour(s), 0.0));

		double[] total = new double[3];
		for (double[] f : groundForce.values())
			total = add(total, f);
		return new Result(joints, effort, force.get(root), moment.get(root), support, total);
	}

	/**
	 * Newton-Euler para un segmento y, antes, para todo lo que cuelga de él
	 * (recursión en POSTORDEN). Guarda en force/moment lo que el padre ejerce
	 * sobre el segmento, en su articulación de arriba.
	 *
	 * <pre>
	 * Fuerza:  F = m·(a - g) + Σ F_hijo - F_suelo
	 * Momento (respecto al centro de masas c):
	 *          M_c = I·α + ω×(I·ω) + Σ [M_hijo + (p_hijo - c) × F_hijo] - (cop - c) × F_suelo - T_suelo
	 * Y respecto a la articulación de arriba p:
	 *          M = M_c - (p - c) × F
	 * </pre>
	 * (p_hijo = articulación donde empieza cada hijo, cop = centro de presión
	 * del pie en el suelo, T_suelo = giro libre del rozamiento, × = producto
	 * vectorial.)
	 */
	private void solve(Segment s, Map<Segment, Matrix4> frames) {
		// Caso recursivo primero: los hijos (en las hojas, el bucle no se ejecuta)
		for (Segment child : s.getChildren())
			solve(child, frames);

		double m = mass(s);
		double[] c = com.get(s);
		double[] joint = position(frames.get(s));

		double[] f = scale(sub(acc.get(s), GRAVITY), m);
		double[] mc = eulerMoment(s, frames.get(s));
		for (Segment child : s.getChildren()) {
			double[] fc = force.get(child);
			f = add(f, fc);
			mc = add(mc, add(moment.get(child), cross(sub(position(frames.get(child)), c), fc)));
		}
		double[] ground = groundForce.get(s);
		if (ground != null) {
			f = sub(f, ground);
			mc = sub(mc, add(cross(sub(pressureCenter.get(s), c), ground), groundMoment.get(s)));
		}
		force.put(s, f);
		moment.put(s, sub(mc, cross(sub(joint, c), f)));
	}

	/**
	 * Momento que necesita el segmento para girar como gira (ecuación de
	 * Euler): I·α + ω × (I·ω).
	 *
	 * El tensor de inercia es diagonal en los ejes LOCALES del segmento: el
	 * mismo valor para los dos ejes transversales (X, Y) y otro más pequeño
	 * para el eje largo (Z). Para multiplicarlo por un vector del mundo: se
	 * pasa el vector a ejes locales (R^T · v), se multiplica por la diagonal y
	 * se vuelve al mundo (R · ...). Es decir, I_mundo = R · I_local · R^T.
	 */
	private double[] eulerMoment(Segment s, Matrix4 frame) {
		double m = bodyPartMass(s); // la inercia es solo la del cuerpo (ver addMass)
		if (m == 0)
			return new double[3];
		Params p = table.get(s);
		double len = s.getLength() * CM;
		double it = m * Math.pow(p.radius() * len, 2), il = m * Math.pow(p.radiusLong() * len, 2);
		double[] w = omega.get(s);
		double[] iw = inertiaTimes(frame, it, il, w);
		return add(inertiaTimes(frame, it, il, alpha.get(s)), cross(w, iw));
	}

	private static double[] inertiaTimes(Matrix4 frame, double it, double il, double[] v) {
		Matrix4 r = frame.rotationOnly();
		double[] local = r.rigidInverse().transformDirection(v);
		return r.transformDirection(new double[] { it * local[0], it * local[1], il * local[2] });
	}

	/**
	 * Estima la fuerza que hace el suelo sobre cada pie y DÓNDE la hace.
	 *
	 * 1. Fuerza total: la que necesita el cuerpo entero para moverse como se
	 * mueve, F = Σ m·(a - g). Parado, es el peso; al aterrizar de un salto,
	 * varias veces el peso.
	 *
	 * 2. Centro de presión (COP): el punto del suelo donde "se aplica" esa
	 * fuerza. No se puede poner en cualquier sitio. El cuerpo entero también
	 * necesita un momento, M = Σ [c × m·(a - g) + I·α + ω×(I·ω)], y el suelo es
	 * lo único que se lo puede dar: p × F + (0, 0, Tz) = M, con p = (px, py,
	 * suelo). De las componentes x e y se despejan px y py:
	 *
	 * <pre>
	 * (p × F)x = py·Fz - pz·Fy = Mx   =>   py = (Mx + pz·Fy) / Fz
	 * (p × F)y = pz·Fx - px·Fz = My   =>   px = (pz·Fx - My) / Fz
	 * </pre>
	 * y Tz (un giro alrededor de la vertical, que hace el rozamiento) es lo
	 * que falta en z. Es lo mismo que mide una plataforma de fuerzas.
	 *
	 * 3. El suelo solo empuja donde hay pie: el COP tiene que estar dentro de
	 * la zona de apoyo (ver supportPolygon). Si sale fuera, se lleva al punto
	 * más cercano del borde, y lo que no cuadra queda como fuerza RESIDUAL en
	 * la pelvis: el cuerpo se estaría cayendo (por ejemplo, si con la pelvis
	 * fija se levanta una pierna y el peso queda fuera del pie).
	 *
	 * 4. Con los dos pies apoyados se reparte con la regla de la palanca: si el
	 * COP está a un 30% del camino del pie derecho al izquierdo, el derecho
	 * lleva el 70% y el izquierdo el 30%. Cada pie recibe su fuerza en un punto
	 * elegido para que el conjunto equivalga a la fuerza total en el COP.
	 *
	 * @return texto con los pies apoyados
	 */
	private String groundForces(Map<Segment, Matrix4> frames, boolean alwaysStanding) {
		groundForce.clear();
		pressureCenter.clear();
		groundMoment.clear();

		// 1) Fuerza y momento (respecto al origen) que necesita el cuerpo entero
		double[] total = new double[3], needed = new double[3];
		for (Segment s : segments) {
			double[] f = scale(sub(acc.get(s), GRAVITY), mass(s));
			total = add(total, f);
			needed = add(needed, add(cross(com.get(s), f), eulerMoment(s, frames.get(s))));
		}

		// Talón y punta de cada pie (en cm, como los frames)
		double[][][] points = new double[2][][];
		double lowest = Double.MAX_VALUE;
		for (int f = 0; f < 2; f++) {
			Matrix4 frame = frames.get(feet[f]);
			points[f] = new double[][] { frame.transformPoint(heelLocal[f]),
					frame.transformPoint(new double[] { 0, 0, feet[f].getLength() }) };
			for (double[] pt : points[f])
				lowest = Math.min(lowest, pt[2]);
		}
		double ground = alwaysStanding ? lowest : soleHeight;

		// Pies apoyados y sus puntos de contacto (en metros, a ras de suelo)
		List<Integer> standing = new ArrayList<>();
		List<List<double[]>> contact = new ArrayList<>();
		for (int f = 0; f < 2; f++) {
			List<double[]> pts = new ArrayList<>();
			for (double[] pt : points[f])
				if (pt[2] < ground + CONTACT)
					pts.add(new double[] { pt[0] * CM, pt[1] * CM });
			contact.add(pts);
			if (!pts.isEmpty())
				standing.add(f);
		}
		// Sin apoyos, o con el cuerpo "cayendo" más rápido que la gravedad
		// (ruido de las derivadas): el suelo no puede tirar hacia abajo
		if (standing.isEmpty() || total[2] <= 1e-6)
			return "en el aire";

		// 2) Centro de presión que equilibra el momento
		double pz = ground * CM;
		double[] cop = { (pz * total[0] - needed[1]) / total[2], (needed[0] + pz * total[1]) / total[2] };

		// 3) Dentro de la zona de apoyo
		List<double[]> area = new ArrayList<>();
		for (int f : standing)
			area.addAll(widen(contact.get(f), points[f]));
		cop = closestInside(convexHull(area), cop);
		double[] p = { cop[0], cop[1], pz };
		double tz = needed[2] - cross(p, total)[2];

		if (standing.size() == 1) {
			Segment foot = feet[standing.get(0)];
			groundForce.put(foot, total);
			pressureCenter.put(foot, p);
			groundMoment.put(foot, new double[] { 0, 0, tz });
			return standing.get(0) == 0 ? "pie D" : "pie I";
		}

		// 4) Dos pies: regla de la palanca entre los centros de los dos apoyos
		double[] c0 = average(contact.get(0)), c1 = average(contact.get(1));
		double[] d = { c1[0] - c0[0], c1[1] - c0[1] };
		double len2 = d[0] * d[0] + d[1] * d[1];
		double t = len2 < 1e-12 ? 0.5 : ((cop[0] - c0[0]) * d[0] + (cop[1] - c0[1]) * d[1]) / len2;
		t = Math.max(0, Math.min(1, t));
		// Lo que el COP se separa de la línea entre los dos pies se le suma a
		// los dos puntos de aplicación: así (1 - t)·p0 + t·p1 = COP exactamente
		double[] off = { cop[0] - (c0[0] + t * d[0]), cop[1] - (c0[1] + t * d[1]) };
		groundForce.put(feet[0], scale(total, 1 - t));
		groundForce.put(feet[1], scale(total, t));
		pressureCenter.put(feet[0], new double[] { c0[0] + off[0], c0[1] + off[1], pz });
		pressureCenter.put(feet[1], new double[] { c1[0] + off[0], c1[1] + off[1], pz });
		groundMoment.put(feet[0], new double[] { 0, 0, tz * (1 - t) });
		groundMoment.put(feet[1], new double[] { 0, 0, tz * t });
		return "los dos pies";
	}

	/**
	 * Los puntos de contacto de un pie son su talón y su punta, que están en
	 * una línea; pero el pie tiene anchura. Cada punto se desdobla en dos,
	 * FOOT_HALF_WIDTH a cada lado, perpendicular a la dirección del pie.
	 */
	private static List<double[]> widen(List<double[]> contact, double[][] footPoints) {
		double[] dir = { footPoints[1][0] - footPoints[0][0], footPoints[1][1] - footPoints[0][1] };
		double len = Math.hypot(dir[0], dir[1]);
		double[] side = len < 1e-9 ? new double[] { 1, 0 } : new double[] { -dir[1] / len, dir[0] / len };
		List<double[]> out = new ArrayList<>();
		for (double[] q : contact)
			for (int sign : new int[] { -1, 1 })
				out.add(new double[] { q[0] + sign * side[0] * FOOT_HALF_WIDTH, q[1] + sign * side[1] * FOOT_HALF_WIDTH });
		return out;
	}

	/**
	 * Envolvente convexa (el "polígono de apoyo") de unos puntos en el plano,
	 * con el algoritmo de la cadena monótona de Andrew:
	 * 1. Ordenar los puntos por x (y por y si empatan): O(n log n).
	 * 2. Recorrerlos de izquierda a derecha construyendo la mitad de abajo, y
	 * de derecha a izquierda la de arriba. Cada punto nuevo se añade a una PILA;
	 * mientras los tres últimos giren hacia el lado que no toca (producto
	 * vectorial <= 0), se saca el del medio (pop), porque queda dentro.
	 * Es como imaginarse una goma elástica alrededor de los puntos.
	 *
	 * @return vértices en sentido antihorario
	 */
	static List<double[]> convexHull(List<double[]> pts) {
		List<double[]> sorted = new ArrayList<>(pts);
		sorted.sort((a, b) -> a[0] != b[0] ? Double.compare(a[0], b[0]) : Double.compare(a[1], b[1]));
		if (sorted.size() < 3)
			return sorted;
		List<double[]> hull = new ArrayList<>(); // se usa como pila: se añade y se quita por el final
		for (int pass = 0; pass < 2; pass++) {
			int start = hull.size();
			for (double[] q : sorted) {
				while (hull.size() >= start + 2 && turn(hull.get(hull.size() - 2), hull.get(hull.size() - 1), q) <= 0)
					hull.remove(hull.size() - 1);
				hull.add(q);
			}
			hull.remove(hull.size() - 1); // el último es el primero de la otra mitad
			java.util.Collections.reverse(sorted);
		}
		return hull;
	}

	/** Producto vectorial en 2D de (b - a) y (c - b): positivo si a -> b -> c gira a la izquierda. */
	private static double turn(double[] a, double[] b, double[] c) {
		return (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0]);
	}

	/**
	 * El punto p si está dentro del polígono convexo; si no, el punto más
	 * cercano de su borde. Dentro = a la izquierda de todos los lados (los
	 * vértices van en sentido antihorario).
	 */
	static double[] closestInside(List<double[]> poly, double[] p) {
		int n = poly.size();
		if (n == 1)
			return poly.get(0).clone();
		boolean inside = n >= 3;
		for (int i = 0; i < n && inside; i++)
			if (turn(poly.get(i), poly.get((i + 1) % n), p) < 0)
				inside = false;
		if (inside)
			return p;
		double[] best = null;
		double bestDist = Double.MAX_VALUE;
		for (int i = 0; i < n; i++) {
			double[] q = closestOnSegment(poly.get(i), poly.get((i + 1) % n), p);
			double dist = Math.hypot(q[0] - p[0], q[1] - p[1]);
			if (dist < bestDist) {
				bestDist = dist;
				best = q;
			}
		}
		return best;
	}

	/** Punto del segmento a-b más cercano a p (proyección, recortada a los extremos). */
	private static double[] closestOnSegment(double[] a, double[] b, double[] p) {
		double dx = b[0] - a[0], dy = b[1] - a[1];
		double len2 = dx * dx + dy * dy;
		double t = len2 < 1e-12 ? 0 : Math.max(0, Math.min(1, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / len2));
		return new double[] { a[0] + t * dx, a[1] + t * dy };
	}

	private static double[] average(List<double[]> pts) {
		double x = 0, y = 0;
		for (double[] q : pts) {
			x += q[0];
			y += q[1];
		}
		return new double[] { x / pts.size(), y / pts.size() };
	}

	/** Centro de masas del segmento en metros, en la postura de esos frames. */
	private double[] comOf(Segment s, Map<Segment, Matrix4> frames) {
		double along = comFraction(s) * s.getLength();
		return scale(frames.get(s).transformPoint(new double[] { 0, 0, along }), CM);
	}

	/** Articulación de arriba del segmento (origen de su frame), en metros. */
	private static double[] position(Matrix4 frame) {
		return new double[] { frame.tx() * CM, frame.ty() * CM, frame.tz() * CM };
	}

	/**
	 * Para el mapa de color: las regiones sin articulación que mostrar toman
	 * el esfuerzo de la de al lado (la zona de la pelvis, el de la lumbar; la
	 * "Cadera" de unión, el de la articulación de la cadera; la clavícula, el
	 * del hombro).
	 */
	private Segment neighbour(Segment s) {
		String base = baseName(s);
		if (base.equals("Pelvis"))
			return find("Lumbar");
		String side = s.getName().substring(s.getName().length() - 2);
		return switch (base) {
		case "Cadera" -> find("Muslo" + side);
		case "Clavícula" -> find("Brazo" + side);
		default -> s;
		};
	}

	private Segment find(String name) {
		for (Segment s : segments)
			if (s.getName().equals(name))
				return s;
		return null;
	}

	/** "Brazo D" -> "Brazo". */
	private static String baseName(Segment s) {
		String n = s.getName();
		return n.endsWith(" D") || n.endsWith(" I") ? n.substring(0, n.length() - 2) : n;
	}
}

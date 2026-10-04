package holograma.dynamics;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import holograma.kinematics.Segment;

/**
 * Exoesqueleto virtual de piernas: motores en cadera, rodilla y/o tobillo que
 * ayudan con parte del par de flexión/extensión.
 *
 * <h2>El controlador: asistencia proporcional</h2>
 * Cada motor da un porcentaje fijo del par que necesita su articulación
 * (calculado con la dinámica inversa), con un tope: el par máximo del motor.
 *
 * <pre>
 * par_motor = recortar(asistencia · par_necesario, -par_max, +par_max)
 * par_persona = par_necesario - par_motor
 * </pre>
 *
 * Es uno de los controladores más usados en exoesqueletos de investigación
 * (con señales de EMG o de sensores en vez de la dinámica inversa, que en la
 * vida real no se conoce de antemano). Implementa la interfaz
 * InverseDynamics.Assistance: la dinámica pregunta y esta clase contesta.
 *
 * <h2>El exo también pesa</h2>
 * Los motores y las barras se añaden como masas a los segmentos
 * (applyMasses), así que la dinámica inversa ya los tiene en cuenta: un motor
 * en el tobillo ayuda al tobillo, pero sus 1,5 kg cuelgan de la pierna y la
 * cadera tiene que moverlos en cada paso. Por eso los exos de verdad ponen
 * los motores lo más arriba posible y llevan la fuerza hacia abajo con cables.
 *
 * <h2>Qué se mide (para dimensionar los motores)</h2>
 * <ul>
 * <li>Par de pico: el motor (con su reductora) tiene que poder darlo.</li>
 * <li>Velocidad de pico (rpm en la articulación): la reductora tiene que
 * permitirla.</li>
 * <li>Potencia: par · velocidad. Positiva = el motor empuja a favor del
 * movimiento y gasta batería. Negativa = frena el movimiento (como al bajar
 * una escalera): un motor con regeneración podría recargar la batería.</li>
 * <li>Potencia media positiva: la que saca la batería de media. Con un
 * rendimiento motor + reductora de un 60%, da la batería necesaria.</li>
 * </ul>
 */
public class Exoskeleton implements InverseDynamics.Assistance {

	/** Rendimiento aproximado de motor + reductora, para estimar la batería. */
	public static final double EFFICIENCY = 0.6;
	/** Masa de cada barra (muslo o pierna), en kg. */
	public static final double LINK_MASS = 0.4;

	private boolean hip = true, knee = true, ankle;
	private double assist = 0.5; // 0 a 1
	private double maxTorque = 60; // N·m
	private double motorMass = 1.5; // kg por motor

	/** Estadísticas de un motor desde el último reinicio. */
	public static final class Stats {
		public double peakTorque, peakPower, peakRpm;
		public double positiveEnergy, negativeEnergy; // J
	}

	private final Map<String, Stats> stats = new LinkedHashMap<>(); // nombre del motor -> estadísticas
	private double time; // segundos acumulados
	private double sumNeeded, sumHuman; // para el "esfuerzo quitado"

	// ================================================================ configuración

	public void setJoints(boolean hip, boolean knee, boolean ankle) {
		this.hip = hip;
		this.knee = knee;
		this.ankle = ankle;
	}

	public void setAssist(double fraction) {
		assist = fraction;
	}

	public void setMaxTorque(double nm) {
		maxTorque = nm;
	}

	public void setMotorMass(double kg) {
		motorMass = kg;
	}

	public boolean hasHip() {
		return hip;
	}

	public boolean hasKnee() {
		return knee;
	}

	public boolean hasAnkle() {
		return ankle;
	}

	/** ¿Lleva motor la articulación de arriba de este segmento? */
	public boolean actuates(Segment s) {
		return switch (base(s)) {
		case "Muslo" -> hip; // la articulación de arriba del muslo es la cadera
		case "Tibia" -> knee;
		case "Pie" -> ankle;
		default -> false;
		};
	}

	/** El controlador (ver javadoc de la clase). */
	@Override
	public double torque(Segment s, double flexion) {
		if (!actuates(s))
			return 0;
		return Math.max(-maxTorque, Math.min(maxTorque, assist * flexion));
	}

	/**
	 * Pone las masas del exo en la dinámica inversa. Cada motor va en su
	 * articulación (al principio del segmento de abajo: fracción 0) y cada
	 * barra a mitad del segmento que acompaña. Hay barra en el muslo si hay
	 * motor en la cadera o en la rodilla, y en la pierna si lo hay en la
	 * rodilla o en el tobillo.
	 *
	 * @return masa total del exo (kg)
	 */
	public double applyMasses(InverseDynamics dynamics, List<Segment> segments) {
		dynamics.clearExtraMass();
		double total = 0;
		for (Segment s : segments) {
			String b = base(s);
			if (actuates(s)) {
				dynamics.addMass(s, motorMass, 0);
				total += motorMass;
			}
			boolean thighBar = b.equals("Muslo") && (hip || knee);
			boolean shinBar = b.equals("Tibia") && (knee || ankle);
			if (thighBar || shinBar) {
				dynamics.addMass(s, LINK_MASS, 0.5);
				total += LINK_MASS;
			}
		}
		return total;
	}

	// ================================================================ estadísticas

	/**
	 * Apunta un cálculo de la dinámica en las estadísticas.
	 *
	 * @param dt segundos que representa este cálculo (0 si la animación está
	 *           parada: entonces cuentan los picos, pero no la energía)
	 */
	public void record(InverseDynamics.Result r, double dt) {
		time += dt;
		for (InverseDynamics.Joint j : r.joints()) {
			if (!actuates(j.segment()))
				continue;
			Stats st = stats.computeIfAbsent(j.name(), k -> new Stats());
			st.peakTorque = Math.max(st.peakTorque, Math.abs(j.exoTorque()));
			st.peakPower = Math.max(st.peakPower, Math.abs(j.exoPower()));
			// rad/s -> vueltas por minuto: una vuelta son 2π rad y un minuto 60 s
			st.peakRpm = Math.max(st.peakRpm, Math.abs(j.speed()) * 60 / (2 * Math.PI));
			// Energía = potencia · tiempo, separando lo que empuja de lo que frena
			if (j.exoPower() > 0)
				st.positiveEnergy += j.exoPower() * dt;
			else
				st.negativeEnergy -= j.exoPower() * dt;
			sumNeeded += j.torque() * dt;
			sumHuman += j.human() * dt;
		}
	}

	public void resetStats() {
		stats.clear();
		time = 0;
		sumNeeded = sumHuman = 0;
	}

	public Map<String, Stats> getStats() {
		return stats;
	}

	/** Potencia mecánica media que dan los motores empujando (W). */
	public double meanPositivePower() {
		if (time <= 0)
			return 0;
		double e = 0;
		for (Stats st : stats.values())
			e += st.positiveEnergy;
		return e / time;
	}

	/** Potencia media que podría recuperarse frenando (W). */
	public double meanNegativePower() {
		if (time <= 0)
			return 0;
		double e = 0;
		for (Stats st : stats.values())
			e += st.negativeEnergy;
		return e / time;
	}

	/**
	 * Cuánto par le quita el exo a la persona en las articulaciones con motor,
	 * de media en el tiempo (0 a 1). Es una medida sencilla; los estudios de
	 * verdad miden el consumo metabólico (oxígeno), que no es lo mismo.
	 */
	public double reduction() {
		return sumNeeded <= 0 ? 0 : 1 - sumHuman / sumNeeded;
	}

	public double getTime() {
		return time;
	}

	private static String base(Segment s) {
		String n = s.getName();
		return n.endsWith(" D") || n.endsWith(" I") ? n.substring(0, n.length() - 2) : n;
	}
}

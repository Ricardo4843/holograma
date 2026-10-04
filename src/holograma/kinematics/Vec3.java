package holograma.kinematics;

/**
 * Operaciones con vectores 3D guardados como arrays de 3 doubles {x, y, z}.
 *
 * Antes estaban dentro de HumanSkeleton (en su clase Builder), pero la
 * captura de movimiento (paquete mocap) también las necesita, así que se han
 * sacado a una clase propia. Es una clase de utilidades: todo es estático y no
 * se crean objetos Vec3 (el constructor privado lo impide).
 */
public final class Vec3 {

	private Vec3() {
	}

	public static double[] add(double[] a, double[] b) {
		return new double[] { a[0] + b[0], a[1] + b[1], a[2] + b[2] };
	}

	public static double[] sub(double[] a, double[] b) {
		return new double[] { a[0] - b[0], a[1] - b[1], a[2] - b[2] };
	}

	public static double[] scale(double[] a, double k) {
		return new double[] { a[0] * k, a[1] * k, a[2] * k };
	}

	/** Producto escalar: |a||b|cos(ángulo). Vale 0 si son perpendiculares. */
	public static double dot(double[] a, double[] b) {
		return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
	}

	/** Producto vectorial: vector perpendicular a a y a b (regla de la mano derecha). */
	public static double[] cross(double[] a, double[] b) {
		return new double[] { a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0] };
	}

	public static double norm(double[] a) {
		return Math.sqrt(dot(a, a));
	}

	/** Mismo vector con longitud 1. */
	public static double[] normalize(double[] a) {
		return scale(a, 1 / norm(a));
	}

	/**
	 * Parte de {@code a} perpendicular a {@code unit} (que debe medir 1): se le
	 * quita su componente en esa dirección. Es el paso de Gram-Schmidt.
	 */
	public static double[] perpendicular(double[] a, double[] unit) {
		return sub(a, scale(unit, dot(a, unit)));
	}
}

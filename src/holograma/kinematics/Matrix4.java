package holograma.kinematics;

/**
 * Matriz de transformación homogénea 4x4: guarda a la vez una ROTACIÓN y una
 * TRASLACIÓN en 3D.
 *
 * <h2>Por qué hace falta (comparación con el lab2)</h2>
 * En el lab2 (2D) cada segmento acumulaba un ángulo: angle + accumulatedAngle.
 * Eso funciona porque en 2D todas las rotaciones son alrededor del mismo eje
 * (el que sale de la pantalla), y girar 30º y luego 20º es lo mismo que girar
 * 20º y luego 30º: las rotaciones conmutan y basta con sumar.
 *
 * En 3D NO conmutan. Prueba con un móvil: gíralo 90º hacia delante y luego 90º
 * hacia la derecha; después repite en el orden contrario. Acaba en posiciones
 * distintas. Por eso ya no se pueden sumar ángulos y hay que guardar la
 * orientación completa en una matriz y MULTIPLICAR matrices.
 *
 * <h2>Por qué 4x4 y no 3x3</h2>
 * Una matriz 3x3 puede representar una rotación, pero no una traslación (mover
 * un punto sin girarlo). El truco de las "coordenadas homogéneas" es añadir
 * una cuarta coordenada que vale siempre 1: el punto (x, y, z) se escribe como
 * (x, y, z, 1). Así, una sola multiplicación de matriz 4x4 por vector aplica
 * la rotación y suma la traslación a la vez:
 *
 * <pre>
 * | r00 r01 r02 tx |   | x |   | r00*x + r01*y + r02*z + tx |
 * | r10 r11 r12 ty | * | y | = | r10*x + r11*y + r12*z + ty |
 * | r20 r21 r22 tz |   | z |   | r20*x + r21*y + r22*z + tz |
 * | 0   0   0   1  |   | 1 |   | 1                          |
 * </pre>
 *
 * Y lo mejor: encadenar transformaciones (girar, luego avanzar, luego volver
 * a girar...) es simplemente multiplicar sus matrices. Es exactamente lo que
 * hace la cinemática directa: ir multiplicando desde la raíz hasta cada
 * segmento. Esto se ve en Robótica con los parámetros de Denavit-Hartenberg.
 *
 * <h2>Cómo se guarda</h2>
 * La última fila siempre es [0 0 0 1], así que no se guarda: solo hay 12
 * números, ordenados por filas en un array:
 *
 * <pre>
 * | m[0] m[1]  m[2]  m[3]  |   m[3], m[7], m[11] = traslación (x, y, z)
 * | m[4] m[5]  m[6]  m[7]  |   el bloque 3x3 de la izquierda = rotación
 * | m[8] m[9]  m[10] m[11] |
 * </pre>
 *
 * Interpretación útil: las COLUMNAS del bloque de rotación son los ejes X, Y,
 * Z locales del segmento vistos desde el mundo. Por ejemplo, la columna 2
 * (m[2], m[6], m[10]) es hacia dónde apunta el segmento, porque por convenio
 * cada segmento se extiende a lo largo de su Z local.
 *
 * La clase es inmutable (final y sin setters): cada operación devuelve una
 * matriz nueva en vez de modificar esta. Así es imposible que un segmento
 * cambie sin querer la matriz de otro que la comparte.
 */
public final class Matrix4 {

	// Los 12 valores de la matriz, por filas. Visible en el paquete (sin
	// modificador) para no tener que copiarlo en cada multiplicación.
	final double[] m;

	// Constructor privado: las matrices se crean con los métodos estáticos de
	// abajo (identity, translation, rotX...), que dejan más claro qué es cada una.
	private Matrix4(double[] m) {
		this.m = m;
	}

	/**
	 * Matriz identidad: no gira ni mueve nada (es el "1" de las matrices).
	 * Multiplicar cualquier matriz por la identidad la deja igual.
	 */
	public static Matrix4 identity() {
		return new Matrix4(new double[] {
				1, 0, 0, 0,
				0, 1, 0, 0,
				0, 0, 1, 0 });
	}

	/**
	 * Traslación pura: mueve los puntos (x, y, z) sin girarlos. La rotación es
	 * la identidad y la última columna es el desplazamiento.
	 */
	public static Matrix4 translation(double x, double y, double z) {
		return new Matrix4(new double[] {
				1, 0, 0, x,
				0, 1, 0, y,
				0, 0, 1, z });
	}

	/**
	 * Rotación de {@code a} radianes alrededor del eje X.
	 *
	 * El eje X no se mueve (primera fila y columna son 1, 0, 0). Lo que gira es
	 * el plano Y-Z, con la misma fórmula que la rotación 2D del lab2:
	 * y' = y*cos(a) - z*sin(a) z' = y*sin(a) + z*cos(a)
	 *
	 * Sentido: regla de la mano derecha. Si el pulgar apunta hacia +X, los dedos
	 * se cierran en el sentido de los ángulos positivos.
	 */
	public static Matrix4 rotX(double a) {
		double c = Math.cos(a), s = Math.sin(a);
		return new Matrix4(new double[] {
				1, 0, 0, 0,
				0, c, -s, 0,
				0, s, c, 0 });
	}

	/**
	 * Rotación de {@code a} radianes alrededor del eje Y. El plano que gira es
	 * Z-X (fíjate en que el signo menos está abajo, no arriba: es por el orden
	 * cíclico X -> Y -> Z -> X).
	 *
	 * Ejemplo útil: rotY(90º) lleva el eje Z (0, 0, 1) al eje X (1, 0, 0). Se
	 * usa en HumanSkeleton para que caderas y clavículas apunten hacia los lados.
	 */
	public static Matrix4 rotY(double a) {
		double c = Math.cos(a), s = Math.sin(a);
		return new Matrix4(new double[] {
				c, 0, s, 0,
				0, 1, 0, 0,
				-s, 0, c, 0 });
	}

	/** Rotación de {@code a} radianes alrededor del eje Z (gira el plano X-Y). */
	public static Matrix4 rotZ(double a) {
		double c = Math.cos(a), s = Math.sin(a);
		return new Matrix4(new double[] {
				c, -s, 0, 0,
				s, c, 0, 0,
				0, 0, 1, 0 });
	}

	/**
	 * Construye una rotación a partir de sus tres ejes locales vistos desde el
	 * mundo (vectores unitarios y perpendiculares entre sí). Como se explica
	 * arriba, esos ejes son justamente las COLUMNAS de la matriz, así que solo
	 * hay que colocarlos en vertical.
	 *
	 * Se usa en HumanSkeleton para orientar cada segmento según las
	 * articulaciones que se detectan en el modelo 3D del cuerpo.
	 */
	public static Matrix4 fromAxes(double[] x, double[] y, double[] z) {
		return new Matrix4(new double[] {
				x[0], y[0], z[0], 0,
				x[1], y[1], z[1], 0,
				x[2], y[2], z[2], 0 });
	}

	/**
	 * Producto de matrices: devuelve this * o.
	 *
	 * Cuidado con el orden. Al aplicarse a un punto, (this * o) * p = this * (o *
	 * p), es decir, primero se aplica {@code o} y después {@code this}. En la
	 * cinemática se usa así: padre.multiply(local) = "dentro del sistema del
	 * padre, aplica la transformación local del hijo".
	 *
	 * Cada elemento del resultado es fila i de A por columna j de B. Como la
	 * cuarta fila implícita es [0 0 0 1], las cuentas se simplifican:
	 * - En las columnas de rotación (j = 0, 1, 2) la cuarta fila de B vale 0,
	 *   así que solo se suman 3 productos.
	 * - En la columna de traslación (j = 3) el último término es a[fila+3] * 1.
	 *
	 * Coste: 36 multiplicaciones, constante. Por eso la cinemática directa
	 * completa es O(n) con n = número de segmentos.
	 */
	public Matrix4 multiply(Matrix4 o) {
		double[] a = m, b = o.m, r = new double[12];
		for (int i = 0; i < 3; i++) {
			int row = i * 4; // índice donde empieza la fila i en el array
			// Columnas 0, 1 y 2 (parte de rotación)
			for (int j = 0; j < 3; j++)
				r[row + j] = a[row] * b[j] + a[row + 1] * b[4 + j] + a[row + 2] * b[8 + j];
			// Columna 3 (traslación): igual, más la traslación propia de A
			r[row + 3] = a[row] * b[3] + a[row + 1] * b[7] + a[row + 2] * b[11] + a[row + 3];
		}
		return new Matrix4(r);
	}

	/**
	 * Inversa de una transformación rígida (rotación + traslación, sin escalar).
	 *
	 * La inversa general de una 4x4 es costosa, pero aquí hay un atajo. Una
	 * matriz de rotación R es ORTOGONAL: su inversa es su transpuesta (R^-1 =
	 * R^T), porque deshacer un giro es girar lo mismo en sentido contrario. Para
	 * la traslación: si M lleva p a R*p + t, para volver hay que restar t y
	 * luego desgirar, R^T * (p - t) = R^T * p - R^T * t. Por eso:
	 *
	 * <pre>
	 * M^-1 = | R^T   -R^T * t |
	 * </pre>
	 *
	 * Se usa en el skinning del holograma (PointCloud) para "quitar" la postura
	 * de reposo antes de aplicar la postura actual.
	 */
	public Matrix4 rigidInverse() {
		double[] r = new double[12];
		// Transponer la rotación: el elemento (i, j) pasa a ser el (j, i)
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				r[i * 4 + j] = m[j * 4 + i];
		// Nueva traslación = -R^T * t (fila i de R^T por el vector t)
		for (int i = 0; i < 3; i++)
			r[i * 4 + 3] = -(r[i * 4] * m[3] + r[i * 4 + 1] * m[7] + r[i * 4 + 2] * m[11]);
		return new Matrix4(r);
	}

	/** Coordenada X de la traslación: dónde está el origen de este sistema. */
	public double tx() {
		return m[3];
	}

	/** Coordenada Y de la traslación. */
	public double ty() {
		return m[7];
	}

	/** Coordenada Z de la traslación. */
	public double tz() {
		return m[11];
	}

	/**
	 * Copia de los 12 valores (por filas). Es una copia (clone) y no el array
	 * original para que nadie pueda modificar la matriz desde fuera y romper la
	 * inmutabilidad.
	 */
	public double[] toArray() {
		return m.clone();
	}
}

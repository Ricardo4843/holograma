package holograma.mocap;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketTimeoutException;
import java.nio.charset.StandardCharsets;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

import holograma.body.JsonParser;

/**
 * Recibe en directo las posturas que manda webcam/pose_sender.py (MediaPipe)
 * por la red local.
 *
 * <h2>UDP</h2>
 * Cada postura llega en un "datagrama" UDP: un paquete suelto con un texto
 * JSON dentro, {"t": segundos, "lm": [[x, y, z, visibilidad], ... 33 puntos]}.
 * UDP no garantiza que lleguen todos ni en orden (TCP sí), pero para vídeo en
 * directo es justo lo que interesa: si se pierde una postura da igual, llega
 * otra 1/30 s después, y no hay que esperar a reenviar nada.
 *
 * <h2>Un hilo aparte</h2>
 * socket.receive() se queda BLOQUEADO hasta que llega un paquete. Si se
 * hiciera en el hilo de JavaFX, la ventana se congelaría. Por eso se escucha
 * en un hilo propio, que deja la última postura en una AtomicReference; el
 * hilo de JavaFX la recoge en cada fotograma.
 *
 * AtomicReference hace que pasar el objeto de un hilo a otro sea seguro: sin
 * ella, Java no garantiza que un hilo vea a tiempo (ni entera) una variable
 * que ha escrito otro. Como la postura es inmutable (un record con un array
 * que nadie modifica después), no hace falta nada más.
 */
public class LiveReceiver {

	public static final int DEFAULT_PORT = 5005;
	public static final int LANDMARKS = 33; // puntos del cuerpo de MediaPipe Pose

	/**
	 * Una postura recibida.
	 *
	 * @param landmarks [33][4]: x, y, z (metros) y visibilidad (0 a 1)
	 * @param seq       número de orden (para saber si es nueva)
	 * @param nanos     cuándo llegó (System.nanoTime)
	 */
	public record Pose(double[][] landmarks, long seq, long nanos) {
	}

	private final int port;
	private final AtomicReference<Pose> latest = new AtomicReference<>();
	private volatile boolean running; // volatile: el otro hilo ve el cambio enseguida
	private volatile String error;
	private Thread thread;
	private long seq;

	public LiveReceiver(int port) {
		this.port = port;
	}

	/** Empieza a escuchar (si no lo estaba ya). */
	public synchronized void start() {
		if (running)
			return;
		running = true;
		error = null;
		thread = new Thread(this::listen, "receptor-webcam");
		// Daemon: no impide que el programa se cierre al cerrar la ventana
		thread.setDaemon(true);
		thread.start();
	}

	public synchronized void stop() {
		running = false;
	}

	/** Última postura recibida, o null si todavía no ha llegado ninguna. */
	public Pose latest() {
		return latest.get();
	}

	/** Mensaje de error (por ejemplo, si el puerto está ocupado), o null. */
	public String getError() {
		return error;
	}

	public int getPort() {
		return port;
	}

	/** Bucle del hilo receptor. */
	private void listen() {
		// try-with-resources: el socket se cierra solo al salir del bloque
		try (DatagramSocket socket = new DatagramSocket(port, InetAddress.getLoopbackAddress())) {
			// Cada medio segundo receive() se despierta aunque no llegue nada, para
			// poder comprobar si hay que parar
			socket.setSoTimeout(500);
			byte[] buffer = new byte[65536];
			while (running) {
				DatagramPacket packet = new DatagramPacket(buffer, buffer.length);
				try {
					socket.receive(packet);
				} catch (SocketTimeoutException e) {
					continue;
				}
				String json = new String(packet.getData(), 0, packet.getLength(), StandardCharsets.UTF_8);
				try {
					latest.set(new Pose(parse(json), ++seq, System.nanoTime()));
				} catch (RuntimeException e) {
					// Un paquete mal formado no debe tumbar el receptor: se ignora
				}
			}
		} catch (Exception e) {
			error = "No se puede escuchar en el puerto " + port + ": " + e.getMessage();
			running = false;
		}
	}

	/** Pasa el JSON a un array [33][4], con el lector de JSON del paquete body. */
	@SuppressWarnings("unchecked")
	private static double[][] parse(String json) {
		Map<String, Object> obj = (Map<String, Object>) JsonParser.parse(json);
		List<Object> lm = (List<Object>) obj.get("lm");
		if (lm.size() != LANDMARKS)
			throw new IllegalArgumentException("Se esperaban " + LANDMARKS + " puntos y hay " + lm.size());
		double[][] out = new double[LANDMARKS][4];
		for (int i = 0; i < LANDMARKS; i++) {
			List<Object> p = (List<Object>) lm.get(i);
			for (int k = 0; k < 4; k++)
				out[i][k] = ((Number) p.get(k)).doubleValue();
		}
		return out;
	}
}

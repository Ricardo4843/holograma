package holograma.kinematics;

import java.util.ArrayList;
import java.util.List;

/**
 * Resultado de la cinemática directa para un segmento: dónde está en el mundo.
 *
 * Es el Node del lab2 con dos diferencias:
 * <ul>
 * <li>Tiene coordenada z.</li>
 * <li>Además de la posición del extremo, guarda el "frame" (sistema de
 * referencia) completo del segmento: posición de la articulación de inicio +
 * orientación de sus ejes locales. Con solo la posición del extremo se puede
 * dibujar una línea, pero para mover la piel (los puntos del holograma) hay que saber
 * también hacia dónde está girada.</li>
 * </ul>
 *
 * Separar Segment (datos relativos: longitud y ángulos respecto al padre) de
 * Node3D (datos absolutos: coordenadas en el mundo) es la misma decisión de
 * diseño del lab2. El modelo dice CÓMO es el esqueleto y el resultado del
 * cálculo dice DÓNDE está ahora.
 *
 * También es un tipo recursivo (contiene una lista de Node3D).
 */
public class Node3D {

	private final Segment segment; // segmento del que sale este nodo
	private final Matrix4 frame; // sistema de referencia en la articulación de inicio
	private final double x, y, z; // posición absoluta del extremo del segmento
	private final List<Node3D> children = new ArrayList<>();

	public Node3D(Segment segment, Matrix4 frame, double x, double y, double z) {
		this.segment = segment;
		this.frame = frame;
		this.x = x;
		this.y = y;
		this.z = z;
	}

	public void addChild(Node3D child) {
		children.add(child);
	}

	public Segment getSegment() {
		return segment;
	}

	/** Matriz que lleva coordenadas locales del segmento a coordenadas del mundo. */
	public Matrix4 getFrame() {
		return frame;
	}

	public double getX() {
		return x;
	}

	public double getY() {
		return y;
	}

	public double getZ() {
		return z;
	}

	public List<Node3D> getChildren() {
		return children;
	}
}

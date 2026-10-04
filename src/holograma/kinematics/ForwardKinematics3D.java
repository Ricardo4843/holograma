package holograma.kinematics;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.IdentityHashMap;
import java.util.Map;

/**
 * Cinemática directa en 3D: a partir de los ángulos de todas las
 * articulaciones, calcula dónde está cada segmento en el mundo.
 *
 * La idea es la del lab2, pero lo que se pasa de padre a hijo es una matriz
 * 4x4 en vez de (x, y, ángulo acumulado):
 *
 * <pre>
 * lab2:  angle = link.getAngle() + accumulatedAngle;
 *        x = baseX + length * cos(angle);   y = baseY + length * sin(angle);
 *
 * 3D:    frame = finalDelPadre * transformaciónLocal;
 *        final = frame * traslación(0, 0, longitud);
 * </pre>
 *
 * "frame" hace el papel del ángulo acumulado (la orientación) y del punto
 * base, todo junto. "final" es el punto donde acaba el segmento, ya con la
 * orientación que heredan sus hijos.
 *
 * Hay dos versiones del algoritmo:
 * <ul>
 * <li>RECURSIVA: la del lab2. Para un esqueleto humano (21 segmentos) es
 * perfecta y la más fácil de leer.</li>
 * <li>ITERATIVA con pila explícita: da el mismo resultado sin recursión. Hace
 * falta si el árbol es muy PROFUNDO (por ejemplo, una cadena de 10.000
 * segmentos, cada uno hijo del anterior). Cada llamada recursiva ocupa un
 * hueco ("marco") en la pila de llamadas de Java, que tiene un tamaño
 * limitado (unos cientos de KB a 1 MB por hilo). Con unos miles de niveles se
 * llena y salta StackOverflowError. La versión iterativa guarda el trabajo
 * pendiente en una estructura de datos propia (en el heap, que es mucho más
 * grande). El benchmark de la etapa 2 (repo exoesqueleto) lo demuestra.</li>
 * </ul>
 *
 * Complejidad de ambas: O(n), porque cada segmento se visita exactamente una
 * vez y en cada visita se hace un trabajo constante (unas pocas
 * multiplicaciones de matrices).
 */
public class ForwardKinematics3D {

	/**
	 * Versión recursiva: método público "fachada", como en el lab2. Quien lo
	 * usa solo da la raíz y el origen. Los parámetros extra que necesita la
	 * recursión los rellena el método privado.
	 *
	 * @param root Raíz del árbol de segmentos.
	 * @param ox   Coordenada X del origen (posición de la articulación raíz).
	 * @param oy   Coordenada Y del origen.
	 * @param oz   Coordenada Z del origen.
	 * @return Raíz del árbol de nodos con las posiciones absolutas.
	 */
	public static Node3D computePositions(Segment root, double ox, double oy, double oz) {
		// La "matriz del final del padre" de la raíz es simplemente el origen:
		// una traslación pura, sin rotación.
		return computePositions(root, Matrix4.translation(ox, oy, oz));
	}

	/**
	 * Método recursivo.
	 *
	 * @param seg       Segmento a procesar.
	 * @param parentEnd Transformación del extremo del padre: dónde empieza este
	 *                  segmento y con qué orientación acumulada.
	 */
	private static Node3D computePositions(Segment seg, Matrix4 parentEnd) {
		// Código común (1): sistema de referencia de este segmento. Es la del
		// padre multiplicada por la local del hijo (base + giros de la
		// articulación). Equivale a "angle = link.getAngle() + accumulatedAngle".
		Matrix4 frame = parentEnd.multiply(seg.localTransform());

		// Código común (2): extremo del segmento. Se avanza "length" a lo largo
		// del eje Z LOCAL (ya girado). Equivale a x = baseX + length*cos(angle)...
		Matrix4 end = frame.multiply(Matrix4.translation(0, 0, seg.getLength()));

		// Código común (3): nodo con el resultado. La posición del extremo es la
		// columna de traslación de "end".
		Node3D node = new Node3D(seg, frame, end.tx(), end.ty(), end.tz());

		// Caso recursivo: cada hijo empieza donde acaba este segmento y hereda
		// su orientación ("end"); su nodo se cuelga de este.
		// Caso base implícito: si el segmento no tiene hijos (una mano, un pie),
		// el bucle no se ejecuta y se devuelve el nodo sin hijos. En el lab2 el
		// caso base era un if explícito; el resultado es el mismo.
		for (Segment child : seg.getChildren())
			node.addChild(computePositions(child, end));
		return node;
	}

	/**
	 * Versión iterativa: mismo resultado que la recursiva, sin recursión.
	 *
	 * La recursión usa (sin que se vea) la pila de llamadas de Java para
	 * recordar qué le falta por hacer. Aquí esa pila se hace a mano: una Deque
	 * usada como pila (push mete arriba, pop saca de arriba, LIFO). Cada
	 * elemento es un trabajo pendiente: "procesa este segmento, que empieza en
	 * esta matriz y cuyo nodo hay que colgar de este padre".
	 *
	 * Diferencia menor: los hijos salen de la pila en orden inverso (el último
	 * en entrar es el primero en salir), así que el árbol de nodos tiene los
	 * hijos en otro orden. Las posiciones son idénticas.
	 */
	public static Node3D computePositionsIterative(Segment root, double ox, double oy, double oz) {
		// "record": clase inmutable de una línea (Java 16+). Agrupa los tres
		// datos de un trabajo pendiente; el compilador genera el constructor,
		// los getters seg(), parentEnd() y parentNode(), y equals/hashCode/toString.
		record Pending(Segment seg, Matrix4 parentEnd, Node3D parentNode) {
		}

		// ArrayDeque es más rápida que la clase antigua Stack (que está
		// sincronizada y hereda de Vector).
		Deque<Pending> stack = new ArrayDeque<>();
		stack.push(new Pending(root, Matrix4.translation(ox, oy, oz), null));
		Node3D rootNode = null;

		while (!stack.isEmpty()) {
			Pending p = stack.pop();
			// Mismo "código común" que la versión recursiva
			Matrix4 frame = p.parentEnd().multiply(p.seg().localTransform());
			Matrix4 end = frame.multiply(Matrix4.translation(0, 0, p.seg().getLength()));
			Node3D node = new Node3D(p.seg(), frame, end.tx(), end.ty(), end.tz());

			// La raíz no tiene padre: es el nodo que se devuelve al final
			if (p.parentNode() == null)
				rootNode = node;
			else
				p.parentNode().addChild(node);

			// En vez de llamar recursivamente para cada hijo, se apunta como
			// trabajo pendiente. El bucle while los irá sacando.
			for (Segment child : p.seg().getChildren())
				stack.push(new Pending(child, end, node));
		}
		return rootNode;
	}

	/**
	 * Recorre el árbol de nodos y devuelve un mapa segmento -> frame. Así, para
	 * saber dónde está un segmento concreto basta un get(), en O(1), en vez de
	 * buscarlo en el árbol.
	 *
	 * Se usa IdentityHashMap en vez de HashMap porque compara las claves con ==
	 * (mismo objeto), que es justo lo que se quiere: cada Segment es único. Es
	 * algo más rápido y no depende de equals/hashCode.
	 *
	 * Este recorrido también es iterativo con pila, por la misma razón que
	 * computePositionsIterative.
	 */
	public static Map<Segment, Matrix4> frames(Node3D root) {
		Map<Segment, Matrix4> map = new IdentityHashMap<>();
		Deque<Node3D> stack = new ArrayDeque<>();
		stack.push(root);
		while (!stack.isEmpty()) {
			Node3D n = stack.pop();
			map.put(n.getSegment(), n.getFrame());
			for (Node3D c : n.getChildren())
				stack.push(c);
		}
		return map;
	}
}

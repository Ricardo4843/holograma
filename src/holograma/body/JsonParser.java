package holograma.body;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Lector de JSON escrito desde cero, por DESCENSO RECURSIVO.
 *
 * Hace falta porque los pesos de MakeHuman (default_weights.mhw) vienen en
 * JSON y Java no trae un lector de JSON de serie. Y es un buen ejemplo de
 * recursividad, porque JSON es un formato RECURSIVO: un valor puede ser un
 * objeto o una lista, que a su vez contienen valores... igual que un Segment
 * contiene Segments.
 *
 * <h2>La gramática de JSON (simplificada)</h2>
 *
 * <pre>
 * valor  := objeto | lista | texto | número | true | false | null
 * objeto := '{' [ texto ':' valor { ',' texto ':' valor } ] '}'
 * lista  := '[' [ valor { ',' valor } ] ']'
 * </pre>
 *
 * El parser tiene un método por cada regla (parseValue, parseObject,
 * parseList...), y cada uno llama a los demás según lo que encuentra. Como
 * parseObject y parseList llaman a parseValue, que a su vez puede llamar a
 * parseObject o parseList, se dice que son mutuamente recursivos.
 * - Caso base: textos, números, true, false y null (no contienen nada más).
 * - Caso recursivo: objetos y listas (contienen otros valores).
 *
 * Resultado en Java: objeto -> Map, lista -> List, número -> Double, texto ->
 * String, true/false -> Boolean, null -> null.
 *
 * Limitación consciente: no interpreta escapes raros como á (los
 * ficheros de MakeHuman no los usan).
 */
public class JsonParser {

	private final String text;
	private int pos; // posición del siguiente carácter a leer

	private JsonParser(String text) {
		this.text = text;
	}

	/** Método público "fachada": lee un texto JSON completo. */
	public static Object parse(String text) {
		JsonParser p = new JsonParser(text);
		Object value = p.parseValue();
		p.skipSpaces();
		if (p.pos != text.length())
			throw p.error("sobra texto al final");
		return value;
	}

	/** valor := objeto | lista | texto | número | true | false | null */
	private Object parseValue() {
		skipSpaces();
		if (pos >= text.length())
			throw error("el texto se acaba antes de tiempo");
		// Se mira el primer carácter para saber qué regla aplicar. Eso es lo que
		// hace que sea tan simple: JSON está diseñado para que un solo carácter
		// baste para decidir.
		char c = text.charAt(pos);
		switch (c) {
		case '{':
			return parseObject(); // caso recursivo
		case '[':
			return parseList(); // caso recursivo
		case '"':
			return parseString(); // casos base a partir de aquí
		case 't':
			expect("true");
			return Boolean.TRUE;
		case 'f':
			expect("false");
			return Boolean.FALSE;
		case 'n':
			expect("null");
			return null;
		default:
			return parseNumber();
		}
	}

	/** objeto := '{' [ texto ':' valor { ',' texto ':' valor } ] '}' */
	private Map<String, Object> parseObject() {
		// LinkedHashMap mantiene las claves en el orden en que aparecen
		Map<String, Object> map = new LinkedHashMap<>();
		pos++; // salta '{'
		skipSpaces();
		if (peek() == '}') { // objeto vacío
			pos++;
			return map;
		}
		while (true) {
			skipSpaces();
			String key = parseString();
			skipSpaces();
			expect(":");
			map.put(key, parseValue()); // <- llamada recursiva
			skipSpaces();
			if (peek() == ',') {
				pos++;
				continue;
			}
			expect("}");
			return map;
		}
	}

	/** lista := '[' [ valor { ',' valor } ] ']' */
	private List<Object> parseList() {
		List<Object> list = new ArrayList<>();
		pos++; // salta '['
		skipSpaces();
		if (peek() == ']') { // lista vacía
			pos++;
			return list;
		}
		while (true) {
			list.add(parseValue()); // <- llamada recursiva
			skipSpaces();
			if (peek() == ',') {
				pos++;
				continue;
			}
			expect("]");
			return list;
		}
	}

	/** Texto entre comillas. Una barra invertida protege el carácter siguiente. */
	private String parseString() {
		expect("\"");
		StringBuilder sb = new StringBuilder(); // más eficiente que ir sumando Strings
		while (text.charAt(pos) != '"') {
			if (text.charAt(pos) == '\\')
				pos++;
			sb.append(text.charAt(pos++));
		}
		pos++; // salta la comilla de cierre
		return sb.toString();
	}

	/** Número: se avanza mientras haya caracteres válidos y se convierte con Double.parseDouble. */
	private Double parseNumber() {
		int start = pos;
		while (pos < text.length() && "+-0123456789.eE".indexOf(text.charAt(pos)) >= 0)
			pos++;
		if (start == pos)
			throw error("carácter inesperado '" + text.charAt(pos) + "'");
		return Double.parseDouble(text.substring(start, pos));
	}

	// ---- Utilidades ----

	private void skipSpaces() {
		while (pos < text.length() && Character.isWhitespace(text.charAt(pos)))
			pos++;
	}

	private char peek() {
		return pos < text.length() ? text.charAt(pos) : '\0';
	}

	/** Comprueba que lo siguiente es exactamente "s" y lo salta; si no, error. */
	private void expect(String s) {
		if (!text.startsWith(s, pos))
			throw error("se esperaba '" + s + "'");
		pos += s.length();
	}

	private IllegalArgumentException error(String msg) {
		return new IllegalArgumentException("JSON no válido en la posición " + pos + ": " + msg);
	}
}

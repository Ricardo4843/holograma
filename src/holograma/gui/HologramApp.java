package holograma.gui;

import java.awt.image.BufferedImage;
import java.io.File;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;

import javax.imageio.ImageIO;

import holograma.body.MakeHumanRig;
import holograma.body.PointCloud;
import holograma.kinematics.ForwardKinematics3D;
import holograma.kinematics.HumanSkeleton;
import holograma.kinematics.Matrix4;
import holograma.kinematics.Node3D;
import holograma.kinematics.Segment;
import javafx.animation.AnimationTimer;
import javafx.animation.PauseTransition;
import javafx.application.Application;
import javafx.application.Platform;
import javafx.geometry.Insets;
import javafx.scene.AmbientLight;
import javafx.scene.Group;
import javafx.scene.PerspectiveCamera;
import javafx.scene.PointLight;
import javafx.scene.Scene;
import javafx.scene.SceneAntialiasing;
import javafx.scene.SubScene;
import javafx.scene.control.Button;
import javafx.scene.control.CheckBox;
import javafx.scene.control.ComboBox;
import javafx.scene.control.Label;
import javafx.scene.control.Separator;
import javafx.scene.control.Slider;
import javafx.scene.control.ToggleButton;
import javafx.scene.effect.Bloom;
import javafx.scene.image.PixelReader;
import javafx.scene.image.WritableImage;
import javafx.scene.layout.Background;
import javafx.scene.layout.BackgroundFill;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.Pane;
import javafx.scene.layout.VBox;
import javafx.scene.paint.Color;
import javafx.scene.paint.CycleMethod;
import javafx.scene.paint.Paint;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.paint.RadialGradient;
import javafx.scene.paint.Stop;
import javafx.scene.shape.Box;
import javafx.scene.shape.Cylinder;
import javafx.scene.shape.MeshView;
import javafx.scene.shape.Sphere;
import javafx.scene.text.Font;
import javafx.scene.text.Text;
import javafx.scene.transform.Affine;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Translate;
import javafx.stage.Stage;
import javafx.util.Duration;

/**
 * Visor del holograma (etapa 3). Es el equivalente a SkeletonVisualizer +
 * SkeletonPanel del lab2, pero con JavaFX en vez de Swing, porque Swing solo
 * dibuja en 2D.
 *
 * Dos capas que se pueden combinar:
 * <ul>
 * <li>Holograma: nube de puntos sobre el cuerpo de MakeHuman (modelo/).</li>
 * <li>Esqueleto: los segmentos y nodos, como el dibujo del lab2.</li>
 * </ul>
 * Si no encuentra el modelo de MakeHuman, arranca solo con un esqueleto
 * genérico.
 *
 * <h2>Conceptos básicos de JavaFX</h2>
 * <ul>
 * <li>Application: clase base de toda app JavaFX. launch() crea la ventana y
 * llama a start(Stage).</li>
 * <li>Stage: la ventana. Scene: su contenido.</li>
 * <li>Grafo de escena: todo lo que se dibuja es un árbol de nodos (Group,
 * botones, esferas...). Otro árbol, como el del holograma. Cada nodo puede
 * tener transformaciones (mover, girar) que también afectan a sus hijos: es
 * cinemática directa otra vez, hecha por JavaFX.</li>
 * <li>SubScene: una "ventana 3D" dentro de la interfaz 2D, con su propia
 * cámara y luces.</li>
 * <li>Hilo de JavaFX: toda la interfaz se toca desde un único hilo. Por eso el
 * bucle de animación (AnimationTimer) se ejecuta en él y no hay problemas de
 * concurrencia.</li>
 * </ul>
 *
 * Controles: arrastrar con el ratón para girar la cámara, rueda para el zoom.
 */
public class HologramApp extends Application {

	private static final String[] AXES = { "X", "Y", "Z" };
	// Rutas relativas a la carpeta del proyecto (desde donde lo lanzan Eclipse y run.ps1)
	private static final Path BODY_FILE = Path.of("modelo", "cuerpo.obj");
	private static final Path WEIGHTS_FILE = Path.of("modelo", "default_weights.mhw");
	private static final Color HOLO = Color.web("#38d6ff"); // cian del holograma

	// ---- Modelo ----
	private HumanSkeleton skeleton;
	private Segment root; // árbol de segmentos
	private List<Segment> segments; // los mismos, en una lista
	private final Map<String, Segment> byName = new HashMap<>(); // búsqueda por nombre
	private Map<Segment, Matrix4> restInverse; // Frame_reposo^-1 de cada segmento (skinning)
	private MakeHumanRig rig; // cuerpo de MakeHuman (null si no se ha encontrado)

	// ---- Vista 3D ----
	private final Group cloudGroup = new Group(); // holograma (nube de puntos)
	private final Group skeletonGroup = new Group(); // esferas (nodos) y cilindros (segmentos)
	private final Group decorGroup = new Group(); // partículas y números del fondo
	private final List<PointCloud> clouds = new ArrayList<>();
	// Affine = transformación general de JavaFX (una matriz 3x4 como Matrix4).
	// Se guarda una por esfera y cilindro, y cada fotograma se le copia el frame
	// calculado por la cinemática.
	private final Map<Segment, Affine> jointTransforms = new IdentityHashMap<>();
	private final Map<Segment, Affine> boneTransforms = new IdentityHashMap<>();
	private SubScene sub;
	private Pane viewport; // contenedor 2D de la SubScene (su fondo es el fondo de la escena)
	private Box floor;

	// ---- Cámara orbital ----
	// La cámara cuelga de un "brazo" que gira alrededor del cuerpo:
	// yaw = giro horizontal, pitch = inclinación arriba/abajo, zoom = distancia.
	private final Rotate yaw = new Rotate(180, Rotate.Y_AXIS);
	private final Rotate pitch = new Rotate(-6, Rotate.X_AXIS);
	private final Translate zoom = new Translate(0, 0, -520);
	private double dragX, dragY; // última posición del ratón al arrastrar

	// ---- Panel de controles ----
	private ComboBox<Segment> selector;
	private final Slider[] axisSliders = new Slider[3];
	private final Label[] axisLabels = new Label[3];
	private final Label cloudLabel = new Label();
	private final Label statsLabel = new Label();
	private final CheckBox showCloud = new CheckBox("Holograma (nube de puntos)");
	private final CheckBox showSkeleton = new CheckBox("Esqueleto (segmentos y nodos)");
	private ToggleButton walkButton;

	// ---- Estado del bucle ----
	// dirty = "la postura ha cambiado y hay que recalcular". Así no se repite
	// la cinemática y el skinning en cada fotograma si nada se ha movido.
	private boolean dirty = true;
	// Evita un bucle de eventos: al mover los sliders desde el código (por
	// ejemplo al cambiar de articulación) no se debe interpretar como si el
	// usuario hubiera movido la articulación.
	private boolean updatingSliders;
	private double walkTime; // segundos de animación de marcha acumulados
	private long lastFrame, fpsWindowStart; // marcas de tiempo en nanosegundos
	private int fpsFrames;
	private double fps;
	private long totalFrames; // fotogramas desde el arranque (para --fpstest)

	/** Punto de entrada de JavaFX: monta toda la escena y arranca el bucle. */
	@Override
	public void start(Stage stage) {
		loadModel();
		Map<String, String> args = getParameters().getNamed(); // parámetros tipo --points=80000
		int pointCount = Integer.parseInt(args.getOrDefault("points", "60000"));

		// Grupo "mundo": aquí van las coordenadas de la cinemática.
		Group world = new Group(cloudGroup, skeletonGroup);
		// La cinemática usa Z hacia arriba (convenio de robótica), pero JavaFX usa
		// Y hacia ABAJO (convenio de pantallas: el píxel (0,0) está arriba a la
		// izquierda). Un giro de 90º sobre X convierte uno en otro:
		// (x, y, z) -> (x, -z, y). Como va en el grupo padre, afecta a todo lo de
		// dentro sin tocar el resto del código.
		world.getTransforms().add(new Rotate(90, Rotate.X_AXIS));
		buildSkeletonView();
		buildCloud(pointCount);
		buildDecor();

		// Suelo: una caja muy fina en y = 0 (solo cuando no se ve el holograma)
		floor = new Box(260, 1, 260);
		floor.setTranslateY(0.5);
		floor.setMaterial(new PhongMaterial(Color.web("#2b2f36")));

		// Iluminación "de tres puntos" simplificada (solo afecta al esqueleto; el
		// holograma brilla por sí mismo): una luz principal
		// cálida, una de relleno fría desde el lado contrario y una ambiental
		// para que las sombras no sean negras del todo.
		PointLight key = new PointLight(Color.web("#fff6ea"));
		key.getTransforms().add(new Translate(-250, -350, -300));
		PointLight fill = new PointLight(Color.web("#6f86a8"));
		fill.getTransforms().add(new Translate(300, -150, 350));
		AmbientLight ambient = new AmbientLight(Color.web("#3a3f48"));

		// Cámara con perspectiva: lo lejano se ve más pequeño.
		// nearClip y farClip: solo se dibuja lo que está entre 1 y 5000 unidades
		// de la cámara. fieldOfView: ángulo de visión (35º, como un teleobjetivo
		// suave, deforma menos que uno ancho).
		PerspectiveCamera camera = new PerspectiveCamera(true);
		camera.setNearClip(1);
		camera.setFarClip(5000);
		camera.setFieldOfView(35);
		// Cadena de transformaciones de la cámara (cinemática otra vez): subir al
		// centro del cuerpo (y = -92, que es "arriba" en JavaFX), girar (yaw),
		// inclinar (pitch) y alejarse hacia atrás (zoom).
		Group cameraRig = new Group(camera);
		cameraRig.getTransforms().addAll(new Translate(0, -92, 0), yaw, pitch);
		camera.getTransforms().add(zoom);

		Group scene3d = new Group(world, decorGroup, floor, key, fill, ambient, cameraRig);
		// true = usar depth buffer (lo de delante tapa a lo de detrás).
		// BALANCED = antialiasing, para que los bordes no salgan en escalera.
		sub = new SubScene(scene3d, 900, 700, true, SceneAntialiasing.BALANCED);
		sub.setCamera(camera);
		// La SubScene no se redimensiona sola: se "enlaza" (bind) su tamaño al de
		// su contenedor. Con bind, cuando cambia uno, el otro se actualiza solo.
		viewport = new Pane(sub);
		sub.widthProperty().bind(viewport.widthProperty());
		sub.heightProperty().bind(viewport.heightProperty());
		installCameraControls(sub);

		// BorderPane: la vista 3D en el centro y el panel a la izquierda
		BorderPane layout = new BorderPane(viewport);
		layout.setLeft(buildControls());
		Scene scene = new Scene(layout, 1200, 800);
		stage.setTitle("Holograma");
		stage.setScene(scene);
		stage.show();

		// AnimationTimer: JavaFX llama a handle() una vez por fotograma (unas 60
		// veces por segundo), con la hora actual en nanosegundos. Es una clase
		// anónima: se define y se instancia a la vez.
		new AnimationTimer() {
			@Override
			public void handle(long now) {
				frame(now);
			}
		}.start();

		String snapshot = args.get("snapshot");
		if (snapshot != null)
			takeSnapshotAndExit(scene, snapshot);

		// Prueba de rendimiento (--fpstest=segundos [--nowalk=1]): camina (o se
		// queda quieto) ese tiempo, imprime los FPS medios y cierra
		if (args.containsKey("fpstest")) {
			walkButton.setSelected(!args.containsKey("nowalk"));
			double seconds = Double.parseDouble(args.get("fpstest"));
			long startFrames = totalFrames;
			PauseTransition test = new PauseTransition(Duration.seconds(seconds));
			test.setOnFinished(e -> {
				System.out.printf("FPS medio: %.1f | %s | %s%n", (totalFrames - startFrames) / seconds,
						statsLabel.getText().replace('\n', ' '), cloudLabel.getText());
				Platform.exit();
			});
			test.play();
		}
	}

	/**
	 * Intenta cargar el cuerpo de MakeHuman y construir el esqueleto con sus
	 * articulaciones. Si falta algún fichero, usa el esqueleto genérico.
	 */
	private void loadModel() {
		Map<String, double[]> joints;
		try {
			rig = MakeHumanRig.load(BODY_FILE, WEIGHTS_FILE);
			joints = rig.getJoints();
		} catch (Exception e) {
			System.out.println("Sin modelo de MakeHuman (" + e.getMessage() + "): esqueleto genérico");
			rig = null;
			joints = HumanSkeleton.defaultJoints();
		}
		skeleton = HumanSkeleton.fromJoints(joints);
		root = skeleton.getRoot();
		segments = root.flatten();
		for (Segment s : segments)
			byName.put(s.getName(), s);
		restInverse = computeRestInverse();
	}

	// ================================================================ postura

	/**
	 * Bucle principal, una vez por fotograma:
	 * 1. Si está caminando, avanza la animación.
	 * 2. Si la postura ha cambiado: cinemática directa -> matrices de skinning
	 *    -> holograma -> esqueleto.
	 * Mide el tiempo de cada fase con System.nanoTime(), como en el lab2.
	 */
	private void frame(long now) {
		if (lastFrame != 0 && walkButton.isSelected()) {
			// Tiempo real transcurrido desde el fotograma anterior. Así la marcha
			// va a la misma velocidad aunque el ordenador vaya a 30 o a 144 FPS.
			walkTime += (now - lastFrame) / 1e9;
			applyWalk(walkTime);
			refreshSliders();
			dirty = true;
		}
		lastFrame = now;

		// Fotogramas por segundo, medidos en ventanas de medio segundo
		totalFrames++;
		fpsFrames++;
		if (now - fpsWindowStart > 500_000_000L) {
			fps = fpsFrames * 1e9 / (now - fpsWindowStart);
			fpsFrames = 0;
			fpsWindowStart = now;
		}
		if (!dirty)
			return;
		dirty = false;

		// 1) Cinemática directa (el algoritmo del lab2, en 3D)
		long t0 = System.nanoTime();
		double[] o = skeleton.getOrigin();
		Node3D tree = ForwardKinematics3D.computePositions(root, o[0], o[1], o[2]);
		Map<Segment, Matrix4> frames = ForwardKinematics3D.frames(tree);
		long t1 = System.nanoTime();

		// 2) Matriz de skinning de cada hueso: S = Frame_actual * Frame_reposo^-1
		Map<Segment, double[]> skin = new IdentityHashMap<>();
		for (Segment s : segments)
			skin.put(s, frames.get(s).multiply(restInverse.get(s)).toArray());
		// 3) Mover los puntos del holograma (si se está viendo). La nube usa un array en el orden de
		//    MakeHumanRig.SEGMENTS, más rápido de consultar que un mapa.
		if (cloudGroup.isVisible() && rig != null) {
			double[][] skinArray = new double[MakeHumanRig.SEGMENTS.length][];
			for (int i = 0; i < skinArray.length; i++)
				skinArray[i] = skin.get(byName.get(MakeHumanRig.SEGMENTS[i]));
			for (PointCloud c : clouds)
				c.update(skinArray);
		}
		long t2 = System.nanoTime();

		// 4) Colocar las esferas (en cada articulación) y los cilindros
		for (Segment s : segments) {
			Matrix4 f = frames.get(s);
			setAffine(jointTransforms.get(s), f);
			Affine bone = boneTransforms.get(s);
			if (bone != null)
				// Un Cylinder de JavaFX está centrado en su origen y orientado
				// según el eje Y. Para que cubra el segmento: avanzar media
				// longitud por Z y girar 90º sobre X (lleva el eje Y al Z).
				setAffine(bone, f.multiply(Matrix4.translation(0, 0, s.getLength() / 2))
						.multiply(Matrix4.rotX(Math.PI / 2)));
		}
		statsLabel.setText(String.format("Cinemática directa: %.1f µs%nSkinning: %.2f ms%nFPS: %.0f",
				(t1 - t0) / 1e3, (t2 - t1) / 1e6, fps));
	}

	/**
	 * Ciclo de marcha muy simplificado: cada articulación oscila con un seno.
	 * No es biomecánica real (eso necesitaría datos de captura de movimiento o
	 * dinámica), pero basta para ver la cinemática en acción.
	 *
	 * p es la fase del ciclo (0,9 pasos por segundo). Las dos piernas van
	 * desfasadas medio ciclo (signos opuestos), y cada brazo va en oposición a
	 * la pierna de su lado, como al caminar de verdad. Los brazos además se
	 * bajan desde la pose en "A" hasta casi pegarlos al cuerpo.
	 */
	private void applyWalk(double t) {
		double p = 2 * Math.PI * 0.9 * t;
		double sin = Math.sin(p), cos = Math.cos(p);
		double armDown = skeleton.getArmRestAngle() - 8; // grados para bajar el brazo
		set("Muslo D", 0, 25 * sin); // cadera: +-25º adelante/atrás
		set("Muslo I", 0, -25 * sin);
		// Rodilla: se dobla sobre todo cuando la pierna pasa por delante (fase de
		// balanceo). Math.max(0, cos) recorta la mitad negativa del coseno.
		set("Tibia D", 0, -(5 + 55 * Math.max(0, cos)));
		set("Tibia I", 0, -(5 + 55 * Math.max(0, -cos)));
		set("Pie D", 0, 10 * sin);
		set("Pie I", 0, -10 * sin);
		set("Brazo D", 0, -22 * sin); // brazos en oposición a las piernas
		set("Brazo I", 0, 22 * sin);
		set("Brazo D", 1, -armDown); // abducción: el derecho baja con ángulo negativo
		set("Brazo I", 1, armDown); // y el izquierdo con positivo (espejo)
		set("Antebrazo D", 0, 15 + 15 * Math.max(0, -sin)); // codos algo doblados
		set("Antebrazo I", 0, 15 + 15 * Math.max(0, sin));
		set("Tórax", 2, -6 * sin); // los hombros giran al contrario que la pelvis
		set("Pelvis", 2, 4 * sin);
	}

	/** Atajo: pone un ángulo en grados a un segmento buscado por nombre. */
	private void set(String name, int axis, double degrees) {
		byName.get(name).setAngle(axis, Math.toRadians(degrees));
	}

	/**
	 * Calcula Frame_reposo^-1 de cada segmento: la cinemática directa con
	 * todos los ángulos a 0, invertida. Se guardan los ángulos actuales, se
	 * ponen a 0, se calcula y se restauran.
	 */
	private Map<Segment, Matrix4> computeRestInverse() {
		Map<Segment, double[]> saved = new IdentityHashMap<>();
		for (Segment s : segments) {
			saved.put(s, new double[] { s.getAngle(0), s.getAngle(1), s.getAngle(2) });
			s.resetAngles();
		}
		double[] o = skeleton.getOrigin();
		Map<Segment, Matrix4> rest = ForwardKinematics3D
				.frames(ForwardKinematics3D.computePositions(root, o[0], o[1], o[2]));
		Map<Segment, Matrix4> inverse = new IdentityHashMap<>();
		for (Segment s : segments) {
			inverse.put(s, rest.get(s).rigidInverse());
			for (int i = 0; i < 3; i++)
				s.setAngle(i, saved.get(s)[i]);
		}
		return inverse;
	}

	// ================================================================ vista 3D

	/**
	 * (Re)genera el holograma con el número de puntos dado. Son dos nubes: una
	 * grande de puntos pequeños y otra con un 8% de puntos más gruesos y
	 * blancos, que dan el efecto de "chispas" de la imagen de referencia.
	 */
	private void buildCloud(int count) {
		clouds.clear();
		cloudGroup.getChildren().clear();
		if (rig == null) {
			cloudLabel.setText("Sin modelo de MakeHuman en modelo/");
			return;
		}
		PointCloud base = new PointCloud(rig, count, 0.22, 1, glowing(HOLO));
		PointCloud sparks = new PointCloud(rig, count / 12, 0.42, 2, glowing(Color.web("#d8f6ff")));
		clouds.add(base);
		clouds.add(sparks);
		cloudGroup.getChildren().addAll(base.getNode(), sparks.getNode());
		cloudLabel.setText(String.format("%,d puntos en %d mallas", count + count / 12,
				base.getBucketCount() + sparks.getBucketCount()));
		dirty = true;
	}

	/**
	 * Material que BRILLA POR SÍ MISMO: color difuso negro (no le afecta la
	 * iluminación) más un "mapa de autoiluminación" de un solo píxel del color
	 * deseado. Así el punto se ve siempre de ese color, como una luz.
	 */
	private static PhongMaterial glowing(Color color) {
		WritableImage pixel = new WritableImage(1, 1);
		pixel.getPixelWriter().setColor(0, 0, color);
		PhongMaterial m = new PhongMaterial(Color.BLACK);
		m.setSpecularColor(Color.BLACK);
		m.setSelfIlluminationMap(pixel);
		return m;
	}

	/**
	 * Decorado del fondo: partículas flotando y "0" y "1" sueltos, como en la
	 * imagen de referencia. Con semilla fija para que siempre salga igual.
	 */
	private void buildDecor() {
		Random rnd = new Random(3);
		PhongMaterial dot = new PhongMaterial(Color.BLACK);
		WritableImage px = new WritableImage(1, 1);
		px.getPixelWriter().setColor(0, 0, Color.web("#2a8cff"));
		dot.setSelfIlluminationMap(px);
		for (int i = 0; i < 70; i++) {
			Sphere s = new Sphere(0.6 + rnd.nextDouble() * 1.4);
			s.setMaterial(dot);
			// Coordenadas de JavaFX (y hacia abajo): repartidas por detrás y a los
			// lados del cuerpo
			s.getTransforms().add(new Translate(rnd.nextGaussian() * 160, -rnd.nextDouble() * 230,
					60 + rnd.nextDouble() * 300));
			decorGroup.getChildren().add(s);
		}
		String[] bits = { "0", "1", "01", "10", "11", "00" };
		for (int i = 0; i < 45; i++) {
			Text t = new Text(bits[rnd.nextInt(bits.length)]);
			t.setFont(Font.font("Consolas", 7 + rnd.nextDouble() * 7));
			t.setFill(Color.web("#3aa8ff", 0.25 + rnd.nextDouble() * 0.35));
			// El texto es un nodo 2D, pero dentro de una SubScene 3D se puede
			// colocar a cualquier profundidad. Se gira 180º para que se lea bien
			// desde la cámara, que mira hacia el cuerpo desde delante.
			t.getTransforms().addAll(new Translate(rnd.nextGaussian() * 170, -rnd.nextDouble() * 220,
					80 + rnd.nextDouble() * 260), new Rotate(180, Rotate.Y_AXIS));
			decorGroup.getChildren().add(t);
		}
	}

	/**
	 * Crea la vista del esqueleto "desnudo", como el dibujo del lab2 pero en 3D:
	 * una esfera en cada articulación (los nodos) y un cilindro por segmento.
	 */
	private void buildSkeletonView() {
		PhongMaterial jointMat = new PhongMaterial(Color.web("#e8553f"));
		PhongMaterial boneMat = new PhongMaterial(Color.web("#f2c14e"));
		for (Segment s : segments) {
			Sphere joint = new Sphere(2.4);
			joint.setMaterial(jointMat);
			Affine ja = new Affine();
			joint.getTransforms().add(ja);
			jointTransforms.put(s, ja);
			skeletonGroup.getChildren().add(joint);
			if (s.getLength() > 0) { // la pelvis mide 0: no lleva cilindro
				Cylinder bone = new Cylinder(1.1, s.getLength());
				bone.setMaterial(boneMat);
				Affine ba = new Affine();
				bone.getTransforms().add(ba);
				boneTransforms.put(s, ba);
				skeletonGroup.getChildren().add(bone);
			}
		}
	}

	/** Copia una Matrix4 en un Affine de JavaFX (los dos son matrices 3x4 por filas). */
	private static void setAffine(Affine a, Matrix4 f) {
		double[] m = f.toArray();
		a.setToTransform(m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8], m[9], m[10], m[11]);
	}

	/**
	 * Aplica el aspecto según lo que esté activado. Con el holograma: fondo
	 * azul con degradado radial, decorado y efecto Bloom (resplandor). Sin él:
	 * fondo gris oscuro y suelo.
	 *
	 * Bloom es un efecto 2D que JavaFX aplica a la imagen ya dibujada de la
	 * SubScene: busca los píxeles más brillantes que un umbral y los difumina
	 * alrededor, como la luz de un neón. Es lo que hace "brillar" los puntos.
	 */
	private void updateLook() {
		boolean holo = showCloud.isSelected() && rig != null;
		cloudGroup.setVisible(showCloud.isSelected());
		skeletonGroup.setVisible(showSkeleton.isSelected());
		decorGroup.setVisible(holo);
		floor.setVisible(!holo);
		// El fondo se pinta en el Pane que hay DETRÁS de la SubScene (que se deja
		// transparente) y no en la propia SubScene: si no, el efecto Bloom, que
		// trabaja sobre la imagen de la SubScene, también lo procesaría y lo
		// pintaría mal.
		sub.setFill(Color.TRANSPARENT);
		Paint background = holo
				// Degradado radial: azul en el centro que se oscurece hacia los bordes
				? new RadialGradient(0, 0, 0.5, 0.45, 0.75, true, CycleMethod.NO_CYCLE,
						new Stop(0, Color.web("#0b3f8a")), new Stop(1, Color.web("#020b1f")))
				: Color.web("#14171c");
		viewport.setBackground(new Background(new BackgroundFill(background, null, null)));
		sub.setEffect(holo ? new Bloom(0.35) : null);
		dirty = true; // lo que se vuelve a mostrar tiene que ponerse en la postura actual
	}

	/**
	 * Cámara orbital con el ratón. Se usan lambdas (e -> {...}): funciones
	 * anónimas que JavaFX ejecuta cuando ocurre el evento.
	 */
	private void installCameraControls(SubScene sub) {
		sub.setOnMousePressed(e -> {
			dragX = e.getSceneX();
			dragY = e.getSceneY();
		});
		sub.setOnMouseDragged(e -> {
			// El desplazamiento del ratón (en píxeles) se convierte en grados
			yaw.setAngle(yaw.getAngle() + (e.getSceneX() - dragX) * 0.4);
			// La inclinación se limita a +-80º para no dar la vuelta por encima
			pitch.setAngle(Math.max(-80, Math.min(80, pitch.getAngle() - (e.getSceneY() - dragY) * 0.4)));
			dragX = e.getSceneX();
			dragY = e.getSceneY();
		});
		// Rueda: acercar o alejar, entre 120 y 1500 unidades del centro
		sub.setOnScroll(e -> zoom.setZ(Math.max(-1500, Math.min(-120, zoom.getZ() + e.getDeltaY()))));
	}

	// ================================================================ panel

	/** Construye el panel de la izquierda con todos los controles. */
	private VBox buildControls() {
		Label title = new Label("Holograma");
		title.setStyle("-fx-font-size: 18; -fx-font-weight: bold;"); // CSS de JavaFX

		// Desplegable con los segmentos que tienen al menos un eje libre
		selector = new ComboBox<>();
		for (Segment s : segments)
			if (s.isAxisFree(0) || s.isAxisFree(1) || s.isAxisFree(2))
				selector.getItems().add(s);
		selector.setMaxWidth(Double.MAX_VALUE);
		selector.setOnAction(e -> refreshSliders());

		// VBox: coloca sus hijos en columna, con 7 px de separación
		VBox box = new VBox(7, title, new Label("Articulación"), selector);

		// Un slider por eje (X, Y, Z)
		for (int i = 0; i < 3; i++) {
			// Copia "final" de i: una lambda solo puede usar variables locales que
			// no cambian, y la i del for cambia en cada vuelta.
			int axis = i;
			axisLabels[i] = new Label();
			axisSliders[i] = new Slider();
			// Listener: se ejecuta cada vez que cambia el valor del slider
			axisSliders[i].valueProperty().addListener((obs, old, val) -> {
				Segment s = selector.getValue();
				if (updatingSliders || s == null)
					return; // el cambio viene del propio código, no del usuario
				s.setAngle(axis, Math.toRadians(val.doubleValue()));
				updateAxisLabel(axis, s);
				dirty = true; // la postura ha cambiado: recalcular en el próximo fotograma
			});
			box.getChildren().addAll(axisLabels[i], axisSliders[i]);
		}

		Button reset = new Button("Postura de reposo");
		reset.setOnAction(e -> {
			walkButton.setSelected(false);
			// Referencia a método: equivale a s -> s.resetAngles()
			segments.forEach(Segment::resetAngles);
			refreshSliders();
			dirty = true;
		});
		// ToggleButton: botón que se queda pulsado o no (on/off)
		walkButton = new ToggleButton("Caminar");
		walkButton.selectedProperty().addListener((obs, old, on) -> {
			// Mientras camina, los sliders solo muestran (no se pueden mover)
			for (Slider s : axisSliders)
				s.setDisable(on || s.isDisable());
			if (!on)
				refreshSliders();
		});

		// Qué se ve. Por defecto el holograma si hay modelo, y si no, el esqueleto.
		showCloud.setSelected(rig != null);
		showSkeleton.setSelected(rig == null);
		showCloud.setDisable(rig == null);
		for (CheckBox c : new CheckBox[] { showCloud, showSkeleton })
			c.selectedProperty().addListener((obs, old, on) -> updateLook());

		// Slider de puntos del holograma: de 10.000 a 300.000
		Slider pointSlider = new Slider(10_000, 300_000, clouds.isEmpty() ? 60_000 : clouds.get(0).getCount());
		pointSlider.setDisable(rig == null);
		// Solo se regenera al SOLTAR el slider (valueChanging pasa a false): si se
		// regenerase mientras se arrastra, se crearían cientos de nubes para nada
		pointSlider.valueChangingProperty().addListener((obs, was, changing) -> {
			if (!changing)
				buildCloud((int) pointSlider.getValue());
		});

		box.getChildren().addAll(new Separator(), reset, walkButton, new Separator(),
				showCloud, showSkeleton, new Separator(),
				new Label("Puntos del holograma"), pointSlider, cloudLabel, new Separator(), statsLabel);
		box.setPadding(new Insets(14));
		box.setPrefWidth(270);

		selector.getSelectionModel().select(byName.get("Brazo D"));
		refreshSliders();
		updateLook();
		return box;
	}

	/**
	 * Ajusta los 3 sliders a la articulación seleccionada: rango = límites del
	 * eje, valor = ángulo actual, deshabilitado si el eje está bloqueado.
	 */
	private void refreshSliders() {
		Segment s = selector.getValue();
		if (s == null)
			return;
		updatingSliders = true; // que el listener ignore estos cambios
		for (int i = 0; i < 3; i++) {
			Slider sl = axisSliders[i];
			boolean free = s.isAxisFree(i);
			sl.setMin(Math.toDegrees(s.getMin(i)));
			sl.setMax(Math.toDegrees(s.getMax(i)));
			sl.setValue(Math.toDegrees(s.getAngle(i)));
			sl.setDisable(!free || walkButton != null && walkButton.isSelected());
			updateAxisLabel(i, s);
		}
		updatingSliders = false;
	}

	/** Texto encima de cada slider, por ejemplo "Flexión (eje X): 35°". */
	private void updateAxisLabel(int axis, Segment s) {
		axisLabels[axis].setText(s.isAxisFree(axis)
				? String.format("%s (eje %s): %.0f°", s.getAxisName(axis), AXES[axis], Math.toDegrees(s.getAngle(axis)))
				: "Eje " + AXES[axis] + ": bloqueado");
	}

	// ================================================================ captura

	/**
	 * Modo captura, para generar imágenes sin tocar el ratón:
	 * --snapshot=fichero.png [--walk=segundos] [--yaw=grados]
	 * [--show=cloud,skeleton] [--points=n]. Espera 2 s a que se dibuje la
	 * escena, la guarda en PNG y cierra.
	 */
	private void takeSnapshotAndExit(Scene scene, String file) {
		Map<String, String> args = getParameters().getNamed();
		if (args.containsKey("walk")) {
			applyWalk(Double.parseDouble(args.get("walk")));
			refreshSliders();
			dirty = true;
		}
		if (args.containsKey("yaw"))
			yaw.setAngle(Double.parseDouble(args.get("yaw")));
		if (args.containsKey("show")) {
			String show = args.get("show");
			showCloud.setSelected(show.contains("cloud"));
			showSkeleton.setSelected(show.contains("skeleton"));
		}
		// PauseTransition: ejecuta algo pasado un tiempo, sin bloquear el hilo de
		// JavaFX (un Thread.sleep congelaría la ventana y no se dibujaría nada)
		PauseTransition wait = new PauseTransition(Duration.seconds(2));
		wait.setOnFinished(e -> {
			WritableImage img = scene.snapshot(null);
			// Se pasa la imagen de JavaFX a una de AWT (BufferedImage) píxel a
			// píxel, porque ImageIO, que es quien sabe escribir PNG, es de AWT
			int w = (int) img.getWidth(), h = (int) img.getHeight();
			BufferedImage out = new BufferedImage(w, h, BufferedImage.TYPE_INT_ARGB);
			PixelReader pr = img.getPixelReader();
			for (int y = 0; y < h; y++)
				for (int x = 0; x < w; x++)
					out.setRGB(x, y, pr.getArgb(x, y));
			try {
				ImageIO.write(out, "png", new File(file));
				System.out.println(statsLabel.getText().replace('\n', ' ') + " | " + cloudLabel.getText());
			} catch (Exception ex) {
				ex.printStackTrace();
			}
			Platform.exit();
		});
		wait.play();
	}

	/** main: delega en launch(), que inicia JavaFX y acaba llamando a start(). */
	public static void main(String[] args) {
		launch(args);
	}
}

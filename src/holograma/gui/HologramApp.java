package holograma.gui;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;

import javax.imageio.ImageIO;

import holograma.body.MakeHumanRig;
import holograma.body.PointCloud;
import holograma.dynamics.Exoskeleton;
import holograma.dynamics.InverseDynamics;
import holograma.kinematics.ForwardKinematics3D;
import holograma.kinematics.HumanSkeleton;
import holograma.kinematics.Matrix4;
import holograma.kinematics.Node3D;
import holograma.kinematics.Segment;
import holograma.mocap.BvhMotion;
import holograma.mocap.LiveReceiver;
import holograma.mocap.LiveRetargeter;
import holograma.mocap.Retargeter;
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
import javafx.scene.control.ScrollPane;
import javafx.scene.control.Separator;
import javafx.scene.control.Slider;
import javafx.scene.control.ToggleButton;
import javafx.scene.control.ToggleGroup;
import javafx.scene.effect.Bloom;
import javafx.scene.image.PixelReader;
import javafx.scene.image.WritableImage;
import javafx.scene.layout.Background;
import javafx.scene.layout.BackgroundFill;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.HBox;
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
import javafx.stage.FileChooser;
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
 *
 * Hay dos formas de animarlo: "Caminar" (senos, ver applyWalk) y
 * "Reproducir" una captura de movimiento real de un fichero .bvh (paquete
 * mocap). Al arrancar se carga animaciones/caminar.bvh si existe.
 *
 * Con "Mapa de esfuerzo" se calculan los pares de las articulaciones
 * (dinámica inversa, paquete dynamics) y el holograma se colorea según lo
 * cerca que está cada articulación de su par máximo.
 *
 * Con "Exoesqueleto virtual" se le ponen motores en las piernas
 * (Exoskeleton, dibujados por ExoView) que ayudan con parte de ese par, y se
 * miden par, velocidad, potencia y batería de cada motor.
 *
 * Con "En directo (webcam)" el holograma copia en tiempo real la postura que
 * manda webcam/pose_sender.py (MediaPipe) por UDP (LiveReceiver y
 * LiveRetargeter).
 */
public class HologramApp extends Application {

	private static final String[] AXES = { "X", "Y", "Z" };
	// Rutas relativas a la carpeta del proyecto (desde donde lo lanzan Eclipse y run.ps1)
	private static final Path BODY_FILE = Path.of("modelo", "cuerpo.obj");
	private static final Path WEIGHTS_FILE = Path.of("modelo", "default_weights.mhw");
	private static final Path ANIMATIONS_DIR = Path.of("animaciones");
	private static final Path DEFAULT_ANIMATION = ANIMATIONS_DIR.resolve("caminar.bvh");
	private static final Color HOLO = Color.web("#38d6ff"); // cian del holograma

	// ---- Modelo ----
	private HumanSkeleton skeleton;
	private Segment root; // árbol de segmentos
	private List<Segment> segments; // los mismos, en una lista
	private final Map<String, Segment> byName = new HashMap<>(); // búsqueda por nombre
	private Map<Segment, Matrix4> restInverse; // Frame_reposo^-1 de cada segmento (skinning)
	private MakeHumanRig rig; // cuerpo de MakeHuman (null si no se ha encontrado)
	private Retargeter mocap; // animación .bvh cargada (null si no hay ninguna)
	private InverseDynamics dynamics; // pares articulares
	// Dónde va la pelvis: el origen en reposo, o el que diga la animación .bvh
	// (que la sube y la baja)
	private double[] origin;

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
	private final CheckBox effortBox = new CheckBox("Mapa de esfuerzo (pares)");
	private final Slider massSlider = new Slider(40, 120, 70);
	private final Label effortLabel = new Label();
	// Pico de par de cada articulación desde la última vez que se reinició
	// (al cargar o reproducir una animación, cambiar la masa o volver al reposo)
	private final Map<String, Double> peaks = new HashMap<>();
	// Paletas de colores del mapa de esfuerzo (ver buildPalettes)
	private static final int PALETTE_SIZE = 32;
	private static final double DYNAMICS_STEP = 0.08; // paso de las diferencias finitas (s), ver showEffort
	private PhongMaterial[] cloudPalette, sparkPalette, jointPalette;
	private PhongMaterial jointMat; // color normal de las esferas
	private static final double JOINT_RADIUS = 2.4; // radio de las esferas de las articulaciones (cm)
	private PhongMaterial limitMat; // esferas de las articulaciones que han llegado a su límite
	private final Map<Segment, Sphere> jointSpheres = new IdentityHashMap<>();
	private boolean colored; // si ahora mismo se ve el mapa de esfuerzo

	// ---- Exoesqueleto virtual ----
	private final Exoskeleton exo = new Exoskeleton();
	private final ExoView exoView = new ExoView();
	private final CheckBox exoBox = new CheckBox("Exoesqueleto virtual");
	private final CheckBox hipBox = new CheckBox("Cadera"), kneeBox = new CheckBox("Rodilla"),
			ankleBox = new CheckBox("Tobillo");
	private final Slider assistSlider = new Slider(0, 100, 50);
	private final Slider motorTorqueSlider = new Slider(10, 150, 60);
	private final Slider motorMassSlider = new Slider(0, 4, 1.5);
	private final Label exoLabel = new Label();
	private double exoMass; // kg de todo el exo con la configuración actual
	// Segundos de animación que han pasado en este fotograma (para la energía
	// de los motores; 0 si está parado)
	private double frameDt;

	// ---- En directo (webcam) ----
	private final LiveReceiver receiver = new LiveReceiver(LiveReceiver.DEFAULT_PORT);
	private LiveRetargeter live;
	private final ToggleButton liveButton = new ToggleButton("En directo (webcam)");
	private final CheckBox mirrorBox = new CheckBox("Modo espejo");
	private final Label liveLabel = new Label();
	private long lastLiveSeq, lastLiveNanos, liveWindowStart;
	private int liveCount; // posturas recibidas en la ventana de medio segundo actual
	private double liveRate;
	/**
	 * Historial de las últimas posturas en directo (cuándo llegó cada una y su
	 * cinemática), para la dinámica inversa: en directo no se conoce el futuro,
	 * así que se calcula un poco en el pasado (ver computeDynamics). Es una
	 * COLA: se añade por el final y se quita por el principio lo que tiene más
	 * de un segundo.
	 */
	private record LiveSample(long nanos, Map<Segment, Matrix4> frames) {
	}

	private final Deque<LiveSample> liveHistory = new ArrayDeque<>();
	private boolean liveNew; // ha llegado una postura nueva en este fotograma
	// Programa de Python que lee la webcam (lo arranca el botón "En directo")
	private static final Path SENDER_SCRIPT = Path.of("webcam", "run.ps1");
	private static final Path SENDER_LOG = Path.of("webcam", "sender.log");
	private Process sender;
	private boolean launchSender = true; // false con --live (pruebas: los datos los manda otro)
	private final CheckBox showCloud = new CheckBox("Holograma (nube de puntos)");
	private final CheckBox showSkeleton = new CheckBox("Esqueleto (segmentos y nodos)");
	private ToggleButton walkButton;
	private final ToggleButton playButton = new ToggleButton("Reproducir");
	private final Label animLabel = new Label();
	private final Slider speedSlider = new Slider(0, 2, 1); // 0 = en pausa (la dinámica sigue usando la animación)
	private Stage stage;

	// ---- Estado del bucle ----
	// dirty = "la postura ha cambiado y hay que recalcular". Así no se repite
	// la cinemática y el skinning en cada fotograma si nada se ha movido.
	private boolean dirty = true;
	// Evita un bucle de eventos: al mover los sliders desde el código (por
	// ejemplo al cambiar de articulación) no se debe interpretar como si el
	// usuario hubiera movido la articulación.
	private boolean updatingSliders;
	private double walkTime; // segundos de animación de marcha acumulados
	private double animTime; // segundos de la animación .bvh (a la velocidad elegida)
	private long retargetNanos; // lo que tardó el último retargeting
	private long lastFrame, fpsWindowStart; // marcas de tiempo en nanosegundos
	private int fpsFrames;
	private double fps;
	private long totalFrames; // fotogramas desde el arranque (para --fpstest)

	/** Punto de entrada de JavaFX: monta toda la escena y arranca el bucle. */
	@Override
	public void start(Stage stage) {
		this.stage = stage;
		loadModel();
		Map<String, String> args = getParameters().getNamed(); // parámetros tipo --points=80000
		int pointCount = Integer.parseInt(args.getOrDefault("points", "60000"));

		// Grupo "mundo": aquí van las coordenadas de la cinemática.
		Group world = new Group(cloudGroup, skeletonGroup, exoView.getNode());
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
		// ScrollPane: si el panel no cabe en la ventana, sale una barra para bajar
		ScrollPane panel = new ScrollPane(buildControls());
		panel.setHbarPolicy(ScrollPane.ScrollBarPolicy.NEVER);
		layout.setLeft(panel);
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

		// --effort=1: arranca con el mapa de esfuerzo activado
		if (args.containsKey("effort"))
			effortBox.setSelected(true);
		// --exo=1: arranca con el exoesqueleto puesto (con --ankle=1, también en el tobillo)
		if (args.containsKey("exo"))
			exoBox.setSelected(true);
		if (args.containsKey("ankle"))
			ankleBox.setSelected(true);
		// --live=1: arranca en directo SIN lanzar la webcam (para pruebas en las
		// que los datos los manda otro programa, por ejemplo pose_sender.py --image)
		if (args.containsKey("live")) {
			launchSender = false;
			liveButton.setSelected(true);
		}

		String snapshot = args.get("snapshot");
		if (snapshot != null)
			takeSnapshotAndExit(scene, snapshot);

		// Prueba de rendimiento (--fpstest=segundos [--nowalk=1]): camina (o se
		// queda quieto) ese tiempo, imprime los FPS medios y cierra. Con --bvh,
		// en vez de caminar reproduce la animación.
		if (args.containsKey("fpstest")) {
			if (!args.containsKey("nowalk"))
				(args.containsKey("bvh") ? playButton : walkButton).setSelected(true);
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
		origin = skeleton.getOrigin();
		root = skeleton.getRoot();
		segments = root.flatten();
		for (Segment s : segments)
			byName.put(s.getName(), s);
		restInverse = computeRestInverse();
		dynamics = new InverseDynamics(skeleton);
		live = new LiveRetargeter(skeleton);

		// Animación: la de --bvh=fichero o, si no, la de por defecto
		String bvh = getParameters().getNamed().get("bvh");
		Path file = bvh != null ? Path.of(bvh) : DEFAULT_ANIMATION;
		if (bvh != null || file.toFile().exists())
			loadAnimation(file);
	}

	/**
	 * Carga un .bvh y prepara el retargeting. Si falla (fichero roto, o un
	 * esqueleto con nombres que no se reconocen), lo dice en el panel y se
	 * queda con la animación que hubiera.
	 */
	private void loadAnimation(Path file) {
		try {
			BvhMotion motion = BvhMotion.load(file);
			mocap = new Retargeter(motion, skeleton);
			animTime = 0;
			resetPeaks();
			animLabel.setText(String.format("%s: %d fotogramas, %.1f s", motion.getName(),
					motion.getFrameCount() - mocap.getFirstFrame(), motion.getDuration()));
		} catch (Exception e) {
			animLabel.setText("No se pudo cargar " + file.getFileName() + ": " + e.getMessage());
			System.out.println(animLabel.getText());
		}
		animLabel.setWrapText(true);
		playButton.setDisable(mocap == null);
	}

	/** Pone el esqueleto en la postura de la animación .bvh en el instante animTime. */
	private void applyAnimation() {
		long t = System.nanoTime();
		origin = mocap.apply(mocap.frameAt(animTime));
		retargetNanos = System.nanoTime() - t;
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
		frameDt = 0;
		if (lastFrame != 0 && walkButton.isSelected()) {
			// Tiempo real transcurrido desde el fotograma anterior. Así la marcha
			// va a la misma velocidad aunque el ordenador vaya a 30 o a 144 FPS.
			frameDt = (now - lastFrame) / 1e9;
			walkTime += frameDt;
			applyWalk(walkTime);
			refreshSliders();
			dirty = true;
		}
		if (lastFrame != 0 && playButton.isSelected() && mocap != null) {
			frameDt = (now - lastFrame) / 1e9 * speedSlider.getValue();
			animTime += frameDt;
			applyAnimation();
			refreshSliders();
			dirty = true;
		}
		liveNew = false;
		if (liveButton.isSelected())
			updateLive(now);
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
		Node3D tree = ForwardKinematics3D.computePositions(root, origin[0], origin[1], origin[2]);
		Map<Segment, Matrix4> frames = ForwardKinematics3D.frames(tree);
		if (liveNew) {
			liveHistory.addLast(new LiveSample(now, frames));
			while (now - liveHistory.peekFirst().nanos() > 1_000_000_000L)
				liveHistory.removeFirst();
		}
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
		// 5) Pares articulares (dinámica inversa), mapa de esfuerzo y exoesqueleto
		long t3 = System.nanoTime();
		boolean needDynamics = effortBox.isSelected() || exoBox.isSelected();
		InverseDynamics.Result result = needDynamics ? computeDynamics(frames) : null;
		if (effortBox.isSelected())
			showEffort(result);
		else if (colored)
			clearEffort();
		exoView.setVisible(exoBox.isSelected());
		if (exoBox.isSelected())
			showExo(frames, result);
		long t4 = System.nanoTime();
		String limits = showLimits();

		String retarget = playButton.isSelected() ? String.format("Retargeting .bvh: %.1f µs%n", retargetNanos / 1e3) : "";
		String dyn = needDynamics ? String.format("Dinámica inversa: %.1f µs%n", (t4 - t3) / 1e3) : "";
		statsLabel.setText(String.format("%sCinemática directa: %.1f µs%nSkinning: %.2f ms%n%sFPS: %.0f%s", retarget,
				(t1 - t0) / 1e3, (t2 - t1) / 1e6, dyn, fps, limits));
	}

	/**
	 * Pinta en rojo (y más grandes) las esferas de las articulaciones que se
	 * han quedado en su límite anatómico: la animación o la webcam pedían más
	 * ángulo del que deja la articulación y Segment.setAngle lo ha recortado.
	 * Con los sliders no pasa nunca, porque su rango ya son los límites.
	 * Va después del mapa de esfuerzo para que el rojo se vea por encima.
	 *
	 * @return Texto para las estadísticas con las articulaciones en el
	 *         límite (vacío si no hay ninguna).
	 */
	private String showLimits() {
		StringBuilder sb = new StringBuilder();
		for (Segment s : segments) {
			Sphere sphere = jointSpheres.get(s);
			String axis = s.limitAxis();
			if (axis != null) {
				sphere.setMaterial(limitMat);
				// Más grande cambiando el radio: setScale no sirve aquí porque
				// JavaFX lo aplica después del Affine y también alejaría la
				// esfera de su sitio (escalaría su posición)
				sphere.setRadius(JOINT_RADIUS * 1.6);
				sb.append(String.format("%n  %s (%s)", s.getName(), axis.toLowerCase()));
			} else {
				// Si hay mapa de esfuerzo, showEffort ya le ha puesto su color
				if (!colored)
					sphere.setMaterial(jointMat);
				sphere.setRadius(JOINT_RADIUS);
			}
		}
		return sb.length() == 0 ? "" : "\nEn el límite:" + sb;
	}

	// ================================================================ esfuerzo

	/**
	 * Calcula los pares con la dinámica inversa (y, si está activado, con el
	 * exoesqueleto: sus masas y su ayuda).
	 *
	 * La dinámica necesita velocidades y aceleraciones, que salen de comparar
	 * la postura actual con la de un poco antes y un poco después (diferencias
	 * finitas). Según lo que se esté viendo:
	 * <ul>
	 * <li>Animación .bvh: los fotogramas de DYNAMICS_STEP (0,08 s) antes y
	 * después, con la pelvis en su trayectoria real (ver
	 * Retargeter.travelledOrigin). No se usa el fotograma de al lado (1/120 s
	 * en CMU) porque al derivar dos veces el ruido de la captura se multiplica
	 * por 1/h²: con 1/120 s la fuerza del suelo salía hasta 4 veces el peso al
	 * caminar. Un paso de 0,08 s hace de filtro paso bajo de unos 6 Hz, que es
	 * lo que se usa en los laboratorios de biomecánica antes de derivar.</li>
	 * <li>Marcha con senos: lo mismo, con applyWalk en t - h y t + h.</li>
	 * <li>Quieto (sliders): la misma postura tres veces, es decir, solo
	 * gravedad (caso estático).</li>
	 * </ul>
	 * Después de calcular las posturas vecinas hay que volver a poner la
	 * actual, porque applyWalk y Retargeter.apply cambian los ángulos.
	 */
	private InverseDynamics.Result computeDynamics(Map<Segment, Matrix4> frames) {
		Map<Segment, Matrix4> prev = frames, cur = frames, next = frames;
		double h = 1;
		boolean standing = true;
		if (playButton.isSelected() && mocap != null) {
			double dt = mocap.getMotion().getFrameTime();
			int k = Math.max(1, (int) Math.round(DYNAMICS_STEP / dt));
			int f = mocap.frameAt(animTime), n = mocap.getMotion().getFrameCount();
			// En los extremos de la animación no hay fotograma anterior o siguiente:
			// se recorta (Math.max/min) y esos pocos fotogramas salen menos precisos
			int before = Math.max(mocap.getFirstFrame(), f - k), after = Math.min(n - 1, f + k);
			mocap.apply(before);
			prev = framesAt(mocap.travelledOrigin(before));
			mocap.apply(after);
			next = framesAt(mocap.travelledOrigin(after));
			mocap.apply(f); // vuelve a la postura actual
			cur = framesAt(mocap.travelledOrigin(f));
			h = k * dt;
			standing = false; // con .bvh hay suelo de verdad: se puede estar en el aire
		} else if (liveButton.isSelected() && liveHistory.size() >= 3) {
			// En directo: la última postura y las que llegaron más o menos h y 2h
			// antes. La dinámica sale para la del medio (con un retraso de h).
			LiveSample last = liveHistory.peekLast();
			LiveSample middle = closest(last.nanos() - (long) (DYNAMICS_STEP * 1e9));
			LiveSample first = closest(last.nanos() - (long) (2 * DYNAMICS_STEP * 1e9));
			double span = (last.nanos() - first.nanos()) / 1e9;
			if (span > 0.02 && middle != first && middle != last) {
				prev = first.frames();
				cur = middle.frames();
				next = last.frames();
				h = span / 2;
			}
		} else if (walkButton.isSelected()) {
			h = DYNAMICS_STEP;
			applyWalk(walkTime - h);
			prev = framesAt(origin);
			applyWalk(walkTime + h);
			next = framesAt(origin);
			applyWalk(walkTime);
		}

		dynamics.setBodyMass(massSlider.getValue());
		dynamics.setContactTolerance(liveButton.isSelected() ? 10 : 3);
		if (exoBox.isSelected()) {
			exo.setJoints(hipBox.isSelected(), kneeBox.isSelected(), ankleBox.isSelected());
			exo.setAssist(assistSlider.getValue() / 100);
			exo.setMaxTorque(motorTorqueSlider.getValue());
			exo.setMotorMass(motorMassSlider.getValue());
			exoMass = exo.applyMasses(dynamics, segments);
			dynamics.setAssistance(exo);
		} else {
			dynamics.clearExtraMass();
			dynamics.setAssistance(null);
		}
		return dynamics.compute(prev, cur, next, h, standing);
	}

	/** Postura del historial en directo que llegó más cerca del instante dado. */
	private LiveSample closest(long nanos) {
		LiveSample best = null;
		for (LiveSample s : liveHistory)
			if (best == null || Math.abs(s.nanos() - nanos) < Math.abs(best.nanos() - nanos))
				best = s;
		return best;
	}

	/**
	 * Modo en directo: recoge la última postura del receptor (que escucha en
	 * otro hilo), y si es nueva, mueve el esqueleto. Cada medio segundo
	 * actualiza el texto de estado.
	 */
	private void updateLive(long now) {
		receiver.start(); // no hace nada si ya estaba escuchando
		LiveReceiver.Pose pose = receiver.latest();
		if (pose != null && pose.seq() != lastLiveSeq) {
			lastLiveSeq = pose.seq();
			origin = live.apply(pose.landmarks());
			frameDt = lastLiveNanos == 0 ? 0 : Math.min(0.2, (now - lastLiveNanos) / 1e9);
			lastLiveNanos = now;
			liveCount++;
			liveNew = true;
			refreshSliders();
			dirty = true;
		}
		if (now - liveWindowStart > 500_000_000L) {
			liveRate = liveCount * 1e9 / (now - liveWindowStart);
			liveCount = 0;
			liveWindowStart = now;
			String text;
			if (receiver.getError() != null)
				text = receiver.getError();
			else if (sender != null && !sender.isAlive() && (pose == null || now - pose.nanos() > 1_000_000_000L))
				text = "El programa de la webcam se ha cerrado:\n" + lastLogLine();
			else if (pose == null || now - pose.nanos() > 1_000_000_000L)
				text = sender != null && sender.isAlive()
						? "Arrancando la webcam... (la primera vez instala MediaPipe y tarda unos minutos; si sale su ventana, ponte delante de la cámara de cuerpo entero)"
						: "Esperando datos en el puerto " + receiver.getPort() + ".";
			else
				text = String.format("Recibiendo: %.0f posturas/s%s", liveRate,
						live.legsVisible() ? "" : "\n(no se ven las piernas: en reposo)");
			liveLabel.setText(text);
		}
	}

	/**
	 * Lanza webcam/run.ps1 (que crea el entorno de Python si hace falta y
	 * arranca pose_sender.py) como un PROCESO aparte: otro programa que corre
	 * a la vez que este. Su salida va a webcam/sender.log, para poder mostrar
	 * el error si falla.
	 *
	 * ProcessBuilder recibe el comando como una lista de palabras (sin
	 * comillas ni espacios que interpretar). "-ExecutionPolicy Bypass" evita
	 * que Windows bloquee el script .ps1 (por defecto no deja ejecutar scripts).
	 */
	private void startSender() {
		if (sender != null && sender.isAlive())
			return;
		if (!SENDER_SCRIPT.toFile().exists()) {
			liveLabel.setText("No encuentro " + SENDER_SCRIPT + " (hay que lanzar el holograma desde su carpeta)");
			return;
		}
		try {
			ProcessBuilder pb = new ProcessBuilder("powershell", "-NoProfile", "-ExecutionPolicy", "Bypass", "-File",
					SENDER_SCRIPT.toString());
			pb.redirectErrorStream(true); // errores y salida normal, juntos
			pb.redirectOutput(SENDER_LOG.toFile());
			sender = pb.start();
		} catch (IOException e) {
			liveLabel.setText("No se pudo lanzar la webcam: " + e.getMessage());
		}
	}

	/**
	 * Cierra el programa de la webcam. PowerShell lanza a su vez a Python
	 * (un proceso "hijo"), así que hay que cerrar también a sus descendientes.
	 */
	private void stopSender() {
		if (sender == null)
			return;
		sender.descendants().forEach(ProcessHandle::destroy);
		sender.destroy();
		sender = null;
	}

	/** Última línea con texto del log de la webcam (para mostrar el error). */
	private static String lastLogLine() {
		try {
			List<String> lines = Files.readAllLines(SENDER_LOG);
			for (int i = lines.size() - 1; i >= 0; i--)
				if (!lines.get(i).isBlank())
					return lines.get(i).trim();
		} catch (IOException e) {
			// sin log: no hay nada que mostrar
		}
		return "(mira webcam/sender.log)";
	}

	/** JavaFX lo llama al cerrar la ventana: que no se quede la webcam encendida. */
	@Override
	public void stop() {
		stopSender();
	}

	/** Colorea el holograma y las esferas según el esfuerzo y rellena la tabla de pares. */
	private void showEffort(InverseDynamics.Result r) {
		// Colores: valor de cada segmento en el orden de MakeHumanRig.SEGMENTS
		if (rig != null) {
			double[] values = new double[MakeHumanRig.SEGMENTS.length];
			for (int i = 0; i < values.length; i++)
				values[i] = r.effort().get(byName.get(MakeHumanRig.SEGMENTS[i]));
			clouds.get(0).colorBy(values, cloudPalette);
			clouds.get(1).colorBy(values, sparkPalette);
		}
		for (Segment s : segments) {
			double e = Math.max(0, Math.min(1, r.effort().get(s)));
			jointSpheres.get(s).setMaterial(jointPalette[(int) Math.round(e * (PALETTE_SIZE - 1))]);
		}
		colored = true;

		// Tabla: articulaciones centrales en una columna, las de los lados en
		// dos (D e I), y el pico de cada fila desde el último reinicio
		Map<String, double[]> rows = new LinkedHashMap<>(); // nombre -> {D, I}; LinkedHashMap mantiene el orden
		for (InverseDynamics.Joint j : r.joints()) {
			String name = j.name();
			int col = name.endsWith(" I") ? 1 : 0;
			String row = name.endsWith(" D") || name.endsWith(" I") ? name.substring(0, name.length() - 2) : name;
			if (row.equals("Cabeza"))
				continue; // se ve en el color; en la tabla no aporta y ocupa sitio
			// j.human(): lo que hace la PERSONA (con exo, lo que le queda después
			// de la ayuda de los motores)
			rows.computeIfAbsent(row, x -> new double[] { Double.NaN, Double.NaN })[col] = j.human();
			peaks.merge(row, j.human(), Math::max); // guarda el mayor de los dos
		}
		StringBuilder sb = new StringBuilder(String.format("%-9s%6s%6s%7s%n", "N·m", "D", "I", "pico"));
		rows.forEach((row, v) -> sb.append(String.format("%-9s%6s%6s%7.0f%n", row, number(v[0]), number(v[1]), peaks.get(row))));
		double weight = massSlider.getValue() * 9.81;
		sb.append(String.format("%nApoyo: %s%nSuelo: %.0f N (%.1f veces el peso)%nResidual: %.0f N, %.0f N·m",
				r.support(), norm(r.groundForce()), norm(r.groundForce()) / weight, norm(r.residualForce()),
				norm(r.residualMoment())));
		effortLabel.setText(sb.toString());
	}

	/**
	 * Coloca el dibujo del exo, apunta las estadísticas de los motores y
	 * rellena su tabla: por motor, par y potencia ahora, picos de par y
	 * potencia y la velocidad máxima (rpm) de la articulación. Debajo, el
	 * resumen para dimensionar la batería.
	 */
	private void showExo(Map<Segment, Matrix4> frames, InverseDynamics.Result r) {
		exoView.update(frames, byName, exo, r);
		exo.record(r, frameDt);

		StringBuilder sb = new StringBuilder(String.format("%-9s%5s%5s%6s%5s%5s%n", "Motor", "N·m", "W", "pico", "W", "rpm"));
		for (InverseDynamics.Joint j : r.joints()) {
			if (!exo.actuates(j.segment()))
				continue;
			Exoskeleton.Stats st = exo.getStats().get(j.name());
			sb.append(String.format("%-9s%5.0f%5.0f%6.0f%5.0f%5.0f%n", j.name(), j.exoTorque(), j.exoPower(),
					st.peakTorque, st.peakPower, st.peakRpm));
		}
		double mean = exo.meanPositivePower();
		sb.append(String.format("%nMasa del exo: %.1f kg%n", exoMass));
		if (exo.getTime() > 0)
			sb.append(String.format("Potencia media: %.0f W (frenando %.0f W)%nBatería para 1 h: %.0f Wh%n"
					+ "Par que le quita a la persona: %.0f%%%n", mean, exo.meanNegativePower(),
					mean / Exoskeleton.EFFICIENCY, exo.reduction() * 100));
		else
			sb.append("(reproduce una animación para la\nbatería y el esfuerzo quitado)");
		exoLabel.setText(sb.toString());
	}

	private static String number(double v) {
		return Double.isNaN(v) ? "" : String.format("%.0f", v);
	}

	private static double norm(double[] v) {
		return Math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
	}

	/** Cinemática directa con la postura actual y la pelvis en ese origen. */
	private Map<Segment, Matrix4> framesAt(double[] pelvis) {
		return ForwardKinematics3D.frames(ForwardKinematics3D.computePositions(root, pelvis[0], pelvis[1], pelvis[2]));
	}

	/** Quita el mapa de esfuerzo: colores normales. */
	private void clearEffort() {
		for (PointCloud c : clouds)
			c.resetColor();
		for (Sphere s : jointSpheres.values())
			s.setMaterial(jointMat);
		effortLabel.setText("");
		colored = false;
	}

	/**
	 * Color según el esfuerzo (0 a 1): cian (el del holograma) -> amarillo ->
	 * rojo. Color.interpolate mezcla dos colores: 0 = el primero, 1 = el
	 * segundo.
	 */
	private static Color effortColor(double e) {
		Color yellow = Color.web("#ffd23f"), red = Color.web("#ff2a1f");
		return e < 0.5 ? HOLO.interpolate(yellow, e / 0.5) : yellow.interpolate(red, (e - 0.5) / 0.5);
	}

	/**
	 * Crea las paletas una sola vez: PALETTE_SIZE materiales para los puntos
	 * (autoiluminados, como el holograma), para las chispas (más claros) y
	 * para las esferas del esqueleto (normales, con luz).
	 */
	private void buildPalettes() {
		cloudPalette = new PhongMaterial[PALETTE_SIZE];
		sparkPalette = new PhongMaterial[PALETTE_SIZE];
		jointPalette = new PhongMaterial[PALETTE_SIZE];
		for (int i = 0; i < PALETTE_SIZE; i++) {
			Color c = effortColor((double) i / (PALETTE_SIZE - 1));
			cloudPalette[i] = glowing(c);
			sparkPalette[i] = glowing(c.interpolate(Color.WHITE, 0.55));
			jointPalette[i] = new PhongMaterial(c);
		}
	}

	/** Borra los picos de la tabla (empiezan a contar otra vez). */
	private void resetPeaks() {
		peaks.clear();
		exo.resetStats();
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
		jointMat = new PhongMaterial(Color.web("#e8553f"));
		limitMat = glowing(Color.web("#ff1010")); // autoiluminado: se distingue del naranja normal
		buildPalettes();
		PhongMaterial boneMat = new PhongMaterial(Color.web("#f2c14e"));
		for (Segment s : segments) {
			Sphere joint = new Sphere(JOINT_RADIUS);
			joint.setMaterial(jointMat);
			jointSpheres.put(s, joint);
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
			playButton.setSelected(false);
			origin = skeleton.getOrigin();
			resetPeaks();
			// Referencia a método: equivale a s -> s.resetAngles()
			segments.forEach(Segment::resetAngles);
			refreshSliders();
			dirty = true;
		});
		// ToggleButton: botón que se queda pulsado o no (on/off)
		walkButton = new ToggleButton("Caminar");
		// ToggleGroup: como mucho uno de los dos pulsado (al pulsar uno se suelta
		// el otro). A diferencia de los RadioButton, se pueden soltar los dos.
		ToggleGroup animations = new ToggleGroup();
		walkButton.setToggleGroup(animations);
		playButton.setToggleGroup(animations);
		liveButton.setToggleGroup(animations);
		for (ToggleButton b : new ToggleButton[] { walkButton, playButton, liveButton })
			b.selectedProperty().addListener((obs, old, on) -> {
				// Mientras se anima, los sliders solo muestran (no se pueden mover)
				for (Slider s : axisSliders)
					s.setDisable(on || s.isDisable());
				if (!on) {
					// Al parar, la pelvis vuelve a su altura de reposo
					origin = skeleton.getOrigin();
					dirty = true;
					refreshSliders();
				}
			});

		// En directo
		liveButton.selectedProperty().addListener((obs, old, on) -> {
			liveHistory.clear();
			lastLiveNanos = 0;
			resetPeaks();
			if (on) {
				liveLabel.setText("Arrancando la webcam...");
				if (launchSender)
					startSender();
			} else {
				receiver.stop(); // deja libre el puerto
				stopSender();
				liveLabel.setText("");
			}
		});
		mirrorBox.setSelected(true);
		mirrorBox.selectedProperty().addListener((obs, old, on) -> live.setMirror(on));
		liveLabel.setWrapText(true);

		// Captura de movimiento (.bvh)
		Button load = new Button("Cargar .bvh...");
		load.setOnAction(e -> {
			// FileChooser: el diálogo de "Abrir" del sistema operativo
			FileChooser chooser = new FileChooser();
			chooser.setTitle("Animación de captura de movimiento");
			chooser.getExtensionFilters().add(new FileChooser.ExtensionFilter("BVH", "*.bvh", "*.BVH"));
			if (ANIMATIONS_DIR.toFile().isDirectory())
				chooser.setInitialDirectory(ANIMATIONS_DIR.toAbsolutePath().toFile());
			File file = chooser.showOpenDialog(stage); // null si se cancela
			if (file != null) {
				loadAnimation(file.toPath());
				playButton.setSelected(mocap != null);
			}
		});
		Label speedLabel = new Label();
		// bind: el texto se recalcula solo cada vez que cambia el slider
		speedLabel.textProperty().bind(speedSlider.valueProperty().asString("Velocidad: %.2fx"));
		playButton.setDisable(mocap == null);
		if (mocap == null && animLabel.getText().isEmpty())
			animLabel.setText("Sin animación cargada");

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

		// Mapa de esfuerzo
		Label massLabel = new Label();
		massLabel.textProperty().bind(massSlider.valueProperty().asString("Masa corporal: %.0f kg"));
		massSlider.valueProperty().addListener((obs, old, val) -> {
			resetPeaks();
			dirty = true;
		});
		effortBox.selectedProperty().addListener((obs, old, on) -> {
			resetPeaks();
			dirty = true;
		});
		// Letra de ancho fijo para que las columnas de la tabla salgan alineadas
		effortLabel.setFont(Font.font("Consolas", 12));
		Label legend = new Label("cian = poco, amarillo = medio,\nrojo = cerca del par máximo");

		// Exoesqueleto: casillas de los motores y sliders. Cualquier cambio
		// reinicia las estadísticas (si no, se mezclarían configuraciones).
		hipBox.setSelected(true);
		kneeBox.setSelected(true);
		HBox motors = new HBox(10, hipBox, kneeBox, ankleBox); // en fila
		Label assistLabel = new Label(), torqueLabel = new Label(), motorMassLabel = new Label();
		assistLabel.textProperty().bind(assistSlider.valueProperty().asString("Asistencia: %.0f%% del par"));
		torqueLabel.textProperty().bind(motorTorqueSlider.valueProperty().asString("Par máximo por motor: %.0f N·m"));
		motorMassLabel.textProperty().bind(motorMassSlider.valueProperty().asString("Masa por motor: %.1f kg"));
		for (CheckBox c : new CheckBox[] { exoBox, hipBox, kneeBox, ankleBox })
			c.selectedProperty().addListener((obs, old, on) -> {
				resetPeaks();
				dirty = true;
			});
		for (Slider sl : new Slider[] { assistSlider, motorTorqueSlider, motorMassSlider })
			sl.valueProperty().addListener((obs, old, val) -> {
				resetPeaks();
				dirty = true;
			});
		exoLabel.setFont(Font.font("Consolas", 12));
		Label exoLegend = new Label("motor naranja = empuja, verde = frena");
		exoLegend.setStyle("-fx-font-size: 11; -fx-text-fill: #666;");
		legend.setStyle("-fx-font-size: 11; -fx-text-fill: #666;");

		box.getChildren().addAll(new Separator(), reset, walkButton, new Separator(),
				new Label("Captura de movimiento"), load, playButton, animLabel, speedLabel, speedSlider,
				new Separator(), liveButton, mirrorBox, liveLabel, new Separator(),
				showCloud, showSkeleton, new Separator(),
				effortBox, legend, massLabel, massSlider, effortLabel, new Separator(),
				exoBox, exoLegend, motors, assistLabel, assistSlider, torqueLabel, motorTorqueSlider, motorMassLabel,
				motorMassSlider, exoLabel, new Separator(),
				new Label("Puntos del holograma"), pointSlider, cloudLabel, new Separator(), statsLabel);
		box.setPadding(new Insets(14));
		box.setPrefWidth(290);

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
			sl.setDisable(!free || walkButton != null && walkButton.isSelected() || playButton.isSelected()
					|| liveButton.isSelected());
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
	 * [--show=cloud,skeleton] [--points=n] [--bvh=fichero --bvhtime=segundos]
	 * [--effort=1] [--exo=1 [--ankle=1]] [--live=1].
	 * Espera 2 s a que se dibuje la escena, la guarda en PNG y cierra.
	 */
	private void takeSnapshotAndExit(Scene scene, String file) {
		Map<String, String> args = getParameters().getNamed();
		if (args.containsKey("walk")) {
			applyWalk(Double.parseDouble(args.get("walk")));
			refreshSliders();
			dirty = true;
		}
		if (args.containsKey("bvhtime") && mocap != null) {
			animTime = Double.parseDouble(args.get("bvhtime"));
			// Pulsado pero sin avanzar (velocidad 0): así el mapa de esfuerzo usa la
			// dinámica de la animación y no el caso estático
			speedSlider.setValue(0);
			playButton.setSelected(true);
			applyAnimation();
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

"""
Detecta la postura del cuerpo con MediaPipe (webcam, vídeo o imagen) y la
manda al holograma por UDP.

Uso (desde la carpeta del proyecto, con el entorno de webcam/.venv):
    webcam/.venv/Scripts/python webcam/pose_sender.py                 # webcam 0
    webcam/.venv/Scripts/python webcam/pose_sender.py --camera 1      # otra webcam
    webcam/.venv/Scripts/python webcam/pose_sender.py --video fondo.mp4 --loop
    webcam/.venv/Scripts/python webcam/pose_sender.py --image foto.jpg

En el holograma hay que pulsar "En directo (webcam)". En la ventana de vista
previa, Q o Esc para salir.

Cómo funciona:
1. OpenCV lee cada fotograma (de la cámara o del fichero).
2. MediaPipe Pose Landmarker (una red neuronal) encuentra 33 puntos del cuerpo.
   Da dos versiones: en la imagen (píxeles, para dibujarlos encima) y en el
   "mundo" (metros, en 3D, con el origen entre las caderas). Al holograma se
   le manda la del mundo.
3. Se empaqueta como JSON, {"t": segundos, "lm": [[x, y, z, visibilidad], ...]},
   y se manda en un datagrama UDP a 127.0.0.1:5005 (este mismo ordenador).
   UDP no espera confirmación: si un paquete se pierde, da igual, llega otro
   enseguida.
"""

import argparse
import json
import os
import socket
import time
import urllib.request

import cv2
import mediapipe as mp
from mediapipe.tasks.python import BaseOptions, vision

MODEL_URL = ("https://storage.googleapis.com/mediapipe-models/pose_landmarker/"
             "pose_landmarker_full/float16/latest/pose_landmarker_full.task")
MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "models", "pose_landmarker_full.task")


def ensure_model():
    """El modelo (la red neuronal ya entrenada, ~9 MB) se descarga la primera vez."""
    if not os.path.exists(MODEL_PATH):
        os.makedirs(os.path.dirname(MODEL_PATH), exist_ok=True)
        print("Descargando el modelo de MediaPipe...")
        urllib.request.urlretrieve(MODEL_URL, MODEL_PATH)
    return MODEL_PATH


def draw(frame, landmarks):
    """Dibuja los puntos y las líneas del esqueleto encima de la imagen."""
    h, w = frame.shape[:2]
    pts = [(int(l.x * w), int(l.y * h)) for l in landmarks]
    for c in vision.PoseLandmarksConnections.POSE_LANDMARKS:
        cv2.line(frame, pts[c.start], pts[c.end], (255, 214, 56), 2)
    for p in pts:
        cv2.circle(frame, p, 3, (40, 90, 255), -1)


def main():
    ap = argparse.ArgumentParser(description="Manda la postura de la webcam (o de un vídeo) al holograma")
    ap.add_argument("--camera", type=int, default=0, help="número de la webcam (0 = la de serie)")
    ap.add_argument("--video", help="fichero de vídeo en vez de la webcam")
    ap.add_argument("--image", help="una imagen fija (se manda una y otra vez; para probar)")
    ap.add_argument("--loop", action="store_true", help="con --video, volver a empezar al acabar")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5005)
    ap.add_argument("--no-preview", action="store_true", help="sin ventana de vista previa")
    ap.add_argument("--seconds", type=float, default=0, help="parar a los N segundos (0 = nunca)")
    args = ap.parse_args()

    # Modo VIDEO: MediaPipe aprovecha el fotograma anterior para seguir a la
    # persona (más rápido y más estable que buscarla desde cero cada vez)
    options = vision.PoseLandmarkerOptions(
        base_options=BaseOptions(model_asset_path=ensure_model()),
        running_mode=vision.RunningMode.VIDEO,
        min_pose_detection_confidence=0.5,
        min_tracking_confidence=0.5)
    landmarker = vision.PoseLandmarker.create_from_options(options)

    if args.image:
        still = cv2.imread(args.image)
        if still is None:
            raise SystemExit(f"No se puede abrir la imagen {args.image}")
        cap, fps = None, 30.0
    else:
        cap = cv2.VideoCapture(args.video if args.video else args.camera)
        if not cap.isOpened():
            raise SystemExit("No se puede abrir " + (args.video or f"la webcam {args.camera}"))
        # Con un vídeo se va a su velocidad real (la dinámica del holograma
        # necesita tiempos reales); con la webcam, tan rápido como llegue
        fps = cap.get(cv2.CAP_PROP_FPS) or 30.0

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    start = time.monotonic()
    timestamp_ms = 0
    sent = 0
    shown_fps = 0.0
    last = time.monotonic()
    print(f"Mandando posturas a {args.host}:{args.port}. Q o Esc para salir.")

    while True:
        if args.seconds and time.monotonic() - start > args.seconds:
            break
        if cap is None:
            frame = still.copy()
        else:
            ok, frame = cap.read()
            if not ok:
                if args.video and args.loop:
                    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    continue
                break

        # MediaPipe quiere RGB y OpenCV da BGR (los canales al revés)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        # En modo VIDEO los tiempos tienen que ir siempre hacia delante
        timestamp_ms += int(1000 / fps) if (args.video or args.image) else max(1, int((time.monotonic() - start) * 1000) - timestamp_ms)
        result = landmarker.detect_for_video(image, timestamp_ms)

        if result.pose_world_landmarks:
            world = result.pose_world_landmarks[0]
            msg = {"t": time.time(),
                   "lm": [[round(l.x, 4), round(l.y, 4), round(l.z, 4), round(l.visibility or 0, 3)] for l in world]}
            sock.sendto(json.dumps(msg).encode("utf-8"), (args.host, args.port))
            sent += 1
            if not args.no_preview:
                draw(frame, result.pose_landmarks[0])

        now = time.monotonic()
        shown_fps = 0.9 * shown_fps + 0.1 / max(now - last, 1e-6)
        last = now
        if not args.no_preview:
            cv2.putText(frame, f"{shown_fps:.0f} fps  enviadas: {sent}", (10, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            cv2.imshow("Holograma - webcam (Q para salir)", frame)
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                break
        if args.video or args.image:
            # Ir a la velocidad real del vídeo
            time.sleep(max(0.0, 1 / fps - (time.monotonic() - now)))

    print(f"Enviadas {sent} posturas.")
    if cap is not None:
        cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

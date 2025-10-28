#!/usr/bin/env python3

# ==============================================================================
#  AsyncHandPoseYolo11 – Entwickler-Dokumentation
# ==============================================================================
#
# 📌 Ziel
# Dieses Skript realisiert eine asynchrone Kamera- und GPU-Pipeline für Handpose-Erkennung
# mit YOLO11n (TensorRT). Es zeigt, wie CPU-Preprocessing und GPU-Inferenz parallelisiert
# werden, um Echtzeitfähigkeit auf Jetson-Systemen zu erreichen.
#
# 📁 Hauptmodule & Klassen
# - HandPose: TensorRT-Wrapper, enthält Preprocessing, Inferenz und Dekodierung.
# - CaptureThread: Liest Frames von Kamera, preprocessiert, legt Daten in Queue.
# - InferenceThread: Führt GPU-Inferenz aus, legt Ergebnisse in zweite Queue.
# - HandPoseApp: Koordiniert Threads, Anzeige, FPS-Messung und sauberen Shutdown.
#
# ⚙️ Ablaufübersicht
# 1. Kamera wird konfiguriert (feste FPS, Exposure).
# 2. CaptureThread liest kontinuierlich Frames → preprocessiert → Queue.
# 3. InferenceThread liest asynchron → GPU-Inferenz → Resultate in Ausgabequeue.
# 4. Main Loop zeigt Bounding Boxes, Keypoints, FPS an.
# 5. ESC beendet Anwendung sicher.
#
# 🚀 Performance
# - Optimiert für Jetson (NVPModel, Jetson_Clocks).
# - FP16/INT8 TensorRT-Modelle empfohlen.
# - Nutzung von GStreamer-Pipelines mit MJPEG empfohlen.
#
# 👀 Debugging & Trace
# Das Skript nutzt eine Trace-Funktion für zeitgenaue Logs von Capture-, Inferenz- und Displayphasen.
# ==============================================================================

# AsyncHandPoseYolo11.py
# Kamera → Preprocess → GPU → Anzeige mit asynchroner Queue-Pipeline
#
# Dieses Skript wurde umfassend kommentiert, um alle Schritte von der Kamerakonfiguration
# bis zur GPU-Inferenz mit YOLO11n (Pose-Modell) verständlich zu machen.
# Es nutzt Threads und Queues, um CPU- und GPU-Aufgaben parallel auszuführen.

import cv2, numpy as np, tensorrt as trt
import pycuda.driver as cuda, pycuda.autoinit
import threading, queue, time
from time import perf_counter_ns
import subprocess
from trace import Trace, PrintTrace  # Benutzerdefinierte Trace-Funktionen (nicht-blockierende Log-Ausgabe)

# === Globale Parameter ===
ENGINE_PATH = "/home/harald/YOLO11n-pose-hands/runs/pose/train/weights/best.engine"  # Pfad zur TensorRT Engine
INPUT_SIZE = 640   # Inputgröße für YOLO11 (640x640)
CONF_THRESH = 0.3  # Mindestkonfidenz für Detektionen
CAM_ID = 2         # Kamera-ID (z. B. /dev/video2)

# Kamera-Auflösung und Framerate – diese Werte bestimmen das Eingangssignal.
# Es wird ein Ausschnitt (Crop) aus dem oberen Bereich des Bildes genutzt,
# um den Blickwinkel der Kamera zu optimieren (z. B. Handerkennung von oben).
CAM_W, CAM_H, CAM_FPS = 1920, 1080, 30

# === Hilfsfunktion: Bildausschnitt berechnen ===
def CenterCrop(image, crop_size):
    # Schneidet einen 640x640-Bereich am oberen Bildrand aus.
    if isinstance(crop_size, int):
        crop_w = crop_h = crop_size
    else:
        crop_w, crop_h = crop_size

    h, w = image.shape[:2]
    crop_w = min(crop_w, w)
    crop_h = min(crop_h, h)

    start_x = (w - crop_w) // 2
    start_y = 0  # nur oberer Rand
    cropped = image[start_y:start_y + crop_h, start_x:start_x + crop_w]
    return cropped

# --------------------------------------------------------------------------
# Klasse: HandPose – TensorRT Wrapper für YOLO11n-Pose
# --------------------------------------------------------------------------
class HandPose:
    def __init__(self):
        TRT_LOGGER = trt.Logger(trt.Logger.INFO)
        with open(ENGINE_PATH, "rb") as f, trt.Runtime(TRT_LOGGER) as runtime:
            engine = runtime.deserialize_cuda_engine(f.read())
        self.context = engine.create_execution_context()

        self.tensor_buffers = {}
        self.input_name = self.output_name = None

        for i in range(engine.num_io_tensors):
            name = engine.get_tensor_name(i)
            shape = tuple(self.context.get_tensor_shape(name))
            mode = engine.get_tensor_mode(name)

            size = int(np.prod(shape)) * np.float32().nbytes
            buf = cuda.mem_alloc(size)
            self.tensor_buffers[name] = buf
            self.context.set_tensor_address(name, int(buf))

            if mode == trt.TensorIOMode.INPUT:
                self.input_name = name
            else:
                self.output_name = name
                self.output_shape = shape

        if self.input_name is None or self.output_name is None:
            raise RuntimeError("Keine Input/Output-Tensors gefunden.")

    def Inference(self, img):
        # Führt eine synchrone TensorRT-Inferenz aus.
        cuda.memcpy_htod(self.tensor_buffers[self.input_name], img)
        self.context.execute_v2([int(self.tensor_buffers[n]) for n in self.tensor_buffers])
        host_output = np.empty(self.output_shape, dtype=np.float32)
        cuda.memcpy_dtoh(host_output, self.tensor_buffers[self.output_name])
        return host_output

    @staticmethod
    def DecodeYolo11Output(output, orig_shape, conf_thresh=0.5):
        # Dekodiert das YOLO11-Pose-Ausgabeformat in Bounding Boxes und Keypoints.
        out = output[0].T
        boxes = out[:, :4]
        scores = out[:, 4]
        kpts = out[:, 5:].reshape(-1, 21, 3)
        mask = scores > conf_thresh
        boxes, scores, kpts = boxes[mask], scores[mask], kpts[mask]
        if len(boxes) == 0:
            return np.zeros((0, 4)), np.zeros((0,)), np.zeros((0, 21, 3))
        boxes_xyxy = np.zeros_like(boxes)
        boxes_xyxy[:, 0] = boxes[:, 0] - boxes[:, 2]/2
        boxes_xyxy[:, 1] = boxes[:, 1] - boxes[:, 3]/2
        boxes_xyxy[:, 2] = boxes[:, 0] + boxes[:, 2]/2
        boxes_xyxy[:, 3] = boxes[:, 1] + boxes[:, 3]/2
        H, W = orig_shape[:2]
        boxes_xyxy[:, [0,2]] *= W / 640
        boxes_xyxy[:, [1,3]] *= H / 640
        kpts[:, :, 0] *= W / 640
        kpts[:, :, 1] *= H / 640
        return boxes_xyxy, scores, kpts

    @staticmethod
    def DrawPose(frame, boxes, kpts, scores, fps=None):
        # Zeichnet Bounding Boxes und Keypoints in das Bild.
        for i, box in enumerate(boxes):
            x1, y1, x2, y2 = box.astype(int)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)
            for (x, y, c) in kpts[i]:
                if c > 0.3:
                    cv2.circle(frame, (int(x), int(y)), 2, (255,0,0), -1)
        if fps is not None:
            cv2.putText(frame, f"{fps:.1f} FPS", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
        return frame

    @staticmethod
    def Preprocess(frame_bgr):
        # Bereitet das Kamerabild für die Inferenz vor (Normalisierung, RGB, Shape).
        img = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        img = CenterCrop(img, INPUT_SIZE)
        img = img.astype(np.float32) / 255.0
        img = np.transpose(img, (2, 0, 1))
        img = np.expand_dims(img, axis=0)
        return np.ascontiguousarray(img)

# --------------------------------------------------------------------------
# Kamera-Thread: Liest Frames, preprocessiert und legt sie in Queue
# --------------------------------------------------------------------------
class CaptureThread(threading.Thread):
    def __init__(self, cam_id, width, height, fps, queue_out):
        super().__init__(daemon=True)
        self.running = True
        self.q_out = queue_out

        gst_str = (
            f"v4l2src device=/dev/video{cam_id} ! "
            f"image/jpeg,width={width},height={height},framerate={fps}/1 ! "
            "jpegdec ! videoconvert ! appsink drop=true sync=false"
        )

        print("[INFO] Öffne Kamera via GStreamer:")
        print("   ", gst_str)

        self.cap = cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            raise RuntimeError("Kamera konnte nicht über GStreamer geöffnet werden")

        time.sleep(0.2)
        print(f"[INFO] Kamera gestartet @ {width}x{height}@{fps}fps (MJPEG via GStreamer)")

    def run(self):
        # Hauptschleife: Kamera lesen, Croppen, Spiegeln, Preprocessen, Queue füllen.
        while self.running:
            ok, frame = self.cap.read()
            if not ok:
                raise RuntimeError(f"Kamera konnte nicht gelesen werden")

            t0 = perf_counter_ns()
            frame = CenterCrop(frame, INPUT_SIZE)
            frame = cv2.flip(frame, 1)
            img = HandPose.Preprocess(frame)

            dt_ms = (perf_counter_ns() - t0) / 1e6

            frameDropped = False
            if self.q_out.full():
                try:
                    _ = self.q_out.get_nowait()
                    frameDropped = True
                except queue.Empty:
                    pass
            self.q_out.put_nowait((img, frame, frame.shape))

            if frameDropped: Trace(f"CaptureThread {dt_ms:.1f} ms (one frame dropped)")
            else: Trace(f"CaptureThread {dt_ms:.1f} ms")

    def stop(self):
        self.running = False
        time.sleep(0.1)
        self.cap.release()
        print("[INFO] Capture-Thread gestoppt")

# --------------------------------------------------------------------------
# Inferenz-Thread – führt GPU-Inferenz mit eigenem CUDA-Kontext aus
# --------------------------------------------------------------------------
class InferenceThread(threading.Thread):
    def __init__(self, q_in, q_out):
        super().__init__(daemon=True)
        self.q_in, self.q_out = q_in, q_out
        self.running = True

    def run(self):
        self.cuda_ctx = cuda.Device(0).make_context()
        try:
            self.hp = HandPose()
            print("[INFO] Inference-Thread CUDA-Context aktiv (ohne Double Buffer)")

            while self.running:
                img, frame, shape = self.q_in.get()
                t0 = perf_counter_ns()
                Trace(f"Inference started")

                output = self.hp.Inference(img)

                result = (output.copy(), frame.copy(), shape)
                dt_ms = (perf_counter_ns() - t0) / 1e6
                Trace(f"Inference done {dt_ms:.1f} ms")
                self.q_out.put_nowait(result)
        finally:
            self.cuda_ctx.pop()
            print("[INFO] Inference-Thread CUDA-Context freigegeben")

    def stop(self):
        self.running = False

# --------------------------------------------------------------------------
# Hilfsfunktionen – Kamera konfigurieren, FPS berechnen
# --------------------------------------------------------------------------
def CameraConfig():
    # Setzt feste Kamera-Parameter (keine dynamische Framerate).
    cmds = [
        ["v4l2-ctl", "-d", "/dev/video2", "--set-ctrl=auto_exposure=3"],
        ["v4l2-ctl", "-d", "/dev/video2", "--get-ctrl=auto_exposure"],
        ["v4l2-ctl", "-d", "/dev/video2", "--set-ctrl=exposure_dynamic_framerate=0"],
        ["v4l2-ctl", "-d", "/dev/video2", "--get-ctrl=exposure_dynamic_framerate"],
        ["v4l2-ctl", "-d", "/dev/video2", "--get-ctrl=exposure_time_absolute"]
    ]
    print("----------- Camera Configuration -----------")
    for cmd in cmds:
        subprocess.run(cmd, check=False)
    time.sleep(0.2)

class FpsCalc():
    # Einfacher FPS-Zähler basierend auf Zeitintervallen.
    def __init__(self, duration_sec=1.0):
        self.fpsCnt = 0
        self.fps = 0.03
        self.duration = int(duration_sec*1e9)
        self.t_last = time.perf_counter_ns()

    def Value(self):
        self.fpsCnt += 1
        now = time.perf_counter_ns()
        if now - self.t_last >= self.duration:
            self.fps = self.fpsCnt * 1e9 / (now - self.t_last)
            self.t_last = now
            self.fpsCnt = 0
        return self.fps

# --------------------------------------------------------------------------
# Hauptklasse: HandPoseApp – steuert Threads, GUI, FPS
# --------------------------------------------------------------------------
class HandPoseApp():
    def __init__(self, CAM_ID):
        CameraConfig()
        self.q_cam, self.q_res = queue.Queue(maxsize=2), queue.Queue(maxsize=2)

        self.inf_thread = InferenceThread(self.q_cam, self.q_res)
        self.inf_thread.start()
        time.sleep(0.5)

        self.cam_thread = CaptureThread(CAM_ID, CAM_W, CAM_H, CAM_FPS, queue_out=self.q_cam)
        self.cam_thread.start()

        self.win = "YOLO11n Pose (Preprocess@Capture)"
        cv2.namedWindow(self.win, cv2.WINDOW_NORMAL)
        cv2.setWindowProperty(self.win, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        self.fpsCalc = FpsCalc()
        self.fps = 30

    def ProcessFrame(self):
        # Liest Inferenz-Resultate, dekodiert sie und zeigt sie an.
        output, frame, shape = self.q_res.get()
        t0 = perf_counter_ns()

        boxes, scores, kpts = HandPose.DecodeYolo11Output(output, shape, conf_thresh=CONF_THRESH)
        self.fps = self.fpsCalc.Value()
        disp = HandPose.DrawPose(frame, boxes, kpts, scores, fps=self.fps)
        cv2.imshow(self.win, disp)

        dt_ms = (perf_counter_ns() - t0) / 1e6
        Trace(f"DisplayThread {dt_ms:.1f} ms")

        exitFlag = cv2.waitKey(1) & 0xFF == 27
        if len(boxes) > 0:
            x1, y1, x2, y2 = boxes[0].astype(int)
            handSize = y2 - y1
            handPos = (x1 + x2) // 2 - disp.shape[1] // 2
        else:
            handSize = 0
            handPos = 0
        return exitFlag, handSize, handPos

    def Exit(self):
        # Beendet Threads und GUI ordentlich.
        self.cam_thread.stop()
        self.inf_thread.stop()
        cv2.destroyAllWindows()
        PrintTrace()
        
    def GetFps(self):
        return self.fps
    

# --------------------------------------------------------------------------
# Hauptprogramm – Startet App, Schleife, beendet sauber
# --------------------------------------------------------------------------
def WebCamDemo():
    app = HandPoseApp(CAM_ID)
    try:
        while True:
            ret = app.ProcessFrame()
            if ret:
                break
    finally:
        app.Exit()

if __name__ == "__main__":
    WebCamDemo()

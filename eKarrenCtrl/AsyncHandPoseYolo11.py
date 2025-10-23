#!/usr/bin/env python3
# AsyncHandPoseYolo11_CamPreprocess.py
# Kamera → Preprocess → GPU → Anzeige mit asynchroner Queue-Pipeline

import cv2, numpy as np, tensorrt as trt
import pycuda.driver as cuda, pycuda.autoinit
import threading, queue, time
from time import perf_counter_ns
import subprocess
from trace import Trace, PrintTrace


# === Konfiguration ===
ENGINE_PATH = "/home/harald/YOLO11n-pose-hands/runs/pose/train/weights/best.engine"
INPUT_SIZE = 640
CONF_THRESH = 0.3
CAM_ID = 2
#CAM_W, CAM_H, CAM_FPS = 1280, 720, 30

# Wir verwenden ein 640x640 Ausschnitt mittig am oberen Bildrand des 
# 1920x1080 Originalbildes.
# Das Originalbild muss möglichst gross sein, um einen Zoomeffekt zu erreichen!
# Der obere Bildrand wird verwendet, um den Winkel um den die Kamera
# nach oben schaut zu minimieren
CAM_W, CAM_H, CAM_FPS = 1920, 1080, 30


# Crop the center region of an image.
# Args:
#     image (np.ndarray): Input image (H x W x C)
#     crop_size (int or tuple): Size of the crop (e.g. 416 or (416, 416))
# Returns:
#     Cropped image (np.ndarray) am oberen Bildrand
def CenterCrop(image, crop_size):
    if isinstance(crop_size, int):
        crop_w = crop_h = crop_size
    else:
        crop_w, crop_h = crop_size

    h, w = image.shape[:2]

    # Ensure crop size does not exceed image size
    crop_w = min(crop_w, w)
    crop_h = min(crop_h, h)

    start_x = (w - crop_w) // 2
    #start_y = (h - crop_h) // 2
    start_y = 0   
    
    cropped = image[start_y:start_y + crop_h, start_x:start_x + crop_w]
    return cropped




# --------------------------------------------------------------------------
# TensorRT Wrapper
# --------------------------------------------------------------------------
class HandPose:
    def __init__(self):
        TRT_LOGGER = trt.Logger(trt.Logger.INFO)
        with open(ENGINE_PATH, "rb") as f, trt.Runtime(TRT_LOGGER) as runtime:
            engine = runtime.deserialize_cuda_engine(f.read())
        self.context = engine.create_execution_context()

        # Tensor-Puffer vorbereiten
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

    # === GPU inference ===
    def Inference(self, img):
        """Synchroner TensorRT-Inferenzaufruf (ohne Double Buffer)"""
        # Eingabedaten an GPU senden
        cuda.memcpy_htod(self.tensor_buffers[self.input_name], img)

        # Ausführen – synchron
        self.context.execute_v2([int(self.tensor_buffers[n]) for n in self.tensor_buffers])

        # Ausgabe aus GPU-Speicher lesen
        host_output = np.empty(self.output_shape, dtype=np.float32)
        cuda.memcpy_dtoh(host_output, self.tensor_buffers[self.output_name])

        return host_output

    # === Postprocessing ===
    @staticmethod
    def DecodeYolo11Output(output, orig_shape, conf_thresh=0.5):
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
        for i, box in enumerate(boxes):
            x1, y1, x2, y2 = box.astype(int)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)
            for (x, y, c) in kpts[i]:
                if c > 0.3:
                    cv2.circle(frame, (int(x), int(y)), 2, (255,0,0), -1)
        if fps is not None:
            cv2.putText(frame, f"{fps:.1f} FPS", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
        return frame

    # === Preprocessing (jetzt für Kamera-Thread) ===
    @staticmethod
    def Preprocess(frame_bgr):
        img = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        img = CenterCrop(img, INPUT_SIZE)
        #img = cv2.resize(img, (INPUT_SIZE, INPUT_SIZE))
        img = img.astype(np.float32) / 255.0
        img = np.transpose(img, (2, 0, 1))
        img = np.expand_dims(img, axis=0)
        return np.ascontiguousarray(img)
        
# --------------------------------------------------------------------------
# Kamera-Thread mit Preprocessing
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
        while self.running:
            ok, frame = self.cap.read()
            if not ok:
                raise RuntimeError(f"Kamera konnte nicht gelesen werden")

            t0 = perf_counter_ns()
            frame = CenterCrop(frame, INPUT_SIZE)
            # Mirror
            frame = cv2.flip(frame, 1)
            img = HandPose.Preprocess(frame)

            # Messung CPU-Zeit
            dt_ms = (perf_counter_ns() - t0) / 1e6

            # Send preprocessed image to queue
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
# Inferenz-Thread (nur GPU)
# --------------------------------------------------------------------------
class InferenceThread(threading.Thread):
    def __init__(self, q_in, q_out):
        super().__init__(daemon=True)
        self.q_in, self.q_out = q_in, q_out
        #self.hp = handPose
        self.running = True

    def run(self):
        # Eigenen CUDA-Context für diesen Thread anlegen
        self.cuda_ctx = cuda.Device(0).make_context()
        try:
            self.hp = HandPose()
            print("[INFO] Inference-Thread CUDA-Context aktiv (ohne Double Buffer)")
            
            while self.running:
                # Warten bis ein Frame aus der Kamera kommt
                img, frame, shape = self.q_in.get()  # blockierend

                t0 = perf_counter_ns()
                Trace(f"Inference started")

                # Direkte (synchronisierte) Inference — keine Slots
                output = self.hp.Inference(img)

                # Ergebnis an den Anzeige-Thread übergeben
                result = (output.copy(), frame.copy(), shape)

                # Messung GPU-Zeit
                dt_ms = (perf_counter_ns() - t0) / 1e6
                Trace(f"Inference done {dt_ms:.1f} ms")
                #if self.q_out.full(): 
                #    try:
                #        _ = self.q_out.get_nowait()
                #    except queue.Empty:
                #        pass
                self.q_out.put_nowait(result)

        finally:
            self.cuda_ctx.pop()
            print("[INFO] Inference-Thread CUDA-Context freigegeben")

    def stop(self):
        self.running = False


# Kamera läuft nur dann mit 30fps, wenn es hell genug ist
def CameraConfig():
    cmds = [
        # Aperture Priority Mode“ (Blendenpriorität) bedeutet:
        # Die Kamera hält die Blende fest und passt Belichtungszeit (und ggf. Gain) automatisch an,
        # um ein korrekt belichtetes Bild zu erzeugen (1=Manual mode,  3=Aperture Priority Mode).
        ["v4l2-ctl", "-d", "/dev/video2", "--set-ctrl=auto_exposure=3"],
        ["v4l2-ctl", "-d", "/dev/video2", "--get-ctrl=auto_exposure"],

        # exposure_dynamic_framerate erlaubt oder verbietet der Kamera,
        # die tatsächliche Bildrate (Framerate) zu ändern,
        # wenn sie versucht, eine korrekte Belichtung zu halten
        ["v4l2-ctl", "-d", "/dev/video2", "--set-ctrl=exposure_dynamic_framerate=1"],
        ["v4l2-ctl", "-d", "/dev/video2", "--get-ctrl=exposure_dynamic_framerate"],

        # exposure_time_absolute kann nur bei Manual Mode gesetzt werden.
        # ["v4l2-ctl", "-d", "/dev/video2", "--set-ctrl=exposure_time_absolute=150"],
        ["v4l2-ctl", "-d", "/dev/video2", "--get-ctrl=exposure_time_absolute"]
    ]
    print("----------- Camera Configuration -----------")
    for cmd in cmds:
        result = subprocess.run(cmd, check=False)
    time.sleep(0.2)


class FpsCalc():
    def __init__(self, duration_sec=1.0):
        self.fpsCnt = 0
        self.fps = 0
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
        #cv2.resizeWindow(win, INPUT_SIZE, INPUT_SIZE)
        cv2.setWindowProperty(self.win, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        self.fpsCalc = FpsCalc()

    def ProcessFrame(self):
        output, frame, shape = self.q_res.get()
        t0 = perf_counter_ns()
        ##boxes, scores, kpts = inf_thread.hp.DecodeYolo11Output(output, shape, conf_thresh=CONF_THRESH)
        boxes, scores, kpts = HandPose.DecodeYolo11Output(output, shape, conf_thresh=CONF_THRESH)
        
        fps = self.fpsCalc.Value()

        disp = HandPose.DrawPose(frame, boxes, kpts, scores, fps=fps)
        #Trace(f"{disp.shape=}")
        cv2.imshow(self.win, disp)

        dt_ms = (perf_counter_ns() - t0) / 1e6
        Trace(f"DisplayThread {dt_ms:.1f} ms")

        exitFlag = cv2.waitKey(1) & 0xFF == 27
        if len(boxes) > 0:
            x1, y1, x2, y2 = boxes[0].astype(int)
            handSize = y2 - y1
            handPos = (x1 + x2) // 2 
            handPos -= disp.shape[1] // 2
        else:
            handSize = 0
            handPos = 0
        return exitFlag, handSize, handPos

    def Exit(self):
        self.cam_thread.stop()
        self.inf_thread.stop()
        cv2.destroyAllWindows()
        PrintTrace()
 

# --------------------------------------------------------------------------
# Hauptprogramm (Anzeige + Performance-Statistik)
# --------------------------------------------------------------------------
def WebCamDemo(): 
    app = HandPoseApp(CAM_ID)
    try:
        while True:
            ret = app.ProcessFrame()
            if ret: break
    finally:
        app.Exit()
        
if __name__ == "__main__":
    WebCamDemo()

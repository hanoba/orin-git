#!/usr/bin/env python3
# AsyncHandPoseYolo11_PingPong.py – echtes Overlap mit Double Buffer + CUDA Events (GPU nur im Main-Thread)

import cv2, numpy as np, tensorrt as trt
import pycuda.driver as cuda, pycuda.autoinit
import threading, queue, time

# ===== Konfiguration =====
ENGINE_PATH = "/home/harald/YOLO11n-pose-hands/runs/pose/train/weights/best.engine"
INPUT_SIZE = 640
CONF_THRESH = 0.4
CAM_ID = 2
CAM_W, CAM_H, CAM_FPS = 1280, 720, 30

# ====== Kamera-Thread (V4L2, MJPG, Buffer=1, neuestes Frame) ======
class CaptureThread(threading.Thread):
    def __init__(self, cam_id=0, width=1280, height=720, fps=30):
        super().__init__(daemon=True)
        self.q = queue.Queue(maxsize=1)
        self.running = True
        self.cap = cv2.VideoCapture(cam_id, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self.cap.isOpened():
            raise RuntimeError(f"Kamera {cam_id} konnte nicht geöffnet werden")
        # ein paar Warmup-Reads, damit Exposure stabil ist
        for _ in range(6):
            self.cap.read()

    def run(self):
        while self.running:
            ok, frame = self.cap.read()
            if not ok:
                continue
            if self.q.full():
                try: _ = self.q.get_nowait()
                except queue.Empty: pass
            self.q.put_nowait(frame)

    def get(self, timeout=0.0):
        try: return True, self.q.get(timeout=timeout)
        except queue.Empty: return False, None

    def stop(self):
        self.running = False
        self.cap.release()

# ====== TensorRT-Wrapper mit Ping-Pong-Buffering ======
class HandPose:
    def __init__(self):
        TRT_LOGGER = trt.Logger(trt.Logger.INFO)
        with open(ENGINE_PATH, "rb") as f, trt.Runtime(TRT_LOGGER) as runtime:
            self.engine = runtime.deserialize_cuda_engine(f.read())
        self.context = self.engine.create_execution_context()
        self.stream = cuda.Stream()

        # I/O Tensor-Namen & Größen ermitteln
        self.input_name = self.output_name = None
        self.output_shape = None
        elt = np.float32().nbytes
        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            mode = self.engine.get_tensor_mode(name)
            shape = tuple(self.context.get_tensor_shape(name))
            print(f"[INFO] Tensor {name}: {shape} ({'IN' if mode==trt.TensorIOMode.INPUT else 'OUT'})")
            nbytes = int(np.prod(shape)) * elt
            if mode == trt.TensorIOMode.INPUT:
                self.input_name = name
                self.in_bytes = nbytes
            else:
                self.output_name = name
                self.output_shape = shape
                self.out_bytes = nbytes
        if self.input_name is None or self.output_name is None:
            raise RuntimeError("Input/Output-Tensor nicht gefunden")

        # Ping-Pong: je 2 Device-Inputs, 2 Device-Outputs, 2 Host-Outputs, 2 Events
        self.SLOTS = 2
        self.d_in  = [cuda.mem_alloc(self.in_bytes)  for _ in range(self.SLOTS)]
        self.d_out = [cuda.mem_alloc(self.out_bytes) for _ in range(self.SLOTS)]
        self.h_out = [cuda.pagelocked_empty(self.output_shape, dtype=np.float32) for _ in range(self.SLOTS)]
        self.done  = [cuda.Event() for _ in range(self.SLOTS)]
        self.slot  = 0  # aktueller Slot zum Befüllen

        # für saubere Zuordnung: zu jedem Slot merken wir das passende Frame (fürs Zeichnen)
        self.slot_frame = [None]*self.SLOTS
        self.slot_shape = [None]*self.SLOTS

    def preprocess(self, frame_bgr):
        # falls du spiegeln willst: frame_bgr = cv2.flip(frame_bgr, 1)
        img = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        img = cv2.resize(img, (INPUT_SIZE, INPUT_SIZE))
        img = img.astype(np.float32) / 255.0
        img = np.transpose(img, (2, 0, 1))  # HWC->CHW
        img = np.expand_dims(img, axis=0)   # NCHW
        return np.ascontiguousarray(img)

    # Startet asynchron die Inferenz für das übergebene Bild im freien Slot.
    # Gibt den Slot-Index zurück, der gerade gestartet wurde.
    def enqueue(self, img_np, frame_for_slot):
        i = self.slot  # Slot auswählen
        # Input/Output-Adressen für diesen Slot setzen
        self.context.set_tensor_address(self.input_name,  int(self.d_in[i]))
        self.context.set_tensor_address(self.output_name, int(self.d_out[i]))

        # H2D Input
        cuda.memcpy_htod_async(self.d_in[i], img_np, self.stream)
        # Inferenz starten
        try:
            self.context.execute_async_v3(stream_handle=self.stream.handle)
        except AttributeError:
            # Fallback für ältere TRT-Versionen (falls nötig)
            self.context.execute_async_v2([int(self.d_in[i]), int(self.d_out[i])],
                                          stream_handle=self.stream.handle)
        # D2H Output in host-seitigen Puffer dieses Slots
        cuda.memcpy_dtoh_async(self.h_out[i], self.d_out[i], self.stream)
        # Fertigstellungs-Event aufzeichnen (kein stream.synchronize!)
        self.done[i].record(self.stream)

        # Frame/Shape für diesen Slot merken (für spätere Dekodierung/Zeichnung)
        self.slot_frame[i] = frame_for_slot
        self.slot_shape[i] = frame_for_slot.shape

        # Slot umschalten (Ping-Pong)
        self.slot = (self.slot + 1) % self.SLOTS
        return i  # gestarteter Slot

    # Holt das Ergebnis eines Slots ab (wartet nur auf dessen Event).
    def fetch(self, i):
        self.done[i].synchronize()  # wartet NUR auf Slot i
        out = np.asarray(self.h_out[i])
        frame = self.slot_frame[i]
        shape = self.slot_shape[i]
        return out, frame, shape

    def decode_yolo11_output(self, output, orig_shape, conf_thresh=0.5):
        out = output[0].T  # (8400, 68)
        boxes = out[:, 0:4]
        scores = out[:, 4]
        kpts   = out[:, 5:].reshape(-1, 21, 3)

        mask = scores > conf_thresh
        boxes, scores, kpts = boxes[mask], scores[mask], kpts[mask]
        if len(boxes) == 0:
            return np.zeros((0,4)), np.zeros((0,)), np.zeros((0,21,3))

        boxes_xyxy = np.zeros_like(boxes)
        boxes_xyxy[:, 0] = boxes[:, 0] - boxes[:, 2] / 2
        boxes_xyxy[:, 1] = boxes[:, 1] - boxes[:, 3] / 2
        boxes_xyxy[:, 2] = boxes[:, 0] + boxes[:, 2] / 2
        boxes_xyxy[:, 3] = boxes[:, 1] + boxes[:, 3] / 2

        H, W = orig_shape[:2]
        boxes_xyxy[:, [0, 2]] *= W / 640
        boxes_xyxy[:, [1, 3]] *= H / 640
        kpts[:, :, 0] *= W / 640
        kpts[:, :, 1] *= H / 640

        return boxes_xyxy, scores, kpts

    def draw_pose(self, frame, boxes, kpts, scores, fps=None):
        for i, box in enumerate(boxes):
            x1, y1, x2, y2 = box.astype(int)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)
            cv2.putText(frame, f"{scores[i]:.2f}", (x1, y1-5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 1)
            for (x, y, c) in kpts[i]:
                if c > 0.3:
                    cv2.circle(frame, (int(x), int(y)), 2, (255,0,0), -1)
        if fps is not None:
            cv2.putText(frame, f"FPS: {fps:.1f}", (10,30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
        return frame

# ====== Hauptprogramm mit echtem Overlap ======
def WebCamDemo():
    # Kamera-Thread starten
    cap_thread = CaptureThread(CAM_ID, CAM_W, CAM_H, CAM_FPS)
    cap_thread.start()

    # TensorRT vorbereiten
    hp = HandPose()
    win = "YOLO11n Pose (Ping-Pong Overlap)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)

    # Prime: 1. Frame holen und erste Inferenz starten (ohne auf Ergebnis zu warten)
    ok, frame = cap_thread.get(timeout=1.0)
    if not ok or frame is None:
        raise RuntimeError("Kein Kameraframe erhalten (Prime)")
    #img = hp.preprocess(frame)
    #prev_slot = hp.enqueue(img, frame)  # startet GPU für Slot 0
    prev_slot = hp.enqueue(hp.preprocess(frame), frame)  # Prime
    latest_frame = frame  # Fallback, falls gerade kein neues Frame anliegt
    t_last = time.time()

    try:    
        while True:
            # 1) Nicht-blockierend das aktuellste Frame holen (Queue leeren, nur das neueste behalten)
            got_any = False
            while True:
                ok, f = cap_thread.get(timeout=0.0)   # ← NICHT blockieren
                if not ok:
                    break
                latest_frame = f
                got_any = True
            frame = latest_frame

            # 2) Sofort neue Inferenz für N+1 starten (anderer Slot)
            img = hp.preprocess(frame)
            curr_slot = hp.enqueue(img, frame)

            # 3) Ergebnis von N (prev_slot) abholen (wartet NUR auf fertiges Event dieses Slots)
            output, prev_frame, prev_shape = hp.fetch(prev_slot)
            prev_slot = curr_slot

            # 4) Postproc + Anzeige
            boxes, scores, kpts = hp.decode_yolo11_output(output, prev_shape, conf_thresh=CONF_THRESH)
            fps = 1.0 / (time.time() - t_last)
            t_last = time.time()
            disp = hp.draw_pose(prev_frame.copy(), boxes, kpts, scores, fps=fps)
            cv2.imshow(win, disp)

            if cv2.waitKey(1) & 0xFF == 27:
                break
    
            import ExecTime
            ExecTime.Print()
    finally:
        cap_thread.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    WebCamDemo()

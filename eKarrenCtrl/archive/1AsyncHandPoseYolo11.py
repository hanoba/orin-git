#!/usr/bin/env python3
# Async YOLO11n Pose (TensorRT) – echte GPU/CPU-Parallelität mit Thread + Double Buffer
import cv2
import numpy as np
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit  # initialisiert Kontext im Main-Thread
import time
import tkinter as tk
import threading
import queue
from collections import deque

# === Configuration ===
ENGINE_PATH = "/home/harald/YOLO11n-pose-hands/runs/pose/train/weights/best.engine"
INPUT_SIZE = 640
CONF_THRESH = 0.4
CAM_ID = 2
RESULT_Q_MAX = 3      # Ergebnis-Puffer
FRAME_Q_MAX = 2       # Frame-Puffer (klein halten für niedrige Latenz)
DRAW_FPS = True

# ========== Hilfen ==========
def ema(prev, new, alpha=0.1):
    return (1 - alpha) * prev + alpha * new if prev is not None else new

def GetScreenSize():
    root = tk.Tk()
    width = root.winfo_screenwidth()
    height = root.winfo_screenheight()
    root.destroy()
    print(f"Screen size: {width}x{height}")

# ========== HandPose (TRT + Pre/Post) ==========
class HandPose:
    def __init__(self):
        TRT_LOGGER = trt.Logger(trt.Logger.INFO)
        with open(ENGINE_PATH, "rb") as f, trt.Runtime(TRT_LOGGER) as runtime:
            self.engine = runtime.deserialize_cuda_engine(f.read())
        self.context = self.engine.create_execution_context()

        # I/O-Infos
        self.input_name = None
        self.output_name = None
        self.output_shape = None
        elt_size = np.float32().nbytes

        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            mode = self.engine.get_tensor_mode(name)
            shape = tuple(self.context.get_tensor_shape(name))
            print(f"[INFO] Tensor {name}: {shape} ({'IN' if mode == trt.TensorIOMode.INPUT else 'OUT'})")
            if mode == trt.TensorIOMode.INPUT:
                self.input_name = name
                self.in_bytes = int(np.prod(shape)) * elt_size
            else:
                self.output_name = name
                self.output_shape = shape
                self.out_bytes = int(np.prod(shape)) * elt_size

        if self.input_name is None or self.output_name is None:
            raise RuntimeError("Could not find input/output tensors.")

        # Zwei GPU-Slots für echtes Double Buffering
        self.slots = []
        for _ in range(2):
            d_in = cuda.mem_alloc(self.in_bytes)
            d_out = cuda.mem_alloc(self.out_bytes)
            # Pinned Host-Ausgabe (schnelle D2H)
            h_out = cuda.pagelocked_empty(self.output_shape, dtype=np.float32)
            self.slots.append({"d_in": d_in, "d_out": d_out, "h_out": h_out})

    # --- Preprocess ---
    def preprocess(self, frame_bgr):
        img = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        img = cv2.resize(img, (INPUT_SIZE, INPUT_SIZE))
        img = img.astype(np.float32) / 255.0
        img = np.transpose(img, (2, 0, 1))  # HWC->CHW
        img = np.expand_dims(img, axis=0)   # NCHW
        return np.ascontiguousarray(img)

    # --- Postprocess/Decode ---
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

    def draw_pose(self, frame, boxes, kpts, scores, fps_cpu=None, fps_gpu=None):
        for i, box in enumerate(boxes):
            x1, y1, x2, y2 = box.astype(int)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f"{scores[i]:.2f}", (x1, y1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            for (x, y, c) in kpts[i]:
                if c > 0.3:
                    cv2.circle(frame, (int(x), int(y)), 2, (255, 0, 0), -1)
        if DRAW_FPS:
            y = 24
            if fps_cpu is not None:
                cv2.putText(frame, f"FPS CPU: {fps_cpu:.1f}", (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2); y+=22
            if fps_gpu is not None:
                cv2.putText(frame, f"FPS GPU: {fps_gpu:.1f}", (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2); y+=22
        return frame

# ========== GPU-Worker-Thread ==========
class GPUWorker(threading.Thread):
    """
    Zieht (frame_bgr, preprocessed, orig_shape) aus in_q,
    führt H2D + Inferenz + D2H im eigenen Stream aus,
    legt (boxes, scores, kpts, gpu_ms) in out_q.
    """
    def __init__(self, hp: HandPose, in_q: queue.Queue, out_q: queue.Queue):
        super().__init__(daemon=True)
        self.hp = hp
        self.in_q = in_q
        self.out_q = out_q
        self.running = True

    def run(self):
        # CUDA-Kontext im Thread aktivieren + eigener Stream
        ctx = pycuda.autoinit.context
        ctx.push()
        try:
            stream = cuda.Stream()
            slot_idx = 0
            start_evt = cuda.Event()
            end_evt = cuda.Event()

            while self.running:
                try:
                    item = self.in_q.get(timeout=0.1)
                except queue.Empty:
                    continue

                frame_bgr, img_chw, orig_shape = item
                slot = self.hp.slots[slot_idx]
                slot_idx ^= 1  # 0/1 togglen

                # Tensor-Adressen auf diesen Slot setzen
                self.hp.context.set_tensor_address(self.hp.input_name, int(slot["d_in"]))
                self.hp.context.set_tensor_address(self.hp.output_name, int(slot["d_out"]))

                # Asynchron: H2D → Inferenz → D2H (in einem Stream)
                cuda.memcpy_htod_async(slot["d_in"], img_chw, stream)
                start_evt.record(stream)
                # TRT v10+: v3-API; Fallback auf v2
                try:
                    self.hp.context.execute_async_v3(stream_handle=stream.handle)
                except AttributeError:
                    self.hp.context.execute_async_v2(
                        [int(slot["d_in"]), int(slot["d_out"])], stream_handle=stream.handle
                    )
                end_evt.record(stream)
                cuda.memcpy_dtoh_async(slot["h_out"], slot["d_out"], stream)
                stream.synchronize()  # nur hier synchronisieren (Thread blockiert, nicht GUI)

                gpu_ms = start_evt.time_till(end_evt)  # reine GPU-Zeit
                #print(f"GPU infer: {gpu_ms:.1f} ms")
                out_np = np.asarray(slot["h_out"])
                boxes, scores, kpts = self.hp.decode_yolo11_output(out_np, orig_shape, conf_thresh=CONF_THRESH)

                # non-blocking put (drop oldest if full)
                try:
                    self.out_q.put_nowait((boxes, scores, kpts, gpu_ms))
                except queue.Full:
                    try:
                        _ = self.out_q.get_nowait()
                    except queue.Empty:
                        pass
                    self.out_q.put_nowait((boxes, scores, kpts, gpu_ms))

                self.in_q.task_done()
        finally:
            ctx.pop()

    def stop(self):
        self.running = False

# ========== Haupt-Demo mit GUI ==========
def WebCamDemo():
    hp = HandPose()

    #cap = cv2.VideoCapture(CAM_ID)
    #if not cap.isOpened():
    #    raise RuntimeError(f"Cannot open camera {CAM_ID}")

    cap = cv2.VideoCapture(CAM_ID, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # <-- entscheidend!



    GetScreenSize()
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    print(f"Camera {cap.get(cv2.CAP_PROP_FRAME_WIDTH)}x{cap.get(cv2.CAP_PROP_FRAME_HEIGHT)}")

    win = "YOLO11n Pose (TensorRT Async)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.setWindowProperty(win, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    # Queues
    in_q  = queue.Queue(maxsize=FRAME_Q_MAX)
    out_q = queue.Queue(maxsize=RESULT_Q_MAX)

    worker = GPUWorker(hp, in_q, out_q)
    worker.start()

    last_boxes = last_scores = last_kpts = None
    fps_cpu = None
    fps_gpu = None

    try:
        while True:
            #t = time.time()
            ret, frame = cap.read()
            #dt = time.time() - t
            #print(f"Camera read(): {dt*1000:.1f} ms")
            if not ret:
                if cv2.waitKey(1) & 0xFF == 27:
                    break
                continue

            frame = cv2.flip(frame, 1)
            img = hp.preprocess(frame)

            # Input-Queue füllen (wenn voll -> ältestes droppen = niedrige Latenz)
            try:
                in_q.put_nowait((frame, img, frame.shape))
            except queue.Full:
                try:
                    _ = in_q.get_nowait()
                except queue.Empty:
                    pass
                in_q.put_nowait((frame, img, frame.shape))

            # Ergebnisse holen (so viele wie vorhanden)
            got = False
            while True:
                try:
                    boxes, scores, kpts, gpu_ms = out_q.get_nowait()
                    last_boxes, last_scores, last_kpts = boxes, scores, kpts
                    fps_gpu = ema(fps_gpu, 1000.0 / max(gpu_ms, 1e-3))
                    out_q.task_done()
                    got = True
                except queue.Empty:
                    break

            # Anzeigen mit letztem verfügbaren Ergebnis
            disp = frame if last_boxes is None else hp.draw_pose(frame.copy(), last_boxes, last_kpts, last_scores, fps_cpu, fps_gpu)
            cv2.imshow(win, disp)

            import ExecTime
            ExecTime.Print()

            if cv2.waitKey(1) & 0xFF == 27:  # ESC
                break
    finally:
        worker.stop()
        worker.join(timeout=1.0)
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    WebCamDemo()

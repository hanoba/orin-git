#!/usr/bin/env python3
# trt_yolo11_pose.py – TensorRT inference for YOLO11n Pose (Hand Pose)

import cv2
import numpy as np
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit
import time
import tkinter as tk

# === Configuration ===
ENGINE_PATH = "/home/harald/YOLO11n-pose-hands/runs/pose/train/weights/best.engine"
INPUT_SIZE = 640
CONF_THRESH = 0.4
IOU_THRESH = 0.45
USE_CAM = True
CAM_ID = 2

class HandPose():
    def __init__(self):
        # === TensorRT Setup ===
        TRT_LOGGER = trt.Logger(trt.Logger.INFO)
        with open(ENGINE_PATH, "rb") as f, trt.Runtime(TRT_LOGGER) as runtime:
            engine = runtime.deserialize_cuda_engine(f.read())
        self.context = engine.create_execution_context()

        self.tensor_buffers = {}
        self.input_name = self.output_name = None

        for i in range(engine.num_io_tensors):
            name = engine.get_tensor_name(i)
            shape = tuple(self.context.get_tensor_shape(name))
            print(f"[INFO] Tensor {name}: {shape}")
            size = int(np.prod(shape)) * np.float32().nbytes
            buf = cuda.mem_alloc(size)
            self.tensor_buffers[name] = buf
            self.context.set_tensor_address(name, int(buf))

            mode = engine.get_tensor_mode(name)
            if mode == trt.TensorIOMode.INPUT:
                self.input_name = name
            else:
                self.output_name = name
                self.output_shape = shape

        if self.input_name is None or self.output_name is None:
            raise RuntimeError("Could not find input/output tensors.")
            


    # === Preprocess ===
    def preprocess(self, frame):
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = cv2.resize(img, (INPUT_SIZE, INPUT_SIZE))
        img = img.astype(np.float32) / 255.0
        img = np.transpose(img, (2, 0, 1))  # HWC -> CHW
        img = np.expand_dims(img, axis=0)   # NCHW
        return np.ascontiguousarray(img)

    # === Postprocess (decode boxes + keypoints) ===
    def xywh2xyxy(self, x):
        y = np.zeros_like(x)
        y[:, 0] = x[:, 0] - x[:, 2] / 2  # x1
        y[:, 1] = x[:, 1] - x[:, 3] / 2  # y1
        y[:, 2] = x[:, 0] + x[:, 2] / 2  # x2
        y[:, 3] = x[:, 1] + x[:, 3] / 2  # y2
        return y

    def nms(self, boxes, scores, iou_threshold):
        """Simple numpy NMS."""
        if len(boxes) == 0:
            return []
        boxes = boxes.astype(np.float32)
        scores = scores.astype(np.float32)
        x1, y1, x2, y2 = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
        areas = (x2 - x1) * (y2 - y1)
        order = scores.argsort()[::-1]
        keep = []
        while order.size > 0:
            i = order[0]
            keep.append(i)
            xx1 = np.maximum(x1[i], x1[order[1:]])
            yy1 = np.maximum(y1[i], y1[order[1:]])
            xx2 = np.minimum(x2[i], x2[order[1:]])
            yy2 = np.minimum(y2[i], y2[order[1:]])
            w = np.maximum(0.0, xx2 - xx1)
            h = np.maximum(0.0, yy2 - yy1)
            inter = w * h
            iou = inter / (areas[i] + areas[order[1:]] - inter + 1e-6)
            inds = np.where(iou <= iou_threshold)[0]
            order = order[inds + 1]
        return keep

    def decode_yolo11_output(self, output, orig_shape, conf_thresh=0.5):
        """
        Decodes YOLO11n pose output from TensorRT engine.
        - output: (1, 68, 8400)
        - orig_shape: (H, W, C)
        Returns: boxes, scores, keypoints
        """
        output = output[0]         # shape (68, 8400)
        output = output.T           # (8400, 68)

        boxes  = output[:, 0:4]     # x, y, w, h (already in pixels)
        scores = output[:, 4]
        kpts   = output[:, 5:].reshape(-1, 21, 3)

        # Filter by confidence
        mask = scores > conf_thresh
        boxes, scores, kpts = boxes[mask], scores[mask], kpts[mask]

        if len(boxes) == 0:
            return np.zeros((0,4)), np.zeros((0,)), np.zeros((0,21,3))

        # Convert [x, y, w, h] → [x1, y1, x2, y2]
        boxes_xyxy = np.zeros_like(boxes)
        boxes_xyxy[:, 0] = boxes[:, 0] - boxes[:, 2] / 2
        boxes_xyxy[:, 1] = boxes[:, 1] - boxes[:, 3] / 2
        boxes_xyxy[:, 2] = boxes[:, 0] + boxes[:, 2] / 2
        boxes_xyxy[:, 3] = boxes[:, 1] + boxes[:, 3] / 2

        # Scale back to original image size
        H, W = orig_shape[:2]
        boxes_xyxy[:, [0, 2]] *= W / 640
        boxes_xyxy[:, [1, 3]] *= H / 640
        kpts[:, :, 0] *= W / 640
        kpts[:, :, 1] *= H / 640

        return boxes_xyxy, scores, kpts

    def draw_pose(self, frame, boxes, kpts, scores):
        for i, box in enumerate(boxes):
            x1, y1, x2, y2 = box.astype(int)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f"{scores[i]:.2f}", (x1, y1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

            for (x, y, c) in kpts[i]:
                if c > 0.3:
                    cv2.circle(frame, (int(x), int(y)), 2, (255, 0, 0), -1)
        return frame

    # === Inference ===
    def infer_image(self, frame):
        img = self.preprocess(frame)
        cuda.memcpy_htod(self.tensor_buffers[self.input_name], img)
        self.context.execute_v2([int(self.tensor_buffers[n]) for n in self.tensor_buffers])

        host_output = np.empty(self.output_shape, dtype=np.float32)
        cuda.memcpy_dtoh(host_output, self.tensor_buffers[self.output_name])
        #print("Output shape:", host_output.shape)
        #print("Output min/max:", host_output.min(), host_output.max())
        #print("First 5 values:", host_output.flatten()[:5])
        return host_output


    def center_crop(self, image, crop_size):
        """
        Crop the center region of an image.

        Args:
            image (np.ndarray): Input image (H x W x C)
            crop_size (int or tuple): Size of the crop (e.g. 416 or (416, 416))

        Returns:
            Cropped image (np.ndarray)
        """
        if isinstance(crop_size, int):
            crop_w = crop_h = crop_size
        else:
            crop_w, crop_h = crop_size

        h, w = image.shape[:2]

        # Ensure crop size does not exceed image size
        crop_w = min(crop_w, w)
        crop_h = min(crop_h, h)

        start_x = (w - crop_w) // 2
        start_y = (h - crop_h) // 2

        cropped = image[start_y:start_y + crop_h, start_x:start_x + crop_w]
        return cropped

    def ProcessFrame(self, frame):
        frame = self.center_crop(frame, INPUT_SIZE)
        frame = cv2.flip(frame, 1)
        output = self.infer_image(frame)
        boxes, scores, kpts = self.decode_yolo11_output(output, frame.shape)
        frame = self.draw_pose(frame, boxes, kpts, scores)
        # comput handsize
        if len(boxes) > 0:
            x1, y1, x2, y2 = boxes[0].astype(int)
            handSize = y2-y1
            handPos = ((x1+x2) // 2) - (INPUT_SIZE // 2)
        else: handSize = handPos = 0
        return frame, handSize, handPos


def GetScreenSize():
    root = tk.Tk()
    width = root.winfo_screenwidth()
    height = root.winfo_screenheight()
    root.destroy()
    print(f"Screen size: {width}x{height}")
    return


def WebCamDemo():
    import subprocess

    subprocess.call([
        "v4l2-ctl", "-d", "/dev/video2",
        "--set-ctrl=auto_exposure=1",
        "--set-ctrl=exposure_dynamic_framerate=0",
        "--set-ctrl=exposure_time_absolute=150"
    ])

    # === Main Loop ===
    handPose = HandPose()
    cap = cv2.VideoCapture(CAM_ID)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera {CAM_ID}")

    GetScreenSize()
    
    # Get frame width and height from camera properties
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    width  = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    print(f"Camera {width=} {height=}")

    win = "YOLO11n Pose (TensorRT)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.setWindowProperty(win, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        frame, handSize, handPos = handPose.ProcessFrame(frame)

        #cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        #cv2.setWindowProperty(win, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        cv2.imshow(win, frame)
        if cv2.waitKey(1) & 0xFF == 27:  # ESC
            break
        import ExecTime
        ExecTime.Print()

    cap.release()

def Demo(imageFileName):
    handPose = HandPose()
    frame = cv2.imread(imageFileName)
    frame = handPose.ProcessFrame(frame)
    cv2.imshow("Result", frame)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    imageFile = "/home/harald/yolo-hand-detection/images/zachary-nelson-98Elr-LIvD8-unsplash.jpg"
    #Demo(imageFile)
    WebCamDemo()
#!/usr/bin/env python
# coding: utf-8
# File: HandPoseLib.py

import os
import json
import cv2
import numpy as np
import torch
import torchvision.transforms as transforms
import PIL.Image

from trt_pose.coco import coco_category_to_topology
from trt_pose.draw_objects import DrawObjects
from trt_pose.parse_objects import ParseObjects
from .preprocessdata import preprocessdata
from .gesture_classifier import gesture_classifier
from pathlib import Path

# TensorRT / PyCUDA
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit

import tkinter as tk    # for screen size detection

class HandPose():
    def __init__(self):
        self.imageSize = 224
        self.printCnt = 0
        # --------------------------
        # Load hand pose topology
        # --------------------------
        with open(Path(__file__).parent / "preprocess" / "hand_pose.json", 'r') as f:
            self.hand_pose = json.load(f)

        self.topology = coco_category_to_topology(self.hand_pose)
        self.num_parts = len(self.hand_pose['keypoints'])
        self.num_links = len(self.hand_pose['skeleton'])

        self.parse_objects = ParseObjects(self.topology, cmap_threshold=0.15, link_threshold=0.15)
        self.draw_objects = DrawObjects(self.topology)
        self.preprocessdata = preprocessdata(self.topology, self.num_parts)
        #gesture_classifier = gesture_classifier()

        # --------------------------
        # Preprocessing (CPU)
        # --------------------------
        self.mean = torch.Tensor([0.485, 0.456, 0.406])
        self.std = torch.Tensor([0.229, 0.224, 0.225])

        # --------------------------
        # Load TensorRT engine
        # --------------------------
        ENGINE_FILE = '/home/harald/trt_pose_hand/model/hand_pose_resnet18_att_244_244.trt'
        TRT_LOGGER = trt.Logger(trt.Logger.INFO)

        if not os.path.exists(ENGINE_FILE):
            raise RuntimeError(f"TensorRT engine not found: {ENGINE_FILE}")

        with open(ENGINE_FILE, 'rb') as f, trt.Runtime(TRT_LOGGER) as runtime:
            engine = runtime.deserialize_cuda_engine(f.read())

        self.context = engine.create_execution_context()

        # --------------------------
        # Allocate buffers
        # --------------------------
        self.buffers = []
        self.binding_shapes = []
        self.binding_dtypes = []

        self.binding_names = [engine.get_tensor_name(i) for i in range(engine.num_io_tensors)]

        for name in self.binding_names:
            dtype = trt.nptype(engine.get_tensor_dtype(name))
            shape = engine.get_tensor_shape(name)
            size = np.prod(shape)
            device_mem = cuda.mem_alloc(int(size * dtype().nbytes))
            self.buffers.append(device_mem)
            self.binding_shapes.append(shape)
            self.binding_dtypes.append(dtype)

        self.stream = cuda.Stream()

        # --------------------------
        # OpenCV loop init
        # --------------------------
        self.cap = cv2.VideoCapture(2)
        if not self.cap.isOpened():
            raise RuntimeError("Cannot open camera")

        # get cam properties
        width  = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps    = self.cap.get(cv2.CAP_PROP_FPS)
        print(f"{height=}, {width=}, {fps=}")

        # Define the cropping coordinates
        # Format: image[y_start:y_end, x_start:x_end]
        self.x_start = (width - height) // 2
        self.y_start = 0
        self.x_end = self.x_start + height
        self.y_end = height

        root = tk.Tk()
        self.screen_w = root.winfo_screenwidth()
        self.screen_h = root.winfo_screenheight()
        root.destroy()
        print(f"{self.screen_w=}, {self.screen_h=}")

        self.win="Hand Pose"
        cv2.namedWindow(self.win, cv2.WINDOW_NORMAL)
        cv2.setWindowProperty(self.win, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    def __del__(self):
        self.cap.release()
        cv2.destroyAllWindows()

    def preprocess(self, image, input_dtype):
        """
        Convert BGR image to normalized FP16 numpy tensor for TensorRT input
        Shape: [1,3,H,W], dtype=np.float16
        """
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = PIL.Image.fromarray(image)
        tensor = transforms.functional.to_tensor(image)  # [C,H,W], float32
        tensor.sub_(self.mean[:, None, None]).div_(self.std[:, None, None])
        return np.ascontiguousarray(tensor[None, ...].numpy().astype(input_dtype))
        
    
    def draw_joints(self, image, joints):
        # count how many joints were not found in the current frame
        count = sum(1 for i in joints if i == [0,0])
        if count >= 5:      # 3
            return False, 0, 0

        radius=6
        thickness=1    
        for i in joints:
            cv2.circle(image, (i[0], i[1]), radius, (0,0,255), thickness)
        cv2.circle(image, (joints[0][0], joints[0][1]), radius, (255,0,0), thickness)
        
        #"skeleton": [[1, 5], [1, 9], [1, 13], [1, 17], [1, 21], 
        #            [2, 3], [3, 4], [4, 5], 
        #            [6, 7], [7, 8], [8, 9], 
        #            [10, 11], [11, 12], [12, 13], 
        #            [14, 15], [15, 16], [16, 17], 
        #            [18, 19], [19, 20], [20, 21]]}
        for i in self.hand_pose['skeleton']:
            if joints[i[0]-1][0]==0 or joints[i[1]-1][0]==0:
                continue
            cv2.line(
                image,
                (joints[i[0]-1][0], joints[i[0]-1][1]),
                (joints[i[1]-1][0], joints[i[1]-1][1]),
                (0,255,0), 2
            )
        x = joints[0][0] - (self.imageSize // 2)
        
        yZeigefinger  = abs(joints[0][1] - joints[8][1] )
        yMittelfinger = abs(joints[0][1] - joints[12][1])
        yRingfinger   = abs(joints[0][1] - joints[16][1])

        points = np.array([joints[8],
                           joints[12],
                           joints[16]])
        ref = np.array(joints[0])

        # axis=1 sagt NumPy, dass die Norm für jede Zeile berechnet wird.
        [yZeigefinger, yMittelfinger, yRingfinger] = np.linalg.norm(points - ref, axis=1)
        #print(dists)  # → [2.236..., 7.211..., 0.0]        
        
        if yZeigefinger > yMittelfinger:
            if yMittelfinger >= yRingfinger: handSize = yMittelfinger
            elif yRingfinger >= yZeigefinger: handSize = yZeigefinger
            else: handSize = yRingfinger
        else:
            if yZeigefinger >= yRingfinger: handSize = yZeigefinger
            elif yMittelfinger >= yRingfinger: handSize = yRingfinger
            else: handSize = yMittelfinger
        
        if handSize < 10:
            return False, 0, 0
        
        if self.printCnt == 10:
            print(f"{x=}, {handSize=:.0f}, {yZeigefinger=:.0f}, {yMittelfinger=:.0f}, {yRingfinger=:.0f}")
        
        return True, x, int(handSize)

    # --------------------------
    # Inference
    # --------------------------
    def infer_and_draw(self, frame):
        # Preprocess input
        input_dtype = self.binding_dtypes[0]
        input_data = self.preprocess(frame, input_dtype)  # [1,3,H,W], FP16
        assert input_data.shape == tuple(self.binding_shapes[0]), f"Input shape mismatch: {input_data.shape} vs {self.binding_shapes[0]}"
        assert input_data.dtype == self.binding_dtypes[0], f"Input dtype mismatch: {input_data.dtype} vs {self.binding_dtypes[0]}"

        # Copy input to GPU
        cuda.memcpy_htod_async(self.buffers[0], input_data, self.stream)

        # Assuming buffers is a list of pycuda device pointers
        for i, name in enumerate(self.binding_names):
            self.context.set_tensor_address(name, int(self.buffers[i]))
            
        # Execute inference
        self.context.execute_async_v3(stream_handle=self.stream.handle)
        
        # Allocate output arrays
        cmap  = np.empty(self.binding_shapes[1], dtype=self.binding_dtypes[1])
        paf   = np.empty(self.binding_shapes[2], dtype=self.binding_dtypes[2])
      
        # Copy outputs to CPU
        cuda.memcpy_dtoh_async(cmap, self.buffers[1], self.stream)
        cuda.memcpy_dtoh_async(paf, self.buffers[2], self.stream)
        self.stream.synchronize()

        # Convert to torch tensors
        cmap_t = torch.from_numpy(cmap)
        paf_t = torch.from_numpy(paf)

        # Parse hand joints and draw
        counts, objects, peaks = self.parse_objects(cmap_t, paf_t)
        joints = self.preprocessdata.joints_inference(frame, counts, objects, peaks)

        handFound, x, handSize = self.draw_joints(frame, joints)
        
        return frame, handFound, x, handSize

    # Process one frame
    def ProcessFrame(self):
        ret, frame = self.cap.read()
        if not ret:
            raise RuntimeError("Error reading camera")
            
        self.printCnt += 1
        
        frame_cropped = frame[self.y_start:self.y_end, self.x_start:self.x_end]
        frame_resized = cv2.resize(frame_cropped, (self.imageSize, self.imageSize))
        result, handFound, x, handSize = self.infer_and_draw(frame_resized)
        display = cv2.resize(result, (self.screen_h,self.screen_h))
        flip = cv2.flip(display, 1)

        if self.printCnt == 10:
            self.printCnt=0

        OSD = False
        if OSD:
            # Text Overlay 
            text = f"{x=} {handSize=}"

            font = cv2.FONT_HERSHEY_SIMPLEX
            color = (0, 255, 0)
            thickness = 2

            # Text zeichnen (oben links)
            cv2.putText(flip, text, (20, 40), font, 1.0, (255, 255, 255), 2, cv2.LINE_AA)

        cv2.imshow(self.win, flip)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            return True, 0, 0
        return handFound, x, handSize

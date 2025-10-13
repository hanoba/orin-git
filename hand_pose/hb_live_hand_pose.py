#!/usr/bin/env python
# coding: utf-8
# File: hb_live_hand_pose_trt.py

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
from preprocessdata import preprocessdata
from gesture_classifier import gesture_classifier

# TensorRT / PyCUDA
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit

import tkinter as tk    # for screen size detection


# --------------------------
# Load hand pose topology
# --------------------------
with open('preprocess/hand_pose.json', 'r') as f:
    hand_pose = json.load(f)

topology = coco_category_to_topology(hand_pose)
num_parts = len(hand_pose['keypoints'])
num_links = len(hand_pose['skeleton'])

parse_objects = ParseObjects(topology, cmap_threshold=0.15, link_threshold=0.15)
draw_objects = DrawObjects(topology)
preprocessdata = preprocessdata(topology, num_parts)
#gesture_classifier = gesture_classifier()

# --------------------------
# Preprocessing (CPU)
# --------------------------
mean = torch.Tensor([0.485, 0.456, 0.406])
std = torch.Tensor([0.229, 0.224, 0.225])

def preprocess(image, input_dtype):
    """
    Convert BGR image to normalized FP16 numpy tensor for TensorRT input
    Shape: [1,3,H,W], dtype=np.float16
    """
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image = PIL.Image.fromarray(image)
    tensor = transforms.functional.to_tensor(image)  # [C,H,W], float32
    tensor.sub_(mean[:, None, None]).div_(std[:, None, None])
    return np.ascontiguousarray(tensor[None, ...].numpy().astype(input_dtype))

printCnt = 0

def draw_joints(image, joints):
    global printCnt
    count = sum(1 for i in joints if i == [0,0])
    if count >= 5:  #3:
        return
    radius=6
    thickness=1    
    for i in joints:
        cv2.circle(image, (i[0], i[1]), radius, (0,0,255), thickness)
    cv2.circle(image, (joints[0][0], joints[0][1]), radius, (255,0,0), thickness)
    printCnt += 1
    if printCnt == 10:
        printCnt=0
        print(f"x={joints[0][0]}, y={joints[0][1]}")
    for i in hand_pose['skeleton']:
        if joints[i[0]-1][0]==0 or joints[i[1]-1][0]==0:
            continue
        cv2.line(
            image,
            (joints[i[0]-1][0], joints[i[0]-1][1]),
            (joints[i[1]-1][0], joints[i[1]-1][1]),
            (0,255,0), 2
        )

# --------------------------
# Load TensorRT engine
# --------------------------
ENGINE_FILE = '/home/harald/trt_pose_hand/model/hand_pose_resnet18_att_244_244.trt'
TRT_LOGGER = trt.Logger(trt.Logger.INFO)

if not os.path.exists(ENGINE_FILE):
    raise RuntimeError(f"TensorRT engine not found: {ENGINE_FILE}")

with open(ENGINE_FILE, 'rb') as f, trt.Runtime(TRT_LOGGER) as runtime:
    engine = runtime.deserialize_cuda_engine(f.read())

context = engine.create_execution_context()

# --------------------------
# Allocate buffers
# --------------------------
buffers = []
binding_shapes = []
binding_dtypes = []

binding_names = [engine.get_tensor_name(i) for i in range(engine.num_io_tensors)]

for name in binding_names:
    dtype = trt.nptype(engine.get_tensor_dtype(name))
    shape = engine.get_tensor_shape(name)
    size = np.prod(shape)
    device_mem = cuda.mem_alloc(int(size * dtype().nbytes))
    buffers.append(device_mem)
    binding_shapes.append(shape)
    binding_dtypes.append(dtype)

stream = cuda.Stream()

# --------------------------
# Inference
# --------------------------
def infer_and_draw(frame):
    # Preprocess input
    input_dtype = binding_dtypes[0]
    input_data = preprocess(frame, input_dtype)  # [1,3,H,W], FP16
    assert input_data.shape == tuple(binding_shapes[0]), f"Input shape mismatch: {input_data.shape} vs {binding_shapes[0]}"
    assert input_data.dtype == binding_dtypes[0], f"Input dtype mismatch: {input_data.dtype} vs {binding_dtypes[0]}"

    # Copy input to GPU
    cuda.memcpy_htod_async(buffers[0], input_data, stream)

    # Assuming buffers is a list of pycuda device pointers
    for i, name in enumerate(binding_names):
        context.set_tensor_address(name, int(buffers[i]))
        
    # Execute inference
    context.execute_async_v3(stream_handle=stream.handle)
    
    # Allocate output arrays
    cmap  = np.empty(binding_shapes[1], dtype=binding_dtypes[1])
    paf   = np.empty(binding_shapes[2], dtype=binding_dtypes[2])
  
    # Copy outputs to CPU
    cuda.memcpy_dtoh_async(cmap, buffers[1], stream)
    cuda.memcpy_dtoh_async(paf, buffers[2], stream)
    stream.synchronize()

    # Convert to torch tensors
    cmap_t = torch.from_numpy(cmap)
    paf_t = torch.from_numpy(paf)

    # Parse hand joints and draw
    counts, objects, peaks = parse_objects(cmap_t, paf_t)
    joints = preprocessdata.joints_inference(frame, counts, objects, peaks)

    draw_joints(frame, joints)
    
    return frame

# --------------------------
# OpenCV loop
# --------------------------
cap = cv2.VideoCapture(2)
if not cap.isOpened():
    raise RuntimeError("Cannot open camera")

# get cam properties
width  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps    = cap.get(cv2.CAP_PROP_FPS)
print(f"{height=}, {width=}, {fps=}")

# Define the cropping coordinates
# Format: image[y_start:y_end, x_start:x_end]
x_start, y_start = (width-height)//2, 0
x_end, y_end = x_start+height, height

root = tk.Tk()
screen_w = root.winfo_screenwidth()
screen_h = root.winfo_screenheight()
root.destroy()
print(f"{screen_w=}, {screen_h=}")

win="Hand Pose"
cv2.namedWindow(win, cv2.WINDOW_NORMAL)
cv2.setWindowProperty(win, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            break
            
        frame_cropped = frame[y_start:y_end, x_start:x_end]
        frame_resized = cv2.resize(frame_cropped, (224, 224))
        #frame_resized = cv2.resize(frame, (244, 244)) # HB
        result = infer_and_draw(frame_resized)
        display = cv2.resize(result, (screen_h,screen_h))

        cv2.imshow(win, display)
        #cv2.imshow(win, result)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
except KeyboardInterrupt:
    print("\nScript terminated")
    cap.release()
    cv2.destroyAllWindows()

#!/usr/bin/env python
# coding: utf-8
# File: ds9_live_hand_pose.py
#!/usr/bin/env python
# coding: utf-8
# File: ds8_live_hand_pose_trt.py

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

from GestureClassifier import GestureClassifier

gesture_classifier = GestureClassifier()

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


def draw_joints(image, joints):
    count = sum(1 for i in joints if i == [0,0])
    if count >= 3:
        return
    for i in joints:
        cv2.circle(image, (i[0], i[1]), 2, (0,0,255), 1)
    cv2.circle(image, (joints[0][0], joints[0][1]), 2, (255,0,255), 1)
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
ENGINE_FILE = 'model/hand_pose_resnet18_att_244_244.trt'
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

    # Run inference
    #context.execute_async_v2(bindings=[int(b) for b in buffers], stream_handle=stream.handle)
    # Assuming buffers is a list of pycuda device pointers
    for i, name in enumerate(binding_names):
        context.set_tensor_address(name, int(buffers[i]))
        
    # Execute inference
    context.execute_async_v3(stream_handle=stream.handle)
    
    ##cuda.memcpy_dtoh_async(host_array, buffers[output_index], stream)
    ##stream.synchronize()

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

    gesture = gesture_classifier.classify(joints)
    cv2.putText(frame, f"{gesture}", (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    draw_joints(frame, joints)
    
    return frame

# --------------------------
# OpenCV loop
# --------------------------
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    raise RuntimeError("Cannot open camera")

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame_resized = cv2.resize(frame, (224, 224)) # HB
        result = infer_and_draw(frame_resized)

        cv2.imshow("Hand Pose", result)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    cap.release()
    cv2.destroyAllWindows()

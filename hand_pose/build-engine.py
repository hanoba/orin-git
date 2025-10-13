#!/usr/bin/env python
# coding: utf-8

import os
import json
import cv2
import numpy as np
import torch
import torchvision.transforms as transforms
import PIL.Image

import trt_pose.coco
from trt_pose.draw_objects import DrawObjects
from trt_pose.parse_objects import ParseObjects
from preprocessdata import preprocessdata
from gesture_classifier import gesture_classifier

import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit

# --------------------------
# Load hand pose topology
# --------------------------
with open('preprocess/hand_pose.json', 'r') as f:
    hand_pose = json.load(f)

topology = trt_pose.coco.coco_category_to_topology(hand_pose)
num_parts = len(hand_pose['keypoints'])
num_links = len(hand_pose['skeleton'])

parse_objects = ParseObjects(topology, cmap_threshold=0.15, link_threshold=0.15)
draw_objects = DrawObjects(topology)

preprocessdata = preprocessdata(topology, num_parts)
gesture_classifier = gesture_classifier()

mean = torch.Tensor([0.485, 0.456, 0.406]).cuda()
std = torch.Tensor([0.229, 0.224, 0.225]).cuda()


# --------------------------
# TensorRT engine
# --------------------------
ONNX_MODEL = 'model/hand_pose_resnet18_att_244_244.onnx'
ENGINE_FILE = 'model/hand_pose_resnet18_att_244_244.trt'
PTH_MODEL = 'model/hand_pose_resnet18_att_244_244.pth'
WIDTH, HEIGHT = 244, 244

TRT_LOGGER = trt.Logger(trt.Logger.INFO)

print("TensorRT engine not found. Creating engine...")

# Load model on CPU
import trt_pose.models
model = trt_pose.models.resnet18_baseline_att(num_parts, 2*num_links).cpu().eval()
model.load_state_dict(torch.load(PTH_MODEL, map_location='cpu'))

# Dummy input for ONNX
dummy_input = torch.zeros((1, 3, WIDTH, HEIGHT))

# Export ONNX
torch.onnx.export(
    model, dummy_input, ONNX_MODEL,
    opset_version=11,
    input_names=["input"],
    output_names=["cmap","paf"],
    dynamic_axes=None
)
print("ONNX model exported:", ONNX_MODEL)

# Build TensorRT engine
# TRT_LOGGER = trt.Logger(trt.Logger.INFO)
builder = trt.Builder(TRT_LOGGER)
network_flags = (1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
network = builder.create_network(network_flags)

parser = trt.OnnxParser(network, TRT_LOGGER)

with open(ONNX_MODEL, 'rb') as f:
    if not parser.parse(f.read()):
        print("ERROR: Failed to parse ONNX model")
        for error in range(parser.num_errors):
            print(parser.get_error(error))
        raise RuntimeError("Failed to parse ONNX model")

# Mark all network outputs
for i in range(network.num_outputs):
    network.get_output(i).name = parser.get_output(i).name
    network.mark_output(network.get_output(i))

# Builder config
config = builder.create_builder_config()
config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, 1 << 27)  # 128 MB
config.set_flag(trt.BuilderFlag.FP16)

# Build serialized engine
serialized_engine = builder.build_serialized_network(network, config)
if serialized_engine is None:
    raise RuntimeError("Failed to build TensorRT engine")    
    
with open(ENGINE_FILE, 'wb') as f:
    f.write(serialized_engine)
print("TensorRT engine created:", ENGINE_FILE)


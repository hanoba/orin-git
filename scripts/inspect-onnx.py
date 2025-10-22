#!/usr/bin/env python3

import onnx
import sys

if len(sys.argv) != 2:
    print("usage: inspect_onxx <onnx-file>")
    exit()
    
# Load the model
model_path = sys.argv[1]

# model_path = "/home/harald/pytorch-YOLOv4/cross_hands.onnx"
#"model/hand_pose_resnet18_att_244_244.onnx"
model = onnx.load(model_path)

# Check the model is well-formed
onnx.checker.check_model(model)
print("ONNX model is valid.")

# Print basic info
print("Model IR version:", model.ir_version)
print("Producer name:", model.producer_name)
print("Producer version:", model.producer_version)
print("Domain:", model.domain)
print("Model version:", model.model_version)
print("Opset version:", model.opset_import[0].version)

# Print inputs
print("\n--- Model Inputs ---")
for input_tensor in model.graph.input:
    name = input_tensor.name
    shape = [dim.dim_value for dim in input_tensor.type.tensor_type.shape.dim]
    dtype = input_tensor.type.tensor_type.elem_type
    print(f"Name: {name}, Shape: {shape}, Type: {dtype}")

# Print outputs
print("\n--- Model Outputs ---")
for output_tensor in model.graph.output:
    name = output_tensor.name
    shape = [dim.dim_value for dim in output_tensor.type.tensor_type.shape.dim]
    dtype = output_tensor.type.tensor_type.elem_type
    print(f"Name: {name}, Shape: {shape}, Type: {dtype}")

# Print number of layers (nodes)
print("\nNumber of nodes/layers in graph:", len(model.graph.node))

print("\n--- Layer summary ---")
for node in model.graph.node:
    print(f"Op type: {node.op_type}, Name: {node.name}, Inputs: {node.input}, Outputs: {node.output}")

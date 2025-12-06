# Engine File Creation

# download pytorch model
wget https://huggingface.co/Ultralytics/YOLO11/resolve/main/yolo11n-pose.pt

# export onnx model
yolo export model=yolo11n-pose.pt format=onnx opset=12 dynamic=False simplify=True

# create engine file for Orin-NX from onnx model
/usr/src/tensorrt/bin/trtexec --onnx=yolo11n-pose.onnx --saveEngine=yolo11n-pose.engine --fp16

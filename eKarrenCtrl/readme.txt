git clone https://github.com/chrismuntean/YOLO11n-pose-hands.git

# run webcam demo (on CPU)
cd ~/YOLO11n-pose-hands/
python webcam-test.py

# export onnx model
cd ~/YOLO11n-pose-hands/train/weights/
yolo export model=best.pt format=onnx opset=12 dynamic=False simplify=True

# create engine file for Orin-Nx from onnx model
/usr/src/tensorrt/bin/trtexec --onnx=best.onnx --saveEngine=best.engine --fp16

# run inference on Orin-NX
cd ~/YOLO11n-pose-hands/
python trt.py

#!/bin/bash

echo "===================================="
echo "        Jetson System Check         "
echo "===================================="

# Board & L4T
echo "Board Info:"
head -n 1 /etc/nv_tegra_release
echo "Ubuntu Version:"
lsb_release -d | cut -f2
echo

# JetPack (indirekt)
L4T=$(head -n 1 /etc/nv_tegra_release | awk '{print $2}')
echo "Detected L4T Version: $L4T"
echo "Corresponding JetPack Version: JetPack 6.2.1 (for R36.4.x)"
echo

# CUDA
echo "CUDA Version:"
if command -v nvcc >/dev/null 2>&1; then
    nvcc --version | grep release
else
    echo "CUDA not found"
fi
echo

# cuDNN
echo "cuDNN Version:"
dpkg -l | grep libcudnn || echo "cuDNN not installed"
echo

# TensorRT
echo "TensorRT Version:"
dpkg -l | grep nvinfer || echo "TensorRT not installed"
echo

# VPI
echo "VPI Version:"
dpkg -l | grep vpi || echo "VPI not installed"
echo

# OpenCV
echo "OpenCV Version & CUDA support:"
pkg-config --modversion opencv4 2>/dev/null || echo "OpenCV not installed"
if python3 -c "import cv2; print(cv2.getBuildInformation().find('CUDA') >=0)" 2>/dev/null; then
    echo "OpenCV built with CUDA"
else
    echo "OpenCV built WITHOUT CUDA"
fi
echo

# DeepStream
echo "DeepStream Version:"
dpkg -l | grep deepstream || echo "DeepStream not installed"
echo

# GPU Status
#echo "GPU Status (via tegrastats, first 10s):"
#echo "(Press Ctrl+C to stop if needed)"
#sudo tegrastats --interval 2
#echo

echo "===================================="
echo "        End of System Check          "
echo "===================================="

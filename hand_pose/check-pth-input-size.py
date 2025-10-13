#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# File: check_pth_input_size.py
#
# Detect expected input size and output shape of a trt_pose .pth model

import torch
import sys
from trt_pose.models import resnet18_baseline_att

# ---------------------------------
# Configuration
# ---------------------------------
MODEL_PATH = "model/hand_pose_resnet18_att_244_244.pth"
NUM_PARTS = 21       # Keypoints (21 for hand)
NUM_LINKS = 20       # Connections (2 * 21)
INPUT_SIZES = [224, 240, 244, 256, 320]  # Candidates to test

# ---------------------------------
# Load model
# ---------------------------------
print(f"\nLoading model: {MODEL_PATH}")
model = resnet18_baseline_att(NUM_PARTS, 2 * NUM_LINKS)
model.load_state_dict(torch.load(MODEL_PATH, map_location='cpu'))
model.eval()

print("✅ Model loaded successfully.\n")

# ---------------------------------
# Test different input sizes
# ---------------------------------
for size in INPUT_SIZES:
    x = torch.zeros((1, 3, size, size))
    try:
        with torch.no_grad():
            cmap, paf = model(x)
        print(f"✅ Works for input size: {size}x{size}")
        print(f"   → cmap shape: {tuple(cmap.shape)}")
        print(f"   → paf shape:  {tuple(paf.shape)}")
        stride = size // cmap.shape[2]
        print(f"   → effective stride: {stride}\n")
    except Exception as e:
        print(f"❌ Fails for {size}x{size}: {e}\n")

print("🔍 Test complete.")

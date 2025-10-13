#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
inspect_pth.py — Inspect contents of a PyTorch .pth or .pt model
Usage:
    python inspect_pth.py model/hand_pose_resnet18_att_244_244.pth
"""

import torch
import sys
from collections import OrderedDict

def human_format(num):
    """Format large numbers nicely (e.g. 1.2M)."""
    for unit in ['', 'K', 'M', 'B']:
        if abs(num) < 1000.0:
            return f"{num:3.1f}{unit}"
        num /= 1000.0
    return f"{num:.1f}T"

def print_header(title):
    print("\n" + "="*len(title))
    print(title)
    print("="*len(title))

def main():
    if len(sys.argv) != 2:
        print("Usage: python inspect_pth.py <model.pth>")
        sys.exit(1)

    path = sys.argv[1]
    print_header(f"Inspecting: {path}")

    try:
        checkpoint = torch.load(path, map_location="cpu")
    except Exception as e:
        print(f"❌ Failed to load model: {e}")
        sys.exit(1)

    print(f"✅ Loaded successfully. Type: {type(checkpoint)}")

    # If it's a simple state_dict
    if isinstance(checkpoint, OrderedDict) or all(isinstance(k, str) for k in checkpoint.keys()):
        print_header("Model State Dictionary (weights only)")
        total_params = 0
        for k, v in checkpoint.items():
            print(f"{k:<45} {tuple(v.shape)}")
            total_params += v.numel()
        print(f"\nTotal parameters: {human_format(total_params)} ({total_params:,})")

    # If it's a full checkpoint
    elif isinstance(checkpoint, dict):
        print_header("Checkpoint Metadata")
        for key in checkpoint.keys():
            if key not in ['model_state_dict', 'optimizer_state_dict']:
                print(f"{key}: {checkpoint[key]}")

        if 'model_state_dict' in checkpoint:
            print_header("Model Weights in Checkpoint")
            total_params = 0
            for k, v in checkpoint['model_state_dict'].items():
                print(f"{k:<45} {tuple(v.shape)}")
                total_params += v.numel()
            print(f"\nTotal parameters: {human_format(total_params)} ({total_params:,})")

    else:
        print("⚠️ Unrecognized file structure — not a typical PyTorch checkpoint or state_dict.")

if __name__ == "__main__":
    main()

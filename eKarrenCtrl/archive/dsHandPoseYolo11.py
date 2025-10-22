#!/usr/bin/env python3
# DeepStream 7.1 — USB Camera (MJPEG) → YOLO11 Pose (TensorRT) → GPU OSD → Screen
# Tested on Jetson Orin NX with DeepStream 7.1.0

import os
import gi, sys, ctypes, time
gi.require_version('Gst', '1.0')
gi.require_version('GObject', '2.0')
from gi.repository import Gst, GObject
import numpy as np
import pyds

# === CONFIG ===
CAM_DEV      = "/dev/video2"
WIDTH        = 640
HEIGHT       = 480
FPS          = 30
CONF_THRESH  = 0.4
IOU_THRESH   = 0.45
ENGINE_PATH  = "runs/pose/train/weights/best.engine"
CONFIG_PATH  = "ds_config_infer_primary_yolo11_pose.txt"

# === Helpers ===
def bus_call(bus, message, loop):
    t = message.type
    if t == Gst.MessageType.EOS:
        print("End of stream")
        loop.quit()
    elif t == Gst.MessageType.ERROR:
        err, dbg = message.parse_error()
        print("Error:", err, dbg)
        loop.quit()
    return True



# ---- Tensor → NumPy ----
import numpy as np
import ctypes
import pyds

def get_tensor_as_numpy(user_meta_data):
    """
    Liest NvDsInferTensorMeta aus DeepStream und gibt ein NumPy-Array (1,68,8400) zurück.
    Kompatibel mit älteren DeepStream-Versionen (ohne get_nvds_buf_surface_gpu_to_cpu).
    """
    tensor_meta = pyds.NvDsInferTensorMeta.cast(user_meta_data)
    num_layers = tensor_meta.num_output_layers
    all_layers = []

    for i in range(num_layers):
        layer = pyds.get_nvds_LayerInfo(tensor_meta, i)
        dims = layer.inferDims
        n_elements = dims.numElements
        shape = tuple(dims.d[0:dims.numDims])

        if len(shape) == 2 and shape[0] == 68:
            shape = (1, 68, shape[1])
        elif len(shape) == 3 and shape[0] == 1 and shape[1] in [68,84]:
            pass
        else:
            print(f"⚠️  Layer {i} hat ungewöhnliche Shape: {shape}")

        # Prüfen, ob neue pyds-Funktion existiert
        if hasattr(pyds, "get_nvds_buf_surface_gpu_to_cpu"):
            np_arr = pyds.get_nvds_buf_surface_gpu_to_cpu(layer.buffer, n_elements)
        else:
            # 🧩 Manuelle CPU-Kopie (ältere DeepStream-Version)
            ptr_int = pyds.get_ptr(layer.buffer)
            float_ptr = ctypes.cast(ptr_int, ctypes.POINTER(ctypes.c_float))
            np_arr = np.ctypeslib.as_array(float_ptr, shape=(n_elements,))

        np_arr = np_arr.reshape(shape)
        all_layers.append(np_arr.astype(np.float32))

        print(f"🧠 Layer{i}: {layer.layerName}, shape={shape}, min={np_arr.min():.2f}, max={np_arr.max():.2f}")

    if len(all_layers) == 1:
        output = all_layers[0]
    else:
        output = np.concatenate(all_layers, axis=1)

    if output.shape[1] != 68 and output.shape[-1] == 68:
        output = np.transpose(output, (0, 2, 1))

    print(f"✅ Tensor ready: shape={output.shape}, dtype={output.dtype}")
    return output


# ---- Decoder von trt_yolo11_pose.py ----
def decode_yolo11_output(output, orig_shape, conf_thresh=0.4):
    """
    Decodes YOLO11n pose output.
    - output: (1, 68, 8400)
    - orig_shape: (H, W, C)
    Returns: boxes, scores, keypoints
    """
    output = output[0]       # (68, 8400)
    output = output.T        # (8400, 68)

    boxes  = output[:, 0:4]  # x, y, w, h
    scores = output[:, 4]
    kpts   = output[:, 5:].reshape(-1, 21, 3)

    mask = scores > conf_thresh
    boxes, scores, kpts = boxes[mask], scores[mask], kpts[mask]

    if len(boxes) == 0:
        return np.zeros((0,4)), np.zeros((0,)), np.zeros((0,21,3))

    # [x,y,w,h] → [x1,y1,x2,y2]
    boxes_xyxy = np.zeros_like(boxes)
    boxes_xyxy[:, 0] = boxes[:, 0] - boxes[:, 2] / 2
    boxes_xyxy[:, 1] = boxes[:, 1] - boxes[:, 3] / 2
    boxes_xyxy[:, 2] = boxes[:, 0] + boxes[:, 2] / 2
    boxes_xyxy[:, 3] = boxes[:, 1] + boxes[:, 3] / 2

    H, W = orig_shape[:2]
    boxes_xyxy[:, [0, 2]] *= W / 640
    boxes_xyxy[:, [1, 3]] *= H / 640
    kpts[:, :, 0] *= W / 640
    kpts[:, :, 1] *= H / 640

    return boxes_xyxy, scores, kpts



# ---- DeepStream PGIE Pad Probe ----
def pgie_src_pad_buffer_probe(pad, info, u_data):
    gst_buffer = info.get_buffer()
    if not gst_buffer:
        return Gst.PadProbeReturn.OK

    batch_meta = pyds.gst_buffer_get_nvds_batch_meta(hash(gst_buffer))
    l_frame = batch_meta.frame_meta_list

    while l_frame is not None:
        try:
            frame_meta = pyds.NvDsFrameMeta.cast(l_frame.data)
        except StopIteration:
            break

        # TensorMeta suchen
        l_user = frame_meta.frame_user_meta_list
        while l_user is not None:
            user_meta = pyds.NvDsUserMeta.cast(l_user.data)
            if user_meta.base_meta.meta_type == pyds.NvDsMetaType.NVDSINFER_TENSOR_OUTPUT_META:
                print("✅ Tensor meta found on frame")
                trt_output = get_tensor_as_numpy(user_meta.user_meta_data)
                fh, fw = frame_meta.source_frame_height, frame_meta.source_frame_width

                # --- Decoder-Aufruf ---
                boxes, scores, kpts = decode_yolo11_output(trt_output, (fh, fw, 3), conf_thresh=CONF_THRESH)
                print(f"📦 Decoded boxes={len(boxes)}, scores>=0.4: {(scores > 0.4).sum()}")

                # --- Boxen zeichnen ---
                for i, box in enumerate(boxes):
                    x1, y1, x2, y2 = box
                    obj_meta = pyds.nvds_acquire_obj_meta_from_pool(batch_meta)
                    obj_meta.class_id = 0
                    obj_meta.obj_label = "hand"
                    obj_meta.confidence = float(scores[i])
                    rect = obj_meta.rect_params
                    rect.left, rect.top = float(x1), float(y1)
                    rect.width, rect.height = float(x2 - x1), float(y2 - y1)
                    rect.border_width = 2
                    rect.border_color.set(0.0, 1.0, 0.0, 1.0)
                    pyds.nvds_add_obj_meta_to_frame(frame_meta, obj_meta, None)

                    # --- Keypoints als Kreise ---
                    display_meta = pyds.nvds_acquire_display_meta_from_pool(batch_meta)
                    for (x, y, c) in kpts[i]:
                        if c > 0.3 and display_meta.num_circles < 16:
                            circ = display_meta.circle_params[display_meta.num_circles]
                            circ.xc = int(x)
                            circ.yc = int(y)
                            circ.radius = 2
                            circ.circle_color.set(1.0, 0.0, 0.0, 1.0)
                            display_meta.num_circles += 1
                    pyds.nvds_add_display_meta_to_frame(frame_meta, display_meta)

            try:
                l_user = l_user.next
            except StopIteration:
                break

        try:
            l_frame = l_frame.next
        except StopIteration:
            break

    return Gst.PadProbeReturn.OK




# === MAIN ===
def main():
    import gi
    gi.require_version('Gst', '1.0')
    gi.require_version('GstBase', '1.0')
    gi.require_version('GObject', '2.0')
    from gi.repository import Gst, GLib, GObject

    Gst.init(None)

    CAM_DEV = "/dev/video2"
    WIDTH, HEIGHT, FPS = 640, 480, 30
    #CONFIG_PATH = "config_infer_primary_yolo11_pose.txt"

    # --- Pipeline erstellen ---
    pipeline = Gst.Pipeline.new("ds-yolo11-pose")

    # === CAMERA: v4l2src → videoconvert → nvvideoconvert (NVMM) ===
    source = Gst.ElementFactory.make("v4l2src", "usb-camera")
    if not source:
        raise RuntimeError("Cannot create v4l2src")
    source.set_property("device", CAM_DEV)
    source.set_property("io-mode", 2)

    # Kamera liefert YUY2 → setze Caps
    caps_yuyv = Gst.ElementFactory.make("capsfilter", "caps_yuyv")
    caps_yuyv.set_property("caps", Gst.Caps.from_string(
        f"video/x-raw,format=YUY2,width={WIDTH},height={HEIGHT},framerate={FPS}/1"
    ))

    videoconvert = Gst.ElementFactory.make("videoconvert", "videoconvert")
    if not videoconvert:
        raise RuntimeError("Cannot create videoconvert")

    # Konvertiere zu NV12 im CPU-Speicher
    caps_nv12 = Gst.ElementFactory.make("capsfilter", "caps_nv12")
    caps_nv12.set_property("caps", Gst.Caps.from_string("video/x-raw,format=NV12"))

    # CPU → GPU (NVMM)
    nvvideoconvert = Gst.ElementFactory.make("nvvideoconvert", "nvvideoconvert")
    nvvideoconvert.set_property("nvbuf-memory-type", 0)

    caps_nvmm = Gst.ElementFactory.make("capsfilter", "caps_nvmm")
    caps_nvmm.set_property("caps", Gst.Caps.from_string(
        f"video/x-raw(memory:NVMM),format=NV12,width={WIDTH},height={HEIGHT},framerate={FPS}/1"
    ))

    # === DeepStream Elemente ===
    streammux = Gst.ElementFactory.make("nvstreammux", "stream-muxer")
    if not streammux:
        raise RuntimeError("Could not create nvstreammux")
    streammux.set_property("batch-size", 1)
    streammux.set_property("width", WIDTH)
    streammux.set_property("height", HEIGHT)
    streammux.set_property("live-source", 1)
    streammux.set_property("batched-push-timeout", 40000)

    pgie = Gst.ElementFactory.make("nvinfer", "primary-infer")
    if not pgie:
        raise RuntimeError("Could not create nvinfer")
    pgie.connect("notify::output-tensor-meta", lambda elem, pspec: print("🔹 output-tensor-meta set to", elem.get_property("output-tensor-meta")))
    pgie.set_property("config-file-path", CONFIG_PATH)
    if not os.path.exists(CONFIG_PATH):
        raise FileNotFoundError(f"Config file not found: {CONFIG_PATH}")
    else:
        print("Loaded nvinfer config:", CONFIG_PATH)
    pgie.set_property("batch-size", 1)
    print("🔍 pgie output-tensor-meta property =", pgie.get_property("output-tensor-meta"))

    nvdsosd = Gst.ElementFactory.make("nvdsosd", "onscreendisplay")
    transform = Gst.ElementFactory.make("nvegltransform", "egl-transform")
    sink = Gst.ElementFactory.make("nveglglessink", "egl-sink")
    sink.set_property("sync", False)

    # === Elemente zur Pipeline hinzufügen ===
    for e in [source, caps_yuyv, videoconvert, caps_nv12,
              nvvideoconvert, caps_nvmm, streammux, pgie, nvdsosd, transform, sink]:
        pipeline.add(e)

    # === Verknüpfen ===
    if not (
        source.link(caps_yuyv)
        and caps_yuyv.link(videoconvert)
        and videoconvert.link(caps_nv12)
        and caps_nv12.link(nvvideoconvert)
        and nvvideoconvert.link(caps_nvmm)
    ):  raise RuntimeError("Link camera conversion chain failed")

    # Verbinde Kamera (NVMM) mit Streammux
    sinkpad = streammux.get_request_pad("sink_0")
    if not sinkpad:
        raise RuntimeError("Cannot get sink_0 pad from streammux")

    srcpad = caps_nvmm.get_static_pad("src")
    if not srcpad:
        raise RuntimeError("Cannot get src pad from caps_nvmm")

    #print("\n==== CAPS DEBUG ====")
    #print("Camera src caps :", srcpad.query_caps(None).to_string())
    #print("Streammux sink caps :", sinkpad.query_caps(None).to_string())
    #print("====================\n")

    if srcpad.link(sinkpad) != Gst.PadLinkReturn.OK:
        raise RuntimeError("Link camera NVMM → streammux sink_0 failed")

    if not streammux.link(pgie):
        raise RuntimeError("link streammux→pgie failed")
    if not pgie.link(nvdsosd):
        raise RuntimeError("link pgie→nvdsosd failed")
    if not nvdsosd.link(transform):
        raise RuntimeError("link nvdsosd→transform failed")
    if not transform.link(sink):
        raise RuntimeError("link transform→sink failed")

    # --- Pad-Probe aktivieren ---
    pgie_src_pad = pgie.get_static_pad("src")
    if not pgie_src_pad:
        print("❌ ERROR: Kein src-Pad an PGIE gefunden!")
    else:
        pgie_src_pad.add_probe(Gst.PadProbeType.BUFFER, pgie_src_pad_buffer_probe, 0)
        print("🟢 PGIE src pad probe attached")


    # === Bus & MainLoop ===
    bus = pipeline.get_bus()
    bus.add_signal_watch()
    loop = GLib.MainLoop()

    def bus_call(bus, message, loop):
        t = message.type
        if t == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            print("❌ ERROR:", err, debug)
            loop.quit()
        elif t == Gst.MessageType.EOS:
            print("EOS (End of Stream)")
            loop.quit()
        return True

    bus.connect("message", bus_call, loop)

    print("Starting YUYV camera pipeline … press Ctrl+C to quit\n")
    pipeline.set_state(Gst.State.PLAYING)
    print("🟢 NVINFER config path =", pgie.get_property("config-file-path"))


    try:
        loop.run()
    except KeyboardInterrupt:
        print("\nStopping pipeline...")
    finally:
        pipeline.set_state(Gst.State.NULL)


if __name__ == "__main__":
    main()
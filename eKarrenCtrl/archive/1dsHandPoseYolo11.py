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

# ---- Tensor conversion (funktioniert mit DS 6.x / 7.x / Orin) ----
def get_tensor_as_numpy(tensor_meta_ptr):
    import ctypes
    import numpy as np
    import pyds

    tensor_meta = pyds.NvDsInferTensorMeta.cast(tensor_meta_ptr)
    outputs = []

    for i in range(tensor_meta.num_output_layers):
        layer = pyds.get_nvds_LayerInfo(tensor_meta, i)
        dims = layer.inferDims
        shape = [dims.d[j] for j in range(dims.numDims)]
        size = int(np.prod(shape))

        # ---- Host oder Device Buffer ----
        if layer.buffer:
            try:
                addr = pyds.get_ptr(layer.buffer)  # DS 7.x
            except AttributeError:
                addr = int(layer.buffer)           # DS 6.x
            ptr = ctypes.c_void_p(addr)
            arr = np.ctypeslib.as_array(
                ctypes.cast(ptr, ctypes.POINTER(ctypes.c_float)),
                shape=(size,)
            ).copy()
        else:
            # Kein Host-Puffer → GPU-Buffer kopieren
            import pycuda.driver as cuda
            dev_ptr = int(tensor_meta.out_buf_ptrs_dev[i])
            arr = np.empty(size, dtype=np.float32)
            cuda.memcpy_dtoh(arr, dev_ptr)

        arr = arr.reshape(shape)
        print(f"🧠 Layer{i}: shape={shape}, min={arr.min():.5f}, max={arr.max():.5f}")
        outputs.append(arr)

    return outputs[0] if len(outputs) == 1 else outputs




# ---- Probe: Postprocess + Object Meta creation (mit Debug + flexiblem Decoder) ----
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

        l_user = frame_meta.frame_user_meta_list
        trt_output = None
        while l_user is not None:
            try:
                user_meta = pyds.NvDsUserMeta.cast(l_user.data)
            except StopIteration:
                break

            if user_meta.base_meta.meta_type == pyds.NvDsMetaType.NVDSINFER_TENSOR_OUTPUT_META:
                print("✅ Tensor meta found on frame")
                trt_output = get_tensor_as_numpy(user_meta.user_meta_data)  # << kein .data !
                break

            try:
                l_user = l_user.next
            except StopIteration:
                break

        if trt_output is not None:
            # Wenn mehrere Layer: meist der erste ist der Kombi-Output
            out = trt_output[0] if isinstance(trt_output, list) else trt_output

            fh = int(frame_meta.source_frame_height)
            fw = int(frame_meta.source_frame_width)
            boxes, scores, kpts = decode_yolo11_output(out, fh, fw, conf_thresh=CONF_THRESH)
            print(f"📦 Decoded boxes={len(boxes)}, scores>={CONF_THRESH}: {np.count_nonzero(scores >= CONF_THRESH)}")

            #for i in range(len(boxes)):
            #    x1, y1, x2, y2 = boxes[i]
            #    w = max(0.0, float(x2 - x1))
            #    h = max(0.0, float(y2 - y1))
            #    if w <= 1 or h <= 1:
            #        continue  # zu klein, nicht anzeigen
            #
            #    conf = float(scores[i])
            #
            #    obj_meta = pyds.nvds_acquire_obj_meta_from_pool(batch_meta)
            #    obj_meta.class_id = 0
            #    obj_meta.obj_label = "hand"
            #    obj_meta.confidence = conf
            #
            #    rect = obj_meta.rect_params
            #    rect.left   = float(max(0.0, x1))
            #    rect.top    = float(max(0.0, y1))
            #    rect.width  = w
            #    rect.height = h
            #    rect.border_width = 2
            #    rect.has_bg_color = 0
            #
            #    txt = obj_meta.text_params
            #    txt.display_text = f"hand {conf:.2f}"
            #    txt.font_params.font_name = "Serif"
            #    txt.font_params.font_size = 12
            #    txt.font_params.font_color.set(1.0, 1.0, 0.0, 1.0)
            #
            #    pyds.nvds_add_obj_meta_to_frame(frame_meta, obj_meta, None)
            
            # Alles in einem DisplayMeta rendern
            num = len(boxes)
            if num > 0:
                display_meta = pyds.nvds_acquire_display_meta_from_pool(batch_meta)
                display_meta.num_rects = num
                display_meta.num_labels = num

                for i in range(display_meta.num_rects):
                    x1, y1, x2, y2 = boxes[i]
                    # clip & size
                    x1 = max(0.0, float(x1)); y1 = max(0.0, float(y1))
                    w  = max(1.0, float(x2 - x1)); h = max(1.0, float(y2 - y1))

                    rp = display_meta.rect_params[i]
                    rp.left = x1; rp.top = y1; rp.width = w; rp.height = h
                    rp.border_width = 3
                    rp.border_color.set(0.0, 1.0, 0.0, 1.0)   # grün
                    rp.has_bg_color = 0

                    lp = display_meta.text_params[i]
                    lp.display_text = f"hand {float(scores[i]):.2f}"
                    lp.x_offset = int(x1)
                    lp.y_offset = int(max(0, y1 - 8))
                    lp.font_params.font_name = "Serif"
                    lp.font_params.font_size = 12
                    lp.font_params.font_color.set(1.0, 1.0, 0.0, 1.0)
                    lp.set_bg_clr = 1
                    lp.text_bg_clr.set(0.0, 0.0, 0.0, 0.6)

                pyds.nvds_add_display_meta_to_frame(frame_meta, display_meta)



        try:
            l_frame = l_frame.next
        except StopIteration:
            break

    return Gst.PadProbeReturn.OK


# ---- Decode YOLO11 Pose output ----
def decode_yolo11_output(output, orig_h, orig_w, conf_thresh=0.4):
    import numpy as np

    # In (C,N) bringen
    out = output
    if out.ndim == 3 and out.shape[0] == 1:
        out = out[0]
    if out.shape[0] in (56, 68):         # (C, N)
        outCN = out
    elif out.shape[1] in (56, 68):       # (N, C)
        outCN = out.T
    else:
        print(f"WARN: unexpected output shape {out.shape} – skip")
        return (np.zeros((0,4), np.float32),
                np.zeros((0,), np.float32),
                np.zeros((0,21,3), np.float32))

    C, N = outCN.shape
    kpt_ch = C - 5
    num_kpts = 17 if kpt_ch == 51 else 21 if kpt_ch == 63 else 0
    if num_kpts == 0:
        print(f"WARN: keypoint channels {kpt_ch} not supported")
        return (np.zeros((0,4), np.float32),
                np.zeros((0,), np.float32),
                np.zeros((0,21,3), np.float32))

    xywh = outCN[0:4, :]      # (4, N)
    conf = outCN[4, :]        # (N,)
    kpts = outCN[5:, :].reshape(num_kpts, 3, -1)  # (K,3,N)

    keep = np.where(conf >= conf_thresh)[0]
    if keep.size == 0:
        return (np.zeros((0,4), np.float32),
                np.zeros((0,), np.float32),
                np.zeros((0,num_kpts,3), np.float32))

    xywh = xywh[:, keep]
    scores = conf[keep]
    kpts   = kpts[:, :, keep]

    x, y, w, h = xywh
    x1 = x - w/2.0
    y1 = y - h/2.0
    x2 = x + w/2.0
    y2 = y + h/2.0
    boxes = np.stack([x1, y1, x2, y2], axis=1)

    # Annahme: Netz rechnet im 640er Raum → auf Originalbild skalieren
    sx = float(orig_w) / 640.0
    sy = float(orig_h) / 640.0
    boxes[:, [0,2]] *= sx
    boxes[:, [1,3]] *= sy

    kpts_xyc = np.zeros((keep.size, num_kpts, 3), np.float32)
    kpts_xyc[:, :, 0] = (kpts[:, 0, :].T) * sx
    kpts_xyc[:, :, 1] = (kpts[:, 1, :].T) * sy
    kpts_xyc[:, :, 2] = (kpts[:, 2, :].T)

    return boxes.astype(np.float32), scores.astype(np.float32), kpts_xyc





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
#!/usr/bin/env python3
# deepstream_detect.py
#
# Python DeepStream pipeline that mirrors your config and prints detected objects + bboxes.
#
# Requirements:
#  - NVIDIA DeepStream SDK (7.x)
#  - pyds (DeepStream Python bindings)
#  - GStreamer (comes with DeepStream)
#
# Adjust paths (config_file) as needed.
"""
Was das Script macht:
- baut eine Pipeline ähnlich deiner deepstream-app-Konfiguration:
  - V4L2 Source (/dev/video2) mit 640×480 @30
  - nvstreammux mit Ausgabeframe 1280×720, batch-size=1
  - nvinfer (Primary GIE) mit Konfigurationsdatei (wie config_infer_primary.txt)
  - nvdsosd für OSD (Bounding Boxes/Text)
  - EGL Sink (nveglglessink) (auf Jetson empfehlenswert)
- hängt einen Probe Callback an (auf dem Pad nach dem GIE / vor OSD), 
  liest NvDsBatchMeta aus und iteriert alle Frames und Objekte
- gibt Klasse/ID + BoundingBox (links, top, width, height in Pixel und Normalized) auf stdout aus
- zeigt, wie du die erkannten Objekte an deine Applikation weiterreichen kannst 
  (z. B. in einer Python-Liste)
"""

import sys
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GObject, GLib

# pyds provides helpers to parse NvDs metadata
try:
    import pyds
except Exception as e:
    print("ERROR: pyds import failed. Make sure DeepStream Python bindings are installed.", file=sys.stderr)
    raise

Gst.init(None)

# Path to your primary infer config (config_infer_primary.txt) or model-engine-file can be set in that file.
PGIE_CONFIG_FILE = "config_infer_primary.txt"   # adjust path if needed
# If you want to override model engine path directly, set MODEL_ENGINE_FILE and assign on nvinfer element.
# MODEL_ENGINE_FILE = "/opt/nvidia/deepstream/deepstream-7.1/samples/models/Primary_Detector/resnet18_trafficcamnet_pruned.onnx_b2_gpu0_int8.engine"

# V4L2 device number from your config (camera-v4l2-dev-node=2 => /dev/video2)
V4L2_DEVICE = "/dev/video2"


CONF_THRESHOLD = 0.5  # only show boxes above this confidence


import pyds

CONF_THRESHOLD = 0.5  # minimum confidence
TARGET_CLASS = "person"  # only show this object class

def osd_sink_pad_buffer_probe(pad, info, u_data):
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

        l_obj = frame_meta.obj_meta_list
        while l_obj is not None:
            try:
                obj_meta = pyds.NvDsObjectMeta.cast(l_obj.data)
            except StopIteration:
                break

            # 🔹 Only show persons with sufficient confidence
            if obj_meta.obj_label == TARGET_CLASS and obj_meta.confidence >= CONF_THRESHOLD:
                # Visible bounding box
                obj_meta.rect_params.border_width = 2
                obj_meta.text_params.font_params.font_size = 14
                obj_meta.text_params.font_params.font_name = "Serif"
                obj_meta.text_params.font_params.font_color.set(1.0, 1.0, 1.0, 1.0)  # white
                obj_meta.text_params.set_bg_clr = 1
                obj_meta.text_params.text_bg_clr.set(0.3, 0.3, 0.3, 1.0)  # gray background

                # Custom text: "person: 0.87"
                obj_meta.text_params.display_text = f"{obj_meta.obj_label}: {obj_meta.confidence:.2f}"
            else:
                # 🔸 Hide non-persons or low-confidence detections
                obj_meta.rect_params.border_width = 0
                obj_meta.text_params.display_text = ""
                obj_meta.text_params.font_params.font_size = 0

            try:
                l_obj = l_obj.next
            except StopIteration:
                break

        try:
            l_frame = l_frame.next
        except StopIteration:
            break

    return Gst.PadProbeReturn.OK


def main():
    # Build pipeline
    pipeline = Gst.Pipeline()

    # Elements
    source = Gst.ElementFactory.make("v4l2src", "camera-source")
    if not source:
        print("Unable to create v4l2src. Is GStreamer + v4l2 plugin installed?", file=sys.stderr)
        return
    source.set_property("device", V4L2_DEVICE)
    # sometimes set caps via capsfilter for width/height/fps
    caps = Gst.ElementFactory.make("capsfilter", "caps")
    caps.set_property("caps", Gst.Caps.from_string("video/x-raw, width=640, height=480, framerate=30/1"))

    # convert to NVMM memory (for DeepStream)
    nvvidconv_src = Gst.ElementFactory.make("nvvidconv", "nvvidconv_src")
    # create streammux
    streammux = Gst.ElementFactory.make("nvstreammux", "stream-muxer")

    pgie = Gst.ElementFactory.make("nvinfer", "primary-infer")
    nvvidconv = Gst.ElementFactory.make("nvvidconv", "nvvidconv")
    nvosd = Gst.ElementFactory.make("nvdsosd", "nv-onscreendisplay")
    # sink - use nveglglessink on Jetson (EGL)
    sink = Gst.ElementFactory.make("nveglglessink", "egl-sink")

    if not streammux or not pgie or not nvosd or not sink or not nvvidconv:
        print("One or more GStreamer elements could not be created. Check DeepStream plugins.", file=sys.stderr)
        return

    # set properties according to your config
    streammux.set_property("width", 1280)
    streammux.set_property("height", 720)
    streammux.set_property("batch-size", 1)
    streammux.set_property("batched-push-timeout", 40000)
    streammux.set_property("live-source", 1)

    # set nvinfer config (use config file that you referenced in your .ini)
    pgie.set_property("config-file-path", PGIE_CONFIG_FILE)
    pgie.set_property("batch-size", 1)
    # if you need to set engine directly:
    # pgie.set_property("model-engine-file", MODEL_ENGINE_FILE)

    # camera source element capabilities; link elements
    pipeline.add(source)
    pipeline.add(caps)
    pipeline.add(nvvidconv_src)
    pipeline.add(streammux)
    pipeline.add(pgie)
    pipeline.add(nvvidconv)
    pipeline.add(nvosd)
    pipeline.add(sink)

    # link simple chain: source -> caps -> nvvidconv_src -> queue -> streammux (we must use sink pad of streammux)
    source.link(caps)
    caps.link(nvvidconv_src)

    # create a queue between nvvidconv_src and streammux to be safe
    queue_src = Gst.ElementFactory.make("queue", "queue_src")
    pipeline.add(queue_src)
    nvvidconv_src.link(queue_src)

    # Get sink pad of streammux for the one source (index 0)
    sinkpad = streammux.get_request_pad("sink_0")
    if not sinkpad:
        print("Unable to get the sink pad of streammux", file=sys.stderr)
        return

    # convert to NVMM mem if needed
    # link queue -> streammux sink pad (must use nvstreammux sink_0)
    srcpad = queue_src.get_static_pad("src")
    if srcpad is None:
        print("Failed to get src pad of queue_src", file=sys.stderr)
        return
    ret = srcpad.link(sinkpad)
    if ret != Gst.PadLinkReturn.OK:
        print("Failed to link source to streammux", file=sys.stderr)
        return

    # now link the rest: streammux -> pgie -> nvvidconv -> nvosd -> sink
    # streammux has a src pad "src", create an nvstreammux src pad via element link using ghost pads (use element link)
    if not Gst.Element.link(streammux, pgie):
        # nvstreammux is a special element; link with a queue in between via src pad retrieval
        # In many systems, simple element.link works. We attempt element.link and fallback
        print("streammux -> pgie link failed with element.link; trying pad-based linking...")

    # Use normal linking (works in DeepStream examples)
    streammux.link(pgie)
    pgie.link(nvvidconv)
    nvvidconv.link(nvosd)
    
    MIRROR = True
    if MIRROR:
        # Neues Element für horizontales Spiegeln hinzufügen
        flipconv = Gst.ElementFactory.make("nvvideoconvert", "flip-converter")
        flipconv.set_property("flip-method", 4)  # 0=none, 4=horizontal flip
        if not flipconv:
            print("Unable to create flip-converter element!", file=sys.stderr)
            return
        pipeline.add(flipconv)
        
         # nvosd -> flipconv -> sink
        nvosd.link(flipconv)
        flipconv.link(sink)   
    else: nvosd.link(sink)

    # attach probe to the pgie src pad OR to nvosd sink pad; we choose pgie src pad to get metadata immediately after inference
    pgie_src_pad = pgie.get_static_pad("src")
    if not pgie_src_pad:
        print("Unable to get pgie src pad", file=sys.stderr)
    else:
        pgie_src_pad.add_probe(Gst.PadProbeType.BUFFER, osd_sink_pad_buffer_probe, 0)

    # bus watch to catch EOS / errors
    bus = pipeline.get_bus()
    bus.add_signal_watch()

    def bus_call(bus, message, loop):
        t = message.type
        if t == Gst.MessageType.EOS:
            print("End-Of-Stream")
            loop.quit()
        elif t == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            print("ERROR:", err, debug)
            loop.quit()
        return True

    loop = GLib.MainLoop()
    bus.connect("message", bus_call, loop)

    # Start pipeline
    pipeline.set_state(Gst.State.PLAYING)
    try:
        print("Running pipeline. Ctrl+C to exit.")
        loop.run()
    except KeyboardInterrupt:
        print("Interrupted by user, stopping...")
    finally:
        pipeline.set_state(Gst.State.NULL)


if __name__ == '__main__':
    main()

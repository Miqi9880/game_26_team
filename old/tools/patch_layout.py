#!/usr/bin/env python3
"""Inject explicit NCHW/NRC layouts into the sp25 yolov5 IR in place.

The upstream IR carries no layout metadata; the C++ detector and the
production qualification gate both refuse to infer axis order.  This
script re-serializes the model with ov.Layout("NCHW") on the input and
ov.Layout("NRC") on the output, keeps f32 weights, overwrites
models/yolov5.xml + .bin, and rewrites the profile SHA-256 values.

Usage: python3 tools/patch_layout.py [models_dir] [profile_yaml]
"""

import hashlib
import pathlib
import re
import sys

import openvino as ov

home = pathlib.Path.home()
models = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else str(home / "game_26_orin_main/models"))
profile = pathlib.Path(sys.argv[2] if len(sys.argv) > 2 else str(home / "game_26_orin_main/config/model_profile.yaml"))

xml = models / "yolov5.xml"
bin_file = models / "yolov5.bin"
if not (xml.is_file() and bin_file.is_file()):
    sys.exit(f"missing model artifacts in {models}")

core = ov.Core()
model = core.read_model(xml, bin_file)
in_node = model.input(0).get_node()
out_node = model.output(0).get_node()
in_node.set_layout(ov.Layout("NCHW"))
out_node.set_layout(ov.Layout("NRC"))
print("input layout :", in_node.get_layout().to_string())
print("output layout:", out_node.get_layout().to_string())

tmp_xml = models / "yolov5.patched.xml"
ov.save_model(model, tmp_xml, compress_to_fp16=False)
tmp_bin = models / "yolov5.patched.bin"
if not (tmp_xml.is_file() and tmp_bin.is_file()):
    sys.exit("save_model did not produce both artifacts")

new_xml_hash = hashlib.sha256(tmp_xml.read_bytes()).hexdigest()
new_bin_hash = hashlib.sha256(tmp_bin.read_bytes()).hexdigest()
tmp_xml.replace(xml)
tmp_bin.replace(bin_file)
print("patched: " + str(xml))
print("patched: " + str(bin_file))
print("xml sha256:", new_xml_hash)
print("bin sha256:", new_bin_hash)

text = profile.read_text(encoding="utf-8")
first = re.sub(r"sha256: [0-9a-f]{64}", f"sha256: {new_xml_hash}", text, count=1)
second = re.sub(r"sha256: [0-9a-f]{64}", f"sha256: {new_bin_hash}", first, count=1)
profile.write_text(second, encoding="utf-8")
print("profile updated:", profile)

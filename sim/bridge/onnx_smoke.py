#!/usr/bin/env python3
"""Smoke-test an ONNX detector in tm_sim_ros: loads with CUDA + CPU providers, prints the I/O
names and shapes and the providers the session actually uses, times N runs, and exits non-zero
if CUDAExecutionProvider is not among the providers in use (onnxruntime silently falls back to
CPU when the CUDA libs are missing).

    docker run --rm --gpus all -v /media/alex/waspsim/cloud_build_bundle/work:/data/work:ro \
        -v "$PWD":/ws:ro tm_sim_ros:latest \
        python3 /ws/sim/bridge/onnx_smoke.py \
        /data/work/models/yolox_s_isaac/yolox_s_isaac_epoch190_416_static.onnx
"""
import argparse
import sys
import time

import numpy as np
import onnxruntime as ort

ap = argparse.ArgumentParser()
ap.add_argument("model")
ap.add_argument("--runs", type=int, default=50)
args = ap.parse_args()

print("onnxruntime", ort.__version__, "available:", ort.get_available_providers())
sess = ort.InferenceSession(args.model, providers=["CUDAExecutionProvider", "CPUExecutionProvider"])
used = sess.get_providers()
print("providers used:", used)
for i in sess.get_inputs():
    print("input ", i.name, i.shape, i.type)
for o in sess.get_outputs():
    print("output", o.name, o.shape, o.type)

inp = sess.get_inputs()[0]
shape = [d if isinstance(d, int) else 1 for d in inp.shape]
x = np.random.rand(*shape).astype(np.float32)
for _ in range(3):
    sess.run(None, {inp.name: x})
t0 = time.perf_counter()
for _ in range(args.runs):
    out = sess.run(None, {inp.name: x})
ms = (time.perf_counter() - t0) / args.runs * 1000.0
print("ms/inference %.2f over %d runs; output shapes %s" % (ms, args.runs, [o.shape for o in out]))

if "CUDAExecutionProvider" not in used:
    print("FAIL: CUDAExecutionProvider is not in use (fell back to %s)" % used)
    sys.exit(1)
print("OK")

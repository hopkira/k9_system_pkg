#!/usr/bin/env python3

from pathlib import Path

import torch
import transformers


model_path = Path(
    "~/k9_models/Qwen3-Reranker-0.6B"
).expanduser()

print("torch:", torch.__version__)
print("CUDA available:", torch.cuda.is_available())

if torch.cuda.is_available():
    print("CUDA device:", torch.cuda.get_device_name(0))

print("transformers:", transformers.__version__)
print("model path:", model_path)
print("model exists:", model_path.is_dir())

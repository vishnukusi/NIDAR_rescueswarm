# Model Weights Directory

Place your trained `best.pt` model weights file in this directory:

```text
assets/models/best.pt
```

## Security & Rules
- `*.pt`, `*.pth`, and `*.onnx` files are ignored by git in `.gitignore` to prevent committing private model weights or large binary files.
- The `YOLOPerceptionAdapter` (`src/rescue_swarm_core/perception.py`) defaults to loading from this path when running live inference.
- To execute live inference with `best.pt`:
  ```bash
  pip install ultralytics torch
  ```

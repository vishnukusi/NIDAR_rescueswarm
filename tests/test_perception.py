import sys
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from rescue_swarm_core.perception import (
    BoundingBox,
    Detection,
    YOLOPerceptionAdapter,
)


class PerceptionTests(unittest.TestCase):
    def test_bounding_box_valid(self):
        bbox = BoundingBox(xmin=0.1, ymin=0.2, xmax=0.5, ymax=0.6)
        self.assertAlmostEqual(bbox.center_x, 0.3)
        self.assertAlmostEqual(bbox.center_y, 0.4)
        self.assertAlmostEqual(bbox.width, 0.4)
        self.assertAlmostEqual(bbox.height, 0.4)

    def test_bounding_box_rejects_inverted_coords(self):
        with self.assertRaises(ValueError):
            BoundingBox(xmin=0.8, ymin=0.2, xmax=0.3, ymax=0.6)

    def test_bounding_box_rejects_out_of_bounds(self):
        with self.assertRaises(ValueError):
            BoundingBox(xmin=-0.1, ymin=0.0, xmax=0.5, ymax=0.5)
        with self.assertRaises(ValueError):
            BoundingBox(xmin=0.0, ymin=0.0, xmax=1.2, ymax=0.5)

    def test_detection_rejects_invalid_confidence(self):
        bbox = BoundingBox(xmin=0.1, ymin=0.1, xmax=0.2, ymax=0.2)
        with self.assertRaises(ValueError):
            Detection(class_id=0, class_name="person", confidence=1.2, bbox=bbox)
        with self.assertRaises(ValueError):
            Detection(class_id=0, class_name="person", confidence=-0.1, bbox=bbox)

    def test_yolo_adapter_missing_file_raises_file_not_found(self):
        non_existent_path = ROOT / "assets" / "models" / "non_existent_best.pt"
        with self.assertRaises(FileNotFoundError) as ctx:
            YOLOPerceptionAdapter(model_path=non_existent_path)
        self.assertIn("Model weights file not found", str(ctx.exception))

    def test_yolo_adapter_invalid_thresholds_raise_error(self):
        valid_model = ROOT / "assets" / "models" / "best.pt"
        if not valid_model.exists():
            valid_model = ROOT / "assets" / "models" / "best"
        if valid_model.exists():
            with self.assertRaises(ValueError):
                YOLOPerceptionAdapter(model_path=valid_model, confidence_threshold=-0.1)
            with self.assertRaises(ValueError):
                YOLOPerceptionAdapter(model_path=valid_model, confidence_threshold=1.1)
            with self.assertRaises(ValueError):
                YOLOPerceptionAdapter(model_path=valid_model, iou_threshold=-0.05)
            with self.assertRaises(ValueError):
                YOLOPerceptionAdapter(model_path=valid_model, iou_threshold=1.5)

    def test_yolo_adapter_predict_missing_image_raises_error(self):
        valid_model = ROOT / "assets" / "models" / "best.pt"
        if not valid_model.exists():
            valid_model = ROOT / "assets" / "models" / "best"
        if valid_model.exists():
            adapter = YOLOPerceptionAdapter(model_path=valid_model)
            with self.assertRaises(FileNotFoundError) as ctx:
                adapter.predict("non_existent_image_12345.jpg")
            self.assertIn("not found", str(ctx.exception).lower())

    def test_yolo_adapter_predict_none_image_raises_value_error(self):
        valid_model = ROOT / "assets" / "models" / "best.pt"
        if not valid_model.exists():
            valid_model = ROOT / "assets" / "models" / "best"
        if valid_model.exists():
            adapter = YOLOPerceptionAdapter(model_path=valid_model)
            with self.assertRaises(ValueError) as ctx:
                adapter.predict(None)
            self.assertIn("cannot be none", str(ctx.exception).lower())


if __name__ == "__main__":
    unittest.main()

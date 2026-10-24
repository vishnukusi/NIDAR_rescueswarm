"""Perception interface and model adapter for NIDAR RescueSwarm.

Provides structured detection types and the adapter for running real trained
YOLO model weights (e.g., best.pt). If weights or deep learning libraries are
absent, it raises descriptive errors rather than faking detections.
"""

from dataclasses import dataclass
from pathlib import Path
from typing import Any, List, Optional, Protocol, Tuple


@dataclass(frozen=True)
class BoundingBox:
    """Normalized bounding box coordinates [0.0, 1.0]."""
    xmin: float
    ymin: float
    xmax: float
    ymax: float

    def __post_init__(self) -> None:
        if not (0.0 <= self.xmin <= 1.0 and 0.0 <= self.xmax <= 1.0):
            raise ValueError(f"xmin and xmax must be in [0.0, 1.0], got ({self.xmin}, {self.xmax})")
        if not (0.0 <= self.ymin <= 1.0 and 0.0 <= self.ymax <= 1.0):
            raise ValueError(f"ymin and ymax must be in [0.0, 1.0], got ({self.ymin}, {self.ymax})")
        if self.xmin > self.xmax:
            raise ValueError(f"xmin ({self.xmin}) cannot be greater than xmax ({self.xmax})")
        if self.ymin > self.ymax:
            raise ValueError(f"ymin ({self.ymin}) cannot be greater than ymax ({self.ymax})")

    @property
    def center_x(self) -> float:
        return (self.xmin + self.xmax) / 2.0

    @property
    def center_y(self) -> float:
        return (self.ymin + self.ymax) / 2.0

    @property
    def width(self) -> float:
        return self.xmax - self.xmin

    @property
    def height(self) -> float:
        return self.ymax - self.ymin

    def iou(self, other: "BoundingBox") -> float:
        """Compute Intersection over Union (IoU) with another bounding box."""
        if not isinstance(other, BoundingBox):
            raise TypeError(f"other must be a BoundingBox instance, got {type(other)}")
        inter_xmin = max(self.xmin, other.xmin)
        inter_ymin = max(self.ymin, other.ymin)
        inter_xmax = min(self.xmax, other.xmax)
        inter_ymax = min(self.ymax, other.ymax)

        inter_w = max(0.0, inter_xmax - inter_xmin)
        inter_h = max(0.0, inter_ymax - inter_ymin)
        inter_area = inter_w * inter_h

        area_self = self.width * self.height
        area_other = other.width * other.height
        union_area = area_self + area_other - inter_area

        if union_area <= 0.0:
            return 0.0
        return inter_area / union_area


@dataclass(frozen=True)
class Detection:
    """Structured detection output from perception model."""
    class_id: int
    class_name: str
    confidence: float
    bbox: BoundingBox
    frame_id: Optional[str] = None
    track_id: Optional[int] = None
    tracker_session_id: Optional[str] = None
    source_id: Optional[str] = None

    def __post_init__(self) -> None:
        if not (0.0 <= self.confidence <= 1.0):
            raise ValueError(f"confidence must be between 0.0 and 1.0, got {self.confidence}")
        if not self.class_name:
            raise ValueError("class_name cannot be empty")
        if self.track_id is not None and self.track_id < 0:
            raise ValueError(f"track_id must be non-negative, got {self.track_id}")

    @property
    def scoped_track_id(self) -> Optional[str]:
        """Globally scoped track identifier: source_id:tracker_session_id:track_id.

        Guarantees that local numeric track IDs are never compared across different
        drones or across tracker resets. Returns None if track_id, source_id, or
        tracker_session_id is not set.
        """
        if self.track_id is None or not self.source_id or not self.tracker_session_id:
            return None
        return f"{self.source_id}:{self.tracker_session_id}:{self.track_id}"


class PerceptionEngine(Protocol):
    """Protocol for perception engines."""

    def predict(self, image: Any) -> List[Detection]:
        """Run inference on an image and return structured detections."""
        ...


class YOLOPerceptionAdapter:
    """Production adapter for trained YOLO weights (e.g., best.pt).

    Requires ultralytics or PyTorch to run actual inference.
    Fails fast with informative errors if weights or libraries are missing.
    """

    def __init__(
        self,
        model_path: str | Path,
        confidence_threshold: float = 0.50,
        iou_threshold: float = 0.45,
        target_classes: Optional[List[str]] = None,
        device: str = "cpu",
        tracker: str = "bytetrack.yaml",
    ) -> None:
        self.model_path = Path(model_path).resolve()
        self.confidence_threshold = confidence_threshold
        self.iou_threshold = iou_threshold
        self.target_classes = target_classes or ["pedestrian", "people", "person", "survivor", "human"]
        self.device = device
        self.tracker = tracker
        self._model: Any = None
        self._session_counter = 1
        self._tracker_session_id = "session_001"

        if not 0.0 <= confidence_threshold <= 1.0:
            raise ValueError("confidence_threshold must be between 0.0 and 1.0")
        if not 0.0 <= iou_threshold <= 1.0:
            raise ValueError("iou_threshold must be between 0.0 and 1.0")

        self._validate_environment()

    def _validate_environment(self) -> None:
        if not self.model_path.exists():
            raise FileNotFoundError(
                f"Model weights file not found: {self.model_path}. "
                "Please place the team's trained 'best.pt' file in assets/models/ "
                "or specify the correct absolute path."
            )

        try:
            from ultralytics import YOLO  # type: ignore
            self._yolo_cls = YOLO
        except ImportError as exc:
            raise ImportError(
                "The 'ultralytics' package is required to run inference with best.pt. "
                "Install it using 'pip install ultralytics torch'."
            ) from exc

    def load_model(self) -> None:
        """Load the model weights into memory."""
        if self._model is None:
            self._model = self._yolo_cls(str(self.model_path))

    def inspect_model(self) -> dict:
        """Inspect model metadata (classes, architecture, names)."""
        self.load_model()
        names = getattr(self._model, "names", {})
        return {
            "model_path": str(self.model_path),
            "classes": names,
            "num_classes": len(names),
            "device": self.device,
        }

    def predict(self, image: Any, frame_id: Optional[str] = None) -> List[Detection]:
        """Run real inference on the input image using the loaded model."""
        if image is None:
            raise ValueError("Input image cannot be None")
        if isinstance(image, (str, Path)):
            img_p = Path(image)
            if not img_p.exists():
                raise FileNotFoundError(f"Input image not found: {img_p}")

        effective_frame_id = frame_id or (Path(image).name if isinstance(image, (str, Path)) else None)

        self.load_model()
        results = self._model(
            image,
            conf=self.confidence_threshold,
            iou=self.iou_threshold,
            device=self.device,
            verbose=False,
        )

        detections: List[Detection] = []
        if not results:
            return detections

        result = results[0]
        boxes = getattr(result, "boxes", None)
        if boxes is None or len(boxes) == 0:
            return detections

        # Extract image dimensions for normalized coordinates
        img_h, img_w = result.orig_shape

        for box in boxes:
            cls_id = int(box.cls[0].item())
            cls_name = result.names.get(cls_id, f"class_{cls_id}")
            conf = float(box.conf[0].item())

            # Bounding box in xyxy absolute pixels
            xyxy = box.xyxy[0].tolist()
            xmin = max(0.0, min(1.0, xyxy[0] / img_w))
            ymin = max(0.0, min(1.0, xyxy[1] / img_h))
            xmax = max(0.0, min(1.0, xyxy[2] / img_w))
            ymax = max(0.0, min(1.0, xyxy[3] / img_h))

            # Filter target classes if specified
            if self.target_classes and cls_name.lower() not in [c.lower() for c in self.target_classes]:
                continue

            bbox = BoundingBox(xmin=xmin, ymin=ymin, xmax=xmax, ymax=ymax)
            detections.append(
                Detection(
                    class_id=cls_id,
                    class_name=cls_name,
                    confidence=conf,
                    bbox=bbox,
                    frame_id=effective_frame_id,
                )
            )

        return detections

    def reset_tracker(self) -> str:
        """Reset the local tracker state and increment the session generation.

        Guarantees that track IDs produced after a reset will not be associated
        with track IDs prior to the reset.
        """
        self._session_counter += 1
        self._tracker_session_id = f"session_{self._session_counter:03d}"
        if self._model is not None:
            predictor = getattr(self._model, "predictor", None)
            if predictor is not None:
                trackers = getattr(predictor, "trackers", None)
                if trackers:
                    for t in trackers:
                        if hasattr(t, "reset"):
                            t.reset()
                self._model.predictor = None
        return self._tracker_session_id

    @property
    def tracker_session_id(self) -> str:
        """Current tracker session identifier."""
        return self._tracker_session_id

    def track(
        self,
        image: Any,
        frame_id: Optional[str] = None,
        source_id: Optional[str] = None,
        persist: bool = True,
        tracker: Optional[str] = None,
    ) -> List[Detection]:
        """Run persistent tracking on the input image using the configured tracker."""
        if image is None:
            raise ValueError("Input image cannot be None")
        if isinstance(image, (str, Path)):
            img_p = Path(image)
            if not img_p.exists():
                raise FileNotFoundError(f"Input image not found: {img_p}")

        effective_frame_id = frame_id or (Path(image).name if isinstance(image, (str, Path)) else None)
        active_tracker = tracker or self.tracker

        self.load_model()
        results = self._model.track(
            image,
            conf=self.confidence_threshold,
            iou=self.iou_threshold,
            device=self.device,
            tracker=active_tracker,
            persist=persist,
            verbose=False,
        )

        detections: List[Detection] = []
        if not results:
            return detections

        result = results[0]
        boxes = getattr(result, "boxes", None)
        if boxes is None or len(boxes) == 0:
            return detections

        img_h, img_w = result.orig_shape

        for box in boxes:
            cls_id = int(box.cls[0].item())
            cls_name = result.names.get(cls_id, f"class_{cls_id}")
            conf = float(box.conf[0].item())

            track_id = int(box.id[0].item()) if (box.id is not None and len(box.id) > 0) else None

            xyxy = box.xyxy[0].tolist()
            xmin = max(0.0, min(1.0, xyxy[0] / img_w))
            ymin = max(0.0, min(1.0, xyxy[1] / img_h))
            xmax = max(0.0, min(1.0, xyxy[2] / img_w))
            ymax = max(0.0, min(1.0, xyxy[3] / img_h))

            if self.target_classes and cls_name.lower() not in [c.lower() for c in self.target_classes]:
                continue

            bbox = BoundingBox(xmin=xmin, ymin=ymin, xmax=xmax, ymax=ymax)
            detections.append(
                Detection(
                    class_id=cls_id,
                    class_name=cls_name,
                    confidence=conf,
                    bbox=bbox,
                    frame_id=effective_frame_id,
                    track_id=track_id,
                    tracker_session_id=self._tracker_session_id if track_id is not None else None,
                    source_id=source_id,
                )
            )

        return detections


def inspect_model_weights(model_path: str | Path) -> dict:
    """Inspect model weights file/archive directly without heavy runtime dependencies.

    Extracts framework, task, and class names from PyTorch/YOLO pickle metadata.
    """
    path = Path(model_path).resolve()
    if not path.exists():
        raise FileNotFoundError(f"Model path does not exist: {path}")

    import pickletools
    import zipfile

    # Handle both directory format and zipfile format
    pkl_bytes = None
    if path.is_dir():
        pkl_path = path / "data.pkl"
        if pkl_path.exists():
            pkl_bytes = pkl_path.read_bytes()
    elif zipfile.is_zipfile(path):
        with zipfile.ZipFile(path, "r") as z:
            for name in z.namelist():
                if name.endswith("data.pkl"):
                    pkl_bytes = z.read(name)
                    break

    if pkl_bytes is None:
        return {
            "path": str(path),
            "status": "present",
            "size_bytes": path.stat().st_size if path.is_file() else 0,
        }

    # Extract class dictionary from pickle opcodes
    classes = {}
    found_names = False
    current_key = None
    task = "detect"
    framework = "Ultralytics YOLOv8"

    for opcode, arg, pos in pickletools.genops(pkl_bytes):
        if arg == "names":
            found_names = True
        elif found_names:
            if opcode.name in ("BININT1", "BININT2", "BININT") and current_key is None:
                current_key = int(arg)
            elif opcode.name in ("BINUNICODE", "SHORT_BINUNICODE") and current_key is not None:
                classes[current_key] = str(arg)
                current_key = None
            elif opcode.name == "SETITEMS":
                break

    return {
        "framework": framework,
        "task": task,
        "classes": classes,
        "num_classes": len(classes),
        "target_survivor_classes": [classes[k] for k in classes if classes[k].lower() in ("pedestrian", "people", "person", "survivor")],
        "file_size_mb": round((path.stat().st_size if path.is_file() else sum(f.stat().st_size for f in path.rglob('*') if f.is_file())) / (1024 * 1024), 2),
    }


def annotate_detection_image(
    image_path: str | Path,
    detections: List[Detection],
    output_path: str | Path,
) -> None:
    """Render bounding boxes, class labels, and confidence badges onto image."""
    import cv2

    img = cv2.imread(str(image_path))
    if img is None:
        raise FileNotFoundError(f"Failed to read image at {image_path}")

    h, w, _ = img.shape
    colors = [(0, 230, 115), (0, 242, 254), (250, 180, 0), (160, 80, 250)]

    for idx, det in enumerate(detections):
        x1 = int(det.bbox.xmin * w)
        y1 = int(det.bbox.ymin * h)
        x2 = int(det.bbox.xmax * w)
        y2 = int(det.bbox.ymax * h)
        color = colors[idx % len(colors)]
        label = f"{det.class_name} {det.confidence:.2f}"

        # Draw bounding rectangle
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)

        # Draw label badge
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
        offset_y = -22 if idx % 2 == 0 else -4
        badge_y = y1 + offset_y
        cv2.rectangle(img, (x1, badge_y - th - 5), (x1 + tw + 6, badge_y + 1), color, -1)
        cv2.putText(img, label, (x1 + 3, badge_y - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (10, 10, 10), 1, cv2.LINE_AA)

    out_p = Path(output_path).resolve()
    out_p.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out_p), img)


def extract_frame_from_video(
    video_path: str | Path,
    frame_index: int = 30,
    output_path: Optional[str | Path] = None,
) -> Path:
    """Extract a single reproducible frame from a video file without modifying the original video.

    Args:
        video_path: Path to the input video file (e.g. assets/vid1.mp4).
        frame_index: 0-indexed frame number to extract (default: 30).
        output_path: Optional destination path for extracted JPEG. If None, saves next to video.

    Returns:
        Path to the saved frame image.
    """
    v_path = Path(video_path).resolve()
    if not v_path.exists():
        raise FileNotFoundError(f"Video file not found: {v_path}")

    import cv2
    cap = cv2.VideoCapture(str(v_path))
    if not cap.isOpened():
        raise ValueError(f"Failed to open video file: {v_path}")

    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    if frame_index < 0 or (total_frames > 0 and frame_index >= total_frames):
        cap.release()
        raise ValueError(f"Frame index {frame_index} out of range [0, {total_frames - 1}]")

    cap.set(cv2.CAP_PROP_POS_FRAMES, frame_index)
    ret, frame = cap.read()
    cap.release()

    if not ret or frame is None:
        raise ValueError(f"Failed to read frame index {frame_index} from {v_path}")

    if output_path is None:
        output_path = v_path.parent / f"{v_path.stem}_frame_{frame_index:04d}.jpg"
    out_p = Path(output_path).resolve()
    out_p.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out_p), frame)
    return out_p



"""Multi-angle frame fusion and analysis.

Combines CCD frames captured under different flash lamp angles to
improve part detection and classification.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from ccd_inspector.core.frame import CCDFrame
from ccd_inspector.processing.edge_detect import DetectedObject, EdgeDetector
from ccd_inspector.processing.filters import normalize


@dataclass
class AngleResult:
    """Edge detection result for one illumination angle."""

    lamp_id: int
    angle_deg: float
    objects: list[DetectedObject]
    shadow_depth: float  # Max shadow amplitude


@dataclass
class MultiAngleResult:
    """Combined analysis across all illumination angles."""

    per_angle: list[AngleResult]
    fused_objects: list[DetectedObject]
    confidence_map: np.ndarray  # Per-pixel confidence of shadow presence

    @property
    def num_angles(self) -> int:
        return len(self.per_angle)


class MultiAngleAnalyzer:
    """Fuses frames captured under different illumination angles."""

    def __init__(
        self,
        detector: EdgeDetector,
        lamp_angles: dict[int, float] | None = None,
    ):
        self.detector = detector
        self.lamp_angles = lamp_angles or {}

    def analyze(self, frames: dict[int, CCDFrame]) -> MultiAngleResult:
        """Analyze frames keyed by lamp_id.

        Returns per-angle results and fused object detections.
        """
        per_angle: list[AngleResult] = []

        for lamp_id, frame in sorted(frames.items()):
            objs = self.detector.detect_objects(frame.pixels)
            eff = frame.effective_pixels.astype(float)
            baseline = np.median(eff)
            depth = float(baseline - np.min(eff)) if len(eff) > 0 else 0

            per_angle.append(AngleResult(
                lamp_id=lamp_id,
                angle_deg=self.lamp_angles.get(lamp_id, 0.0),
                objects=objs,
                shadow_depth=depth,
            ))

        # Build per-pixel shadow confidence by combining all angles
        confidence_map = self._build_confidence_map(frames)

        # Fuse object detections across angles
        fused = self._fuse_objects(per_angle)

        return MultiAngleResult(
            per_angle=per_angle,
            fused_objects=fused,
            confidence_map=confidence_map,
        )

    def _build_confidence_map(self, frames: dict[int, CCDFrame]) -> np.ndarray:
        """Build per-pixel shadow confidence from all angles.

        Each angle contributes a normalized shadow signal. The confidence
        at each pixel is the mean normalized shadow depth across angles.
        """
        signals = []
        for frame in frames.values():
            s = frame.pixels.astype(float)
            baseline = np.median(s)
            # Shadow depth = how far below baseline (positive = deeper shadow)
            shadow = np.clip(baseline - s, 0, None)
            if shadow.max() > 0:
                signals.append(shadow / shadow.max())
            else:
                signals.append(np.zeros_like(shadow))

        if not signals:
            return np.zeros(2200)

        return np.mean(signals, axis=0)

    def _fuse_objects(self, per_angle: list[AngleResult]) -> list[DetectedObject]:
        """Merge object detections across angles by position overlap.

        Objects at similar positions across multiple angles are merged.
        Objects seen at only one angle are kept but flagged.
        """
        if not per_angle:
            return []

        # Collect all objects with their source angle
        all_objects: list[tuple[DetectedObject, int]] = []
        for ar in per_angle:
            for obj in ar.objects:
                all_objects.append((obj, ar.lamp_id))

        if not all_objects:
            return []

        # Sort by center position
        all_objects.sort(key=lambda x: x[0].center_px)

        # Cluster objects that overlap in position
        clusters: list[list[tuple[DetectedObject, int]]] = []
        current_cluster = [all_objects[0]]

        for obj, lamp_id in all_objects[1:]:
            prev_obj = current_cluster[-1][0]
            # Objects overlap if their centers are within half the wider object's width
            max_width = max(prev_obj.width_px, obj.width_px)
            if abs(obj.center_px - prev_obj.center_px) < max_width * 0.5:
                current_cluster.append((obj, lamp_id))
            else:
                clusters.append(current_cluster)
                current_cluster = [(obj, lamp_id)]

        clusters.append(current_cluster)

        # For each cluster, pick the detection with the sharpest edges
        fused: list[DetectedObject] = []
        for cluster in clusters:
            best = max(cluster, key=lambda x: x[0].edge_sharpness)
            fused.append(best[0])

        return fused

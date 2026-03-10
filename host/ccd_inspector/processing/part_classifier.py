"""Part classification: singlated vs grouped.

Uses shadow width, edge sharpness, profile shape, and multi-angle
illumination data to determine if a detected object is a single part
or a cluster of overlapping parts.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from ccd_inspector.comm.protocol import EFFECTIVE_END, EFFECTIVE_START
from ccd_inspector.core.frame import CCDFrame
from ccd_inspector.processing.edge_detect import DetectedObject, EdgeDetector


@dataclass
class PartMeasurement:
    """Classification result for a detected object."""

    center_px: float
    width_px: float
    width_mm: float
    amplitude: float           # Shadow depth (ADC units below baseline)
    edge_sharpness: float      # Average gradient magnitude at edges
    profile_symmetry: float    # 0 = symmetric, higher = asymmetric

    classification: str        # 'single', 'grouped', 'unknown'
    confidence: float          # 0.0 to 1.0
    reasons: list[str] = field(default_factory=list)


class PartClassifier:
    """Classifies detected objects as singlated or grouped."""

    def __init__(
        self,
        known_part_width_mm: float = 5.0,
        pixel_pitch_mm: float = 0.014,
        width_tolerance_pct: float = 15.0,
        min_edge_sharpness: float = 30.0,
        symmetry_threshold: float = 0.3,
    ):
        self.known_part_width_mm = known_part_width_mm
        self.pixel_pitch_mm = pixel_pitch_mm
        self.width_tolerance_pct = width_tolerance_pct
        self.min_edge_sharpness = min_edge_sharpness
        self.symmetry_threshold = symmetry_threshold

        # Derived
        self._expected_width_px = known_part_width_mm / pixel_pitch_mm
        self._width_tol_px = self._expected_width_px * width_tolerance_pct / 100

    def classify(
        self,
        obj: DetectedObject,
        signal: np.ndarray,
    ) -> PartMeasurement:
        """Classify a detected object from a single-angle frame."""
        width_px = obj.width_px
        width_mm = width_px * self.pixel_pitch_mm
        center_px = obj.center_px

        # Extract the shadow profile
        left = max(0, int(obj.left_edge.position))
        right = min(len(signal), int(obj.right_edge.position) + 1)
        profile = signal[left:right].astype(float)

        # Compute metrics
        baseline = self._estimate_baseline(signal, obj)
        amplitude = baseline - float(np.min(profile)) if len(profile) > 0 else 0
        symmetry = self._profile_symmetry(profile)
        sharpness = obj.edge_sharpness

        # Classification logic
        reasons = []
        scores = []  # (is_single_indicator, weight)

        # Width check
        width_diff = abs(width_px - self._expected_width_px)
        if width_diff <= self._width_tol_px:
            scores.append((True, 2.0))
            reasons.append(f"Width {width_mm:.2f}mm within tolerance")
        elif width_px > self._expected_width_px * 1.5:
            scores.append((False, 3.0))
            reasons.append(f"Width {width_mm:.2f}mm >> expected {self.known_part_width_mm:.2f}mm")
        else:
            scores.append((False, 1.0))
            reasons.append(f"Width {width_mm:.2f}mm outside tolerance")

        # Edge sharpness
        if sharpness >= self.min_edge_sharpness:
            scores.append((True, 1.0))
            reasons.append(f"Sharp edges ({sharpness:.0f})")
        else:
            scores.append((False, 1.0))
            reasons.append(f"Soft edges ({sharpness:.0f})")

        # Profile symmetry
        if symmetry < self.symmetry_threshold:
            scores.append((True, 1.0))
            reasons.append(f"Symmetric profile ({symmetry:.2f})")
        else:
            scores.append((False, 1.0))
            reasons.append(f"Asymmetric profile ({symmetry:.2f})")

        # Internal edges — check for dips inside the shadow
        has_internal = self._detect_internal_edges(profile)
        if has_internal:
            scores.append((False, 2.0))
            reasons.append("Internal edges detected")
        else:
            scores.append((True, 1.0))

        # Compute weighted confidence
        single_weight = sum(w for is_s, w in scores if is_s)
        total_weight = sum(w for _, w in scores)
        confidence = single_weight / total_weight if total_weight > 0 else 0.5

        if confidence >= 0.65:
            classification = 'single'
        elif confidence <= 0.35:
            classification = 'grouped'
        else:
            classification = 'unknown'

        return PartMeasurement(
            center_px=center_px,
            width_px=width_px,
            width_mm=width_mm,
            amplitude=amplitude,
            edge_sharpness=sharpness,
            profile_symmetry=symmetry,
            classification=classification,
            confidence=confidence,
            reasons=reasons,
        )

    def classify_multi_angle(
        self,
        obj: DetectedObject,
        frames: dict[int, CCDFrame],
        detector: EdgeDetector,
    ) -> PartMeasurement:
        """Classify using multi-angle illumination data.

        Multi-angle indicators:
        - Width consistency across angles → single part
        - Width variation → stacked/overlapping parts
        - Internal edges appear at oblique angles → grouped
        """
        # Get base classification from first available frame
        first_frame = next(iter(frames.values()))
        result = self.classify(obj, first_frame.pixels)

        if len(frames) < 2:
            return result

        # Measure width at each angle
        widths: list[float] = []
        internal_edge_count = 0

        for lamp_id, frame in frames.items():
            objs = detector.detect_objects(frame.pixels)
            # Find the object closest to our target center
            best = None
            best_dist = float('inf')
            for o in objs:
                dist = abs(o.center_px - obj.center_px)
                if dist < best_dist:
                    best_dist = dist
                    best = o
            if best and best_dist < obj.width_px:
                widths.append(best.width_px)
                profile = frame.pixels[
                    int(best.left_edge.position):int(best.right_edge.position) + 1
                ].astype(float)
                if self._detect_internal_edges(profile):
                    internal_edge_count += 1

        if widths:
            width_std = float(np.std(widths))
            width_mean = float(np.mean(widths))
            width_cv = width_std / width_mean if width_mean > 0 else 0

            if width_cv > 0.15:
                result.reasons.append(
                    f"Width varies across angles (CV={width_cv:.2f})"
                )
                result.confidence *= 0.6
            else:
                result.reasons.append(
                    f"Width consistent across angles (CV={width_cv:.2f})"
                )
                result.confidence = min(1.0, result.confidence * 1.2)

        if internal_edge_count > 0:
            result.reasons.append(
                f"Internal edges in {internal_edge_count}/{len(frames)} angles"
            )
            result.confidence *= 0.5

        # Reclassify based on updated confidence
        if result.confidence >= 0.65:
            result.classification = 'single'
        elif result.confidence <= 0.35:
            result.classification = 'grouped'
        else:
            result.classification = 'unknown'

        return result

    def _estimate_baseline(
        self, signal: np.ndarray, obj: DetectedObject
    ) -> float:
        """Estimate the baseline (no-shadow) signal level near the object."""
        margin = 50
        left = max(0, int(obj.left_edge.position) - margin)
        right_start = min(len(signal), int(obj.right_edge.position) + 10)
        right_end = min(len(signal), right_start + margin)

        samples = np.concatenate([
            signal[left:max(left, int(obj.left_edge.position) - 10)],
            signal[right_start:right_end],
        ])

        return float(np.median(samples)) if len(samples) > 0 else 0.0

    def _profile_symmetry(self, profile: np.ndarray) -> float:
        """Measure left-right symmetry of the shadow profile.

        Returns 0 for perfectly symmetric, higher for asymmetric.
        """
        if len(profile) < 4:
            return 0.0

        n = len(profile)
        half = n // 2
        left_half = profile[:half]
        right_half = profile[n - half:][::-1]

        # Truncate to same length
        min_len = min(len(left_half), len(right_half))
        if min_len == 0:
            return 0.0

        diff = np.abs(left_half[:min_len] - right_half[:min_len])
        amplitude = float(np.ptp(profile))
        if amplitude < 1:
            return 0.0
        return float(np.mean(diff)) / amplitude

    def _detect_internal_edges(self, profile: np.ndarray) -> bool:
        """Check if the shadow profile has internal edges (bumps/dips).

        Indicates overlapping or grouped parts.
        """
        if len(profile) < 20:
            return False

        from scipy.signal import find_peaks

        # Smooth the profile to ignore noise
        from ccd_inspector.processing.filters import smooth_gaussian
        smoothed = smooth_gaussian(profile, sigma=5.0)

        # Look for local maxima inside the shadow (bright spots between parts)
        margin = max(3, len(smoothed) // 10)
        interior = smoothed[margin:-margin]
        if len(interior) < 5:
            return False

        peaks, props = find_peaks(interior, prominence=50)
        return len(peaks) > 0

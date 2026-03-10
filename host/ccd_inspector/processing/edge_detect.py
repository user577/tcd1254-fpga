"""1D edge detection for CCD line scan data.

Detects rising (bright-to-dark) and falling (dark-to-bright) edges in the
CCD signal, pairs them into shadow regions representing parts on the sensor.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.signal import find_peaks

from ccd_inspector.comm.protocol import EFFECTIVE_END, EFFECTIVE_START
from ccd_inspector.processing.filters import compute_gradient, smooth_gaussian


@dataclass
class Edge:
    """A detected edge in the CCD signal."""

    position: float      # Pixel index (subpixel)
    polarity: str        # 'rising' (bright→dark) or 'falling' (dark→bright)
    strength: float      # Gradient magnitude at the edge

    @property
    def position_dict(self) -> dict:
        return {
            'position': self.position,
            'polarity': self.polarity,
            'strength': self.strength,
        }


@dataclass
class DetectedObject:
    """A shadow region (part) bounded by a rising and falling edge."""

    left_edge: Edge
    right_edge: Edge

    @property
    def center_px(self) -> float:
        return (self.left_edge.position + self.right_edge.position) / 2

    @property
    def width_px(self) -> float:
        return self.right_edge.position - self.left_edge.position

    @property
    def edge_sharpness(self) -> float:
        """Average gradient magnitude of both edges."""
        return (self.left_edge.strength + self.right_edge.strength) / 2


class EdgeDetector:
    """1D edge detection on CCD line scan data."""

    def __init__(
        self,
        gradient_window: int = 20,
        smooth_sigma: float = 3.0,
        threshold: float = 50.0,
        min_width_px: int = 10,
        max_width_px: int = 1500,
        min_edge_separation: int = 5,
        search_start: int = EFFECTIVE_START,
        search_end: int = EFFECTIVE_END,
    ):
        self.gradient_window = gradient_window
        self.smooth_sigma = smooth_sigma
        self.threshold = threshold
        self.min_width_px = min_width_px
        self.max_width_px = max_width_px
        self.min_edge_separation = min_edge_separation
        self.search_start = search_start
        self.search_end = search_end

    def detect_edges(self, signal: np.ndarray) -> list[Edge]:
        """Find all rising and falling edges in the signal.

        Rising edge = bright to dark transition (negative gradient, shadow entry).
        Falling edge = dark to bright transition (positive gradient, shadow exit).
        """
        # Smooth to reduce noise
        smoothed = smooth_gaussian(signal.astype(float), sigma=self.smooth_sigma)

        # Compute gradient over window
        grad = compute_gradient(smoothed, window=self.gradient_window)

        # Limit search to effective pixel region
        region = grad[self.search_start:self.search_end]

        edges: list[Edge] = []

        # Find negative gradient peaks (bright→dark = shadow entry = "rising" edge)
        neg_region = -region
        neg_peaks, neg_props = find_peaks(
            neg_region,
            height=self.threshold,
            distance=self.min_edge_separation,
        )
        for pk, height in zip(neg_peaks, neg_props['peak_heights']):
            abs_pos = pk + self.search_start
            subpixel = self._subpixel_refine(neg_region, pk)
            edges.append(Edge(
                position=subpixel + self.search_start,
                polarity='rising',
                strength=float(height),
            ))

        # Find positive gradient peaks (dark→bright = shadow exit = "falling" edge)
        pos_peaks, pos_props = find_peaks(
            region,
            height=self.threshold,
            distance=self.min_edge_separation,
        )
        for pk, height in zip(pos_peaks, pos_props['peak_heights']):
            abs_pos = pk + self.search_start
            subpixel = self._subpixel_refine(region, pk)
            edges.append(Edge(
                position=subpixel + self.search_start,
                polarity='falling',
                strength=float(height),
            ))

        # Sort by position
        edges.sort(key=lambda e: e.position)
        return edges

    def detect_objects(self, signal: np.ndarray) -> list[DetectedObject]:
        """Pair rising/falling edges into detected objects (shadow regions).

        Expects alternating rising→falling edge pairs.
        """
        edges = self.detect_edges(signal)
        objects: list[DetectedObject] = []

        # Pair up: find each rising edge followed by the next falling edge
        rising_edges = [e for e in edges if e.polarity == 'rising']
        falling_edges = [e for e in edges if e.polarity == 'falling']

        for rising in rising_edges:
            # Find the nearest falling edge after this rising edge
            best_falling = None
            for falling in falling_edges:
                if falling.position > rising.position:
                    best_falling = falling
                    break

            if best_falling is None:
                continue

            width = best_falling.position - rising.position
            if self.min_width_px <= width <= self.max_width_px:
                objects.append(DetectedObject(
                    left_edge=rising,
                    right_edge=best_falling,
                ))
                # Remove this falling edge so it's not reused
                falling_edges.remove(best_falling)

        return objects

    @staticmethod
    def _subpixel_refine(signal: np.ndarray, peak_idx: int) -> float:
        """Parabolic interpolation for subpixel peak position."""
        if peak_idx <= 0 or peak_idx >= len(signal) - 1:
            return float(peak_idx)

        y_m1 = signal[peak_idx - 1]
        y_0 = signal[peak_idx]
        y_p1 = signal[peak_idx + 1]

        denom = 2.0 * (2 * y_0 - y_m1 - y_p1)
        if abs(denom) < 1e-10:
            return float(peak_idx)

        offset = (y_m1 - y_p1) / denom
        return peak_idx + offset

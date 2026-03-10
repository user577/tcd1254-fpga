"""Signal conditioning filters for CCD data."""

from __future__ import annotations

import numpy as np
from scipy.ndimage import gaussian_filter1d, median_filter


def smooth_gaussian(signal: np.ndarray, sigma: float = 3.0) -> np.ndarray:
    """Gaussian smoothing."""
    return gaussian_filter1d(signal.astype(float), sigma=sigma)


def smooth_median(signal: np.ndarray, size: int = 5) -> np.ndarray:
    """Median filter — good for salt-and-pepper noise."""
    return median_filter(signal.astype(float), size=size)


def dark_subtract(signal: np.ndarray, dark: np.ndarray) -> np.ndarray:
    """Subtract dark frame, clamp to zero."""
    result = signal.astype(float) - dark.astype(float)
    return np.clip(result, 0, None)


def flat_field_correct(signal: np.ndarray, flat: np.ndarray) -> np.ndarray:
    """Normalize by flat field response."""
    flat_f = flat.astype(float)
    mean_flat = np.mean(flat_f)
    # Avoid division by zero
    safe = np.where(flat_f > 10, flat_f, mean_flat)
    return signal.astype(float) * (mean_flat / safe)


def normalize(signal: np.ndarray) -> np.ndarray:
    """Normalize to [0, 1] range."""
    s = signal.astype(float)
    lo, hi = s.min(), s.max()
    if hi - lo < 1e-6:
        return np.zeros_like(s)
    return (s - lo) / (hi - lo)


def compute_gradient(signal: np.ndarray, window: int = 1) -> np.ndarray:
    """Compute forward difference gradient.

    With window > 1, computes signal[i+window] - signal[i] (sliding difference).
    """
    s = signal.astype(float)
    if window == 1:
        grad = np.zeros_like(s)
        grad[:-1] = np.diff(s)
        return grad

    grad = np.zeros_like(s)
    grad[:len(s) - window] = s[window:] - s[:len(s) - window]
    return grad

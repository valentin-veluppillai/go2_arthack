"""Human channel colours for map points: photographic RGB, or a stand-in texture for offline tests.

The image is blurred heavily before sampling, so small misalignments between
the image and the map (pose timing, calibration) only shift soft colour
gradients instead of putting sharp edges on the wrong surfaces. The live
camera path is meant to use the same blur.

Offline, a panorama (equirectangular: azimuth across, elevation down) is
wrapped around a point in the map, so every direction the view cone can
point has colour. Without a real panorama, test_pattern() draws a distinct
one: twelve hue sectors around the horizon, checkered in brightness every 15°
of elevation, so the orientation and the blur are easy to read in the viz.
"""

from __future__ import annotations

import colorsys

import numpy as np
from scipy.ndimage import gaussian_filter

PATTERN_SIZE = (1024, 512)  # width, height
PATTERN_SECTORS = 12        # hue sectors around the horizon (30° each)
PATTERN_BAND = 15.0         # degrees of elevation per brightness band


def test_pattern(width: int = PATTERN_SIZE[0], height: int = PATTERN_SIZE[1]) -> np.ndarray:
    """Equirectangular test panorama, (height, width, 3) float32 in 0–1."""
    sector = np.arange(width) * PATTERN_SECTORS // width
    band = (np.arange(height) * 180 / height // PATTERN_BAND).astype(int)
    hues = np.array([colorsys.hsv_to_rgb(s / PATTERN_SECTORS, 0.85, 1.0) for s in range(PATTERN_SECTORS)])
    bright = np.where((sector[None, :] + band[:, None]) % 2 == 0, 1.0, 0.45)
    return (hues[sector][None, :, :] * bright[:, :, None]).astype(np.float32)


def load_image(path: str) -> np.ndarray:
    """An image file as (height, width, 3) float32 in 0–1 (needs Pillow)."""
    from PIL import Image  # only needed for real images

    return np.asarray(Image.open(path).convert("RGB"), dtype=np.float32) / 255


def blur(image: np.ndarray, fraction: float, wrap: bool = False) -> np.ndarray:
    """Gaussian blur with sigma = fraction of the image width; wrap joins the left and right edges
    (for a panorama)."""
    sigma = fraction * image.shape[1]
    if sigma <= 0:
        return image
    return gaussian_filter(image, sigma=(sigma, sigma, 0), mode=["nearest", "wrap" if wrap else "nearest", "nearest"])


def panorama_centre(points: np.ndarray) -> np.ndarray:
    """Where go2_viz's virtual robot stands: centre of the map's footprint, 0.3 m above the floor."""
    lo, hi = points.min(axis=0), points.max(axis=0)
    return np.array([(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, np.percentile(points[:, 2], 2) + 0.3])


def panorama_colors(points: np.ndarray, image: np.ndarray, centre: np.ndarray) -> np.ndarray:
    """Colour of each point (z-up map frame) seen from centre in an equirectangular image whose
    centre column looks along +x, left of it along +y. Returns (N, 3) uint8."""
    d = points - centre
    azimuth = np.arctan2(d[:, 1], d[:, 0])                        # +x = 0, +y (left) = +π/2
    elevation = np.arctan2(d[:, 2], np.hypot(d[:, 0], d[:, 1]))  # up = +π/2
    h, w = image.shape[:2]
    u = ((0.5 - azimuth / (2 * np.pi)) * w).astype(int) % w
    v = np.clip(((0.5 - elevation / np.pi) * h).astype(int), 0, h - 1)
    return (image[v, u] * 255).round().astype(np.uint8)

"""Grass-like ground texture for the orchard (pure numpy, no Isaac imports).

Why: a flat-colour ground is textureless, and stereo block matching (sim_perception SGBM) cannot match it. At 15 m
the canopy-to-ground disparity step is under 1 px, and the matcher fills the untextured ground by propagation from
the crowns: in the first closed-loop smoke (results/corridor_nav_isaac.md, "Smoke") 24 % of the ground pixels came
out higher than 1 m, and TreeMapper mapped phantom crowns and over-wide outlines from them. Real grass, and the
recorded Isaac scene (grass instances), have texture at the scale the cameras resolve (3.3 cm/px at 15 m, 1.2 cm at
5.5 m). The spec's ``clutter.grass`` is rendered as this texture on the ground mesh, not as grass geometry.

The texture is band-limited noise made periodic by construction (filtered in the Fourier domain), so it tiles
seamlessly; it is mapped planar in world xy, one tile every ``TILE_M`` metres (far larger than the 64 px stereo search
range at any flight height, so the repetition cannot create false matches). It only modulates the brightness of the
terrain colour, the hue stays.
"""
from typing import Sequence, Tuple

import numpy as np

TILE_M = 8.0                  # m of ground per texture tile
SIZE_PX = 1024                # texels per tile side (7.8 mm per texel)
WAVELENGTH_M = (0.03, 0.40)   # band of the noise: blades and clumps
CONTRAST = 0.45               # brightness modulation, +- (std of the multiplier ~ CONTRAST / 2.5)


def grass_texture(rgb: Sequence[float], seed: int = 0, size: int = SIZE_PX, tile_m: float = TILE_M,
                  wavelength_m: Tuple[float, float] = WAVELENGTH_M, contrast: float = CONTRAST) -> np.ndarray:
    """(size, size, 3) uint8 texture: ``rgb`` (0..1) times a periodic band-pass noise brightness multiplier."""
    if size < 16 or tile_m <= 0.0 or not (0.0 < wavelength_m[0] < wavelength_m[1]):
        raise ValueError("bad texture parameters: size %r, tile %r m, band %r m" % (size, tile_m, wavelength_m))
    rng = np.random.default_rng(seed)
    white = rng.standard_normal((size, size))
    f = np.fft.fftfreq(size, d=tile_m / size)          # cycles per metre
    fr = np.hypot(f[:, None], f[None, :])
    lo, hi = 1.0 / wavelength_m[1], 1.0 / wavelength_m[0]
    band = ((fr >= lo) & (fr <= hi)).astype(float) / np.maximum(fr, lo) ** 0.5   # a little more weight on coarse
    noise = np.real(np.fft.ifft2(np.fft.fft2(white) * band))
    noise /= max(noise.std(), 1e-12)
    mult = 1.0 + contrast * np.clip(noise / 2.5, -1.0, 1.0)
    img = np.clip(np.asarray(rgb, dtype=float)[None, None, :] * mult[:, :, None], 0.0, 1.0)
    return np.round(img * 255.0).astype(np.uint8)


def ground_uv(points_xy: Sequence[Sequence[float]], tile_m: float = TILE_M) -> np.ndarray:
    """Planar texture coordinates of ground points: world xy / tile size (the texture repeats)."""
    return np.asarray(points_xy, dtype=float)[:, :2] / float(tile_m)

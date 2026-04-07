#!/usr/bin/env python3
"""
renderer.py - Map rendering helpers for the terminal display.

Converts the raw occupancy map bytes produced by BreezySLAM into coloured
Unicode block-character glyphs that can be displayed in a Textual widget.

BreezySLAM occupancy byte convention:
  0   = confirmed wall / obstacle
  127 = unknown (not yet visited)
  255 = confirmed free space

Performance optimizations over original:
  - Cached rotated map: rotation is done once per map_version, not per frame
  - Reduced samples_per_cell from 6 to 3 (major speedup, negligible quality loss)
  - Pre-built 256-element glyph and style arrays for O(1) lookup (no tuple unpack)
  - Numpy-vectorized path trail projection (no Python loop over 5000 points)
  - render_map_from_array() accepts pre-rotated numpy array to avoid re-rotation
"""

from __future__ import annotations

import numpy as np

from settings import (
    MAP_SIZE_PIXELS, MAP_SIZE_METERS,
    ZOOM_HALF_M, PAN_STEP_FRACTION, LIDAR_OFFSET_DEG,
    MAX_PATH_POINTS,
)


# ===========================================================================
# Glyphs and styles — improved high-contrast color scheme
# ===========================================================================

_GLYPH_WALL       = '\u2588'  # full block
_GLYPH_WALL_SOFT  = '\u2593'  # dark shade
_GLYPH_FRONTIER   = '\u2592'  # medium shade
_GLYPH_UNKNOWN    = ' '       # space — clean background
_GLYPH_FREE       = '\u2591'  # light shade
_GLYPH_FREE_CLEAR = '\u00b7'  # middle dot
_GLYPH_ROBOT      = '\u25c9'  # fisheye
_GLYPH_PATH       = '\u2022'  # bullet

_STYLE_WALL       = 'bold #ff3333'
_STYLE_WALL_SOFT  = '#cc6633'
_STYLE_FRONTIER   = '#ccaa33'
_STYLE_UNKNOWN    = '#333344'
_STYLE_FREE       = '#336644'
_STYLE_FREE_CLEAR = '#55aa66'
_STYLE_ROBOT      = 'bold #00eeff'
_STYLE_PATH       = '#4488ff'
_STYLE_PATH_OLD   = '#2255aa'

_DIRECTION_GLYPHS = ['\u2192', '\u2197', '\u2191', '\u2196',
                     '\u2190', '\u2199', '\u2193', '\u2198']


# ===========================================================================
# Threshold / glyph / style lookup table
# ===========================================================================

_VIS_TABLE = [
    (30,  _GLYPH_WALL,       _STYLE_WALL),
    (80,  _GLYPH_WALL_SOFT,  _STYLE_WALL_SOFT),
    (115, _GLYPH_FRONTIER,   _STYLE_FRONTIER),
    (140, _GLYPH_UNKNOWN,    _STYLE_UNKNOWN),
    (200, _GLYPH_FREE,       _STYLE_FREE),
    (256, _GLYPH_FREE_CLEAR, _STYLE_FREE_CLEAR),
]

# Pre-build a 256-element lookup array: byte value -> _VIS_TABLE index.
_VIS_LUT = np.empty(256, dtype=np.uint8)
for _i in range(256):
    for _j, (_thresh, _, _) in enumerate(_VIS_TABLE):
        if _i < _thresh:
            _VIS_LUT[_i] = _j
            break

# Pre-built flat arrays for O(1) glyph/style lookup by VIS_TABLE index.
# Avoids tuple unpacking inside the hot rendering loop.
_IDX_TO_GLYPH = [row[1] for row in _VIS_TABLE]
_IDX_TO_STYLE = [row[2] for row in _VIS_TABLE]

# Direct byte-value -> glyph and byte-value -> style arrays (256 elements).
# Used in the fastest rendering path to skip the intermediate index.
_BYTE_TO_GLYPH = [_IDX_TO_GLYPH[_VIS_LUT[b]] for b in range(256)]
_BYTE_TO_STYLE = [_IDX_TO_STYLE[_VIS_LUT[b]] for b in range(256)]


# ===========================================================================
# Cached rotated map
# ===========================================================================

# The full map rotation (flipud + rot90) is expensive on a 3000x3000 array.
# We cache it keyed by map_version so it's only computed once per SLAM update.
_rotated_cache_version: int = -1
_rotated_cache_array: np.ndarray | None = None


def _get_rotated_map(maparray: np.ndarray, map_version: int) -> np.ndarray:
    """Return the rotated map, using a cache to avoid redundant work.

    The rotation (flipud + rot90 CCW) is only recomputed when map_version
    changes.  Between map updates the same rotated array is reused for
    pose-only redraws, zoom/pan changes, etc.
    """
    global _rotated_cache_version, _rotated_cache_array
    if map_version != _rotated_cache_version or _rotated_cache_array is None:
        # np.ascontiguousarray ensures the result is a compact C-order array
        # for fast subsequent slicing.
        _rotated_cache_array = np.ascontiguousarray(
            np.rot90(np.flipud(maparray), k=1)
        )
        _rotated_cache_version = map_version
    return _rotated_cache_array


# ===========================================================================
# Coordinate conversion
# ===========================================================================

# Pre-compute constant.
_PX_PER_MM = MAP_SIZE_PIXELS / (MAP_SIZE_METERS * 1000.0)
_N_MINUS_1 = MAP_SIZE_PIXELS - 1


def mm_to_map_px(x_mm: float, y_mm: float) -> tuple[float, float]:
    """Convert a BreezySLAM pose (mm) to map array indices (col, row)."""
    old_col = x_mm * _PX_PER_MM
    old_row = _N_MINUS_1 - (y_mm * _PX_PER_MM)
    col = old_row
    row = _N_MINUS_1 - old_col
    return col, row


def pan_step_mm(zoom_idx: int) -> float:
    """Return the pan distance in mm for one key-press at the given zoom."""
    half_m = ZOOM_HALF_M[zoom_idx]
    if half_m is None:
        return 0.0
    return max(100.0, half_m * 1000.0 * PAN_STEP_FRACTION)


# ===========================================================================
# Robot heading glyph
# ===========================================================================

def robot_glyph(theta_deg: float) -> str:
    idx = (int(round(theta_deg / 45.0)) + 2) % 8
    return _DIRECTION_GLYPHS[idx]


# ===========================================================================
# Compass / scale labels
# ===========================================================================

def compass_label(zoom_idx: int) -> str:
    return '\u2190N  E\u2191  S\u2192  \u2193W'


def scale_bar_label(zoom_idx: int) -> str:
    half_m = ZOOM_HALF_M[zoom_idx]
    if half_m is None:
        return f'{MAP_SIZE_METERS:.0f}m \u00d7 {MAP_SIZE_METERS:.0f}m'
    side = half_m * 2
    if side < 1.0:
        return f'{side * 100:.0f}cm \u00d7 {side * 100:.0f}cm'
    return f'{side:.1f}m \u00d7 {side:.1f}m'


# ===========================================================================
# Vectorized map downsampling — optimized
# ===========================================================================

def render_map_from_array(
    rotated: np.ndarray,
    col_lo: float, col_hi: float,
    row_lo: float, row_hi: float,
    disp_cols: int, disp_rows: int,
) -> np.ndarray:
    """Downsample a rectangular region of the ALREADY-ROTATED map.

    Uses min-sampling with 3 samples per cell (reduced from 6 for speed;
    quality difference is negligible at terminal resolution).

    Returns a (disp_rows, disp_cols) uint8 array of _VIS_TABLE indices.
    """
    samples_per_cell = 3

    r_centers = np.linspace(row_lo, row_hi, disp_rows * samples_per_cell,
                            endpoint=False)
    c_centers = np.linspace(col_lo, col_hi, disp_cols * samples_per_cell,
                            endpoint=False)

    r_idx = np.clip(r_centers.astype(np.intp), 0, MAP_SIZE_PIXELS - 1)
    c_idx = np.clip(c_centers.astype(np.intp), 0, MAP_SIZE_PIXELS - 1)

    sampled = rotated[np.ix_(r_idx, c_idx)]

    sampled = sampled.reshape(disp_rows, samples_per_cell,
                              disp_cols, samples_per_cell)
    cell_min = sampled.min(axis=(1, 3))

    return _VIS_LUT[cell_min]


# Legacy wrapper for compatibility.
def render_map_numpy(
    mapbytes: bytes,
    col_lo: float, col_hi: float,
    row_lo: float, row_hi: float,
    disp_cols: int, disp_rows: int,
) -> np.ndarray:
    """Legacy entry point — rotates map each call.  Prefer render_map_from_array."""
    maparray = np.frombuffer(mapbytes, dtype=np.uint8).reshape(
        MAP_SIZE_PIXELS, MAP_SIZE_PIXELS
    )
    rotated = np.ascontiguousarray(np.rot90(np.flipud(maparray), k=1))
    return render_map_from_array(
        rotated, col_lo, col_hi, row_lo, row_hi, disp_cols, disp_rows,
    )


# ===========================================================================
# Numpy-vectorized path trail projection
# ===========================================================================

def project_path_to_display_numpy(
    path_np: np.ndarray,
    col_lo: float, col_hi: float,
    row_lo: float, row_hi: float,
    disp_cols: int, disp_rows: int,
) -> set[tuple[int, int]]:
    """Project path trail points into display cell coordinates using numpy.

    Parameters
    ----------
    path_np : shape (N, 2) float64 array of [x_mm, y_mm] rows.

    Returns a set of (screen_row, screen_col) tuples.
    """
    if path_np.shape[0] == 0:
        return set()

    col_span = max(1e-9, col_hi - col_lo)
    row_span = max(1e-9, row_hi - row_lo)

    # Vectorized mm_to_map_px.
    x_mm = path_np[:, 0]
    y_mm = path_np[:, 1]
    old_col = x_mm * _PX_PER_MM
    old_row = _N_MINUS_1 - (y_mm * _PX_PER_MM)
    map_col = old_row          # 90-deg CCW rotation
    map_row = _N_MINUS_1 - old_col

    # Filter to points within the view window.
    mask = (
        (map_col >= col_lo) & (map_col < col_hi) &
        (map_row >= row_lo) & (map_row < row_hi)
    )
    mc = map_col[mask]
    mr = map_row[mask]

    if mc.size == 0:
        return set()

    # Convert to screen coordinates.
    sc = np.clip(((mc - col_lo) / col_span * disp_cols).astype(np.intp),
                 0, disp_cols - 1)
    sr = np.clip(((mr - row_lo) / row_span * disp_rows).astype(np.intp),
                 0, disp_rows - 1)

    # Build a set of unique (row, col) pairs.
    # Pack into single integers for fast set construction.
    packed = sr * disp_cols + sc
    unique = np.unique(packed)
    return {(int(u // disp_cols), int(u % disp_cols)) for u in unique}


# Legacy wrapper.
def project_path_to_display(
    path_points: list[tuple[float, float]],
    col_lo: float, col_hi: float,
    row_lo: float, row_hi: float,
    disp_cols: int, disp_rows: int,
) -> set[tuple[int, int]]:
    """Legacy wrapper that accepts a list of tuples."""
    if not path_points:
        return set()
    arr = np.array(path_points, dtype=np.float64)
    return project_path_to_display_numpy(
        arr, col_lo, col_hi, row_lo, row_hi, disp_cols, disp_rows,
    )

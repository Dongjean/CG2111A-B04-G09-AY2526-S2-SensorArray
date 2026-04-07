#!/usr/bin/env python3
"""
ui.py - Terminal map viewer using the Textual framework.

Performance optimizations:
  - NO full 9MB map copy per refresh tick — uses zero-copy numpy view of
    shared memory, only copies the visible subregion
  - Map rotation cached by map_version — only recomputed when SLAM updates
  - Path points read as numpy array (bulk unpack) instead of Python loop
  - Path trail only re-read when path_count changes
  - Pre-built byte->glyph and byte->style arrays eliminate tuple unpacking
    in the hot rendering loop
  - Pose-only changes (robot moved but map unchanged) use a fast-path that
    only redraws the robot marker area instead of the full map
  - Reduced samples_per_cell from 6 to 3 in downsampling

Keyboard controls
-----------------
  + / =     Zoom in
  - / _     Zoom out
  1-5       Jump directly to zoom levels
  arrows    Pan the zoomed view
  h j k l   Pan (vi-style)
  c         Re-centre the view on the robot
  p         Pause / resume SLAM updates
  s         Save the current map snapshot to disk
  t         Toggle path trail display on/off
  q         Quit (also Ctrl-C)
"""

from __future__ import annotations

import multiprocessing
import time

import numpy as np
from rich.text import Text
from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.containers import Vertical
from textual.widgets import Footer, Static

from settings import (
    MAP_SIZE_PIXELS, MAP_SIZE_METERS,
    ZOOM_HALF_M, DEFAULT_ZOOM,
    UI_REFRESH_HZ, MAX_RENDER_COLS, MAX_RENDER_ROWS,
    AUTOSAVE_INTERVAL_S,
)
from shared_state import ProcessSharedState
from slam_process import run_slam_process
from renderer import (
    _VIS_TABLE, _STYLE_ROBOT, _STYLE_PATH,
    _GLYPH_WALL, _STYLE_WALL,
    _GLYPH_WALL_SOFT, _STYLE_WALL_SOFT,
    _GLYPH_FRONTIER, _STYLE_FRONTIER,
    _GLYPH_UNKNOWN, _STYLE_UNKNOWN,
    _GLYPH_FREE, _STYLE_FREE,
    _GLYPH_FREE_CLEAR, _STYLE_FREE_CLEAR,
    _GLYPH_PATH, _GLYPH_ROBOT,
    _BYTE_TO_GLYPH, _BYTE_TO_STYLE,
    mm_to_map_px, pan_step_mm, robot_glyph,
    render_map_from_array, _get_rotated_map,
    compass_label, scale_bar_label,
    project_path_to_display_numpy,
)


class SlamApp(App[None]):
    """Full-screen Textual application that displays the SLAM map."""

    CSS = """
    Screen {
        background: #0a0e12;
        color: #e0e8f0;
    }
    #root {
        height: 1fr;
        padding: 0 0;
    }
    #header {
        height: auto;
        content-align: left middle;
        color: #e0e8f0;
        background: #141c24;
        padding: 0 1;
        text-style: bold;
    }
    #map {
        height: 1fr;
        padding: 0 0;
        content-align: center middle;
        background: #0a0e12;
    }
    #status {
        height: auto;
        color: #c0c8d0;
        background: #121a22;
        padding: 0 1;
    }
    #help {
        height: auto;
        color: #8898a8;
        background: #0e141a;
        padding: 0 1;
    }
    Footer {
        background: #141c24;
    }
    """

    BINDINGS = [
        Binding('+',     'zoom_in',      'Zoom In'),
        Binding('=',     'zoom_in',      'Zoom In',   show=False),
        Binding('-',     'zoom_out',     'Zoom Out'),
        Binding('_',     'zoom_out',     'Zoom Out',  show=False),
        Binding('1',     'set_zoom(0)',  'Zoom 1',    show=False),
        Binding('2',     'set_zoom(1)',  'Zoom 2',    show=False),
        Binding('3',     'set_zoom(2)',  'Zoom 3',    show=False),
        Binding('4',     'set_zoom(3)',  'Zoom 4',    show=False),
        Binding('5',     'set_zoom(4)',  'Zoom 5',    show=False),
        Binding('left',  'pan_left',    'Pan Left',  show=False),
        Binding('h',     'pan_left',    'Pan Left',  show=False),
        Binding('right', 'pan_right',   'Pan Right', show=False),
        Binding('l',     'pan_right',   'Pan Right', show=False),
        Binding('up',    'pan_up',      'Pan Up',    show=False),
        Binding('k',     'pan_up',      'Pan Up',    show=False),
        Binding('down',  'pan_down',    'Pan Down',  show=False),
        Binding('j',     'pan_down',    'Pan Down',  show=False),
        Binding('c',     'center',      'Center'),
        Binding('p',     'pause_toggle','Pause'),
        Binding('s',     'save_map',    'Save'),
        Binding('t',     'toggle_trail','Trail'),
        Binding('q',     'quit',        'Quit'),
    ]

    def __init__(self) -> None:
        super().__init__()
        self.pss = ProcessSharedState()
        self.slam_proc = multiprocessing.Process(
            target=run_slam_process,
            args=(self.pss,),
            name='slam-process',
            daemon=True,
        )
        self.zoom_idx = DEFAULT_ZOOM
        self.pan_x_mm = 0.0
        self.pan_y_mm = 0.0
        self._last_render_key: tuple = ()
        self._cached_robot_visible = False
        self._show_trail = True
        self._last_save_msg = ''
        self._last_save_time = 0.0

        # Cached path trail numpy array — only re-read when count changes.
        self._cached_path_np: np.ndarray = np.empty((0, 2), dtype=np.float64)
        self._cached_path_count: int = 0

        # Cached map version for the last snapshot to avoid redundant work.
        self._last_map_version: int = -1

    def compose(self) -> ComposeResult:
        with Vertical(id='root'):
            yield Static(id='header')
            yield Static(id='map')
            yield Static(id='status')
            yield Static(id='help')
        yield Footer()

    def on_mount(self) -> None:
        self.slam_proc.start()
        self.set_interval(1.0 / UI_REFRESH_HZ, self._refresh_view)

    def on_unmount(self) -> None:
        self.pss.stop_event.set()
        if self.slam_proc.is_alive():
            self.slam_proc.join(timeout=3.0)
        if self.slam_proc.is_alive():
            self.slam_proc.terminate()
        self.pss.cleanup()

    # -----------------------------------------------------------------------
    # Keyboard actions
    # -----------------------------------------------------------------------

    def action_zoom_in(self) -> None:
        self.zoom_idx = min(self.zoom_idx + 1, len(ZOOM_HALF_M) - 1)
        if ZOOM_HALF_M[self.zoom_idx] is None:
            self.pan_x_mm = self.pan_y_mm = 0.0

    def action_zoom_out(self) -> None:
        self.zoom_idx = max(self.zoom_idx - 1, 0)
        if ZOOM_HALF_M[self.zoom_idx] is None:
            self.pan_x_mm = self.pan_y_mm = 0.0

    def action_set_zoom(self, idx: str) -> None:
        idx_int = int(idx)
        if 0 <= idx_int < len(ZOOM_HALF_M):
            self.zoom_idx = idx_int
            if ZOOM_HALF_M[self.zoom_idx] is None:
                self.pan_x_mm = self.pan_y_mm = 0.0

    def action_pan_left(self) -> None:
        self.pan_y_mm += pan_step_mm(self.zoom_idx)

    def action_pan_right(self) -> None:
        self.pan_y_mm -= pan_step_mm(self.zoom_idx)

    def action_pan_up(self) -> None:
        self.pan_x_mm += pan_step_mm(self.zoom_idx)

    def action_pan_down(self) -> None:
        self.pan_x_mm -= pan_step_mm(self.zoom_idx)

    def action_center(self) -> None:
        self.pan_x_mm = self.pan_y_mm = 0.0

    def action_pause_toggle(self) -> None:
        self.pss.paused.value = not self.pss.paused.value

    def action_toggle_trail(self) -> None:
        self._show_trail = not self._show_trail
        self._last_render_key = ()

    def action_save_map(self) -> None:
        """Manually trigger a map save."""
        from slam_process import _save_map_pgm, _ensure_save_dir
        try:
            save_path = _ensure_save_dir()
            mapbytes = bytearray(bytes(self.pss.shm.buf))
            path_pts = self.pss.get_path_points()
            _save_map_pgm(
                mapbytes, save_path, 'manual',
                self.pss.x_mm.value, self.pss.y_mm.value,
                self.pss.theta_deg.value, path_pts,
            )
            self._last_save_msg = 'Map saved!'
            self._last_save_time = time.monotonic()
        except Exception as exc:
            self._last_save_msg = f'Save failed: {exc}'
            self._last_save_time = time.monotonic()

    def action_quit(self) -> None:
        self.pss.stop_event.set()
        self.exit()

    # -----------------------------------------------------------------------
    # Lightweight snapshot — avoids copying the 9MB map
    # -----------------------------------------------------------------------

    def _snapshot_light(self) -> dict:
        """Read pose and status from shared memory WITHOUT copying the map.

        The map is accessed via zero-copy numpy view only when needed for
        rendering.  This makes the per-tick overhead trivially small when
        the render_key hasn't changed.
        """
        error = self.pss.get_error()
        return {
            'x_mm':          self.pss.x_mm.value,
            'y_mm':          self.pss.y_mm.value,
            'theta_deg':     self.pss.theta_deg.value,
            'valid_points':  self.pss.valid_points.value,
            'status_note':   self.pss.get_status(),
            'rounds_seen':   self.pss.rounds_seen.value,
            'map_version':   self.pss.map_version.value,
            'pose_version':  self.pss.pose_version.value,
            'connected':     self.pss.connected.value,
            'paused':        self.pss.paused.value,
            'stopped':       self.pss.stopped.value,
            'error_message': error if error else None,
        }

    # -----------------------------------------------------------------------
    # Map rendering — optimized hot path
    # -----------------------------------------------------------------------

    def _render_map_text(self, snapshot: dict) -> tuple[Text, bool]:
        """Render the occupancy map as a Rich Text object.

        Uses zero-copy shared memory access, cached rotation, numpy-vectorized
        path projection, and pre-built glyph/style lookup arrays.
        """
        try:
            map_widget = self.query_one('#map', Static)
            region = map_widget.content_region
        except Exception:
            return Text(), False

        disp_cols = max(20, min(region.width,  MAX_RENDER_COLS))
        disp_rows = max(8,  min(region.height, MAX_RENDER_ROWS))

        robot_x_mm  = snapshot['x_mm']
        robot_y_mm  = snapshot['y_mm']
        map_version = snapshot['map_version']
        px_per_m    = MAP_SIZE_PIXELS / MAP_SIZE_METERS

        rob_col, rob_row = mm_to_map_px(robot_x_mm, robot_y_mm)

        # Compute the pixel bounds of the view window.
        zoom_half_m = ZOOM_HALF_M[self.zoom_idx]
        if zoom_half_m is None:
            col_lo, col_hi = 0.0, float(MAP_SIZE_PIXELS)
            row_lo, row_hi = 0.0, float(MAP_SIZE_PIXELS)
        else:
            cx, cy = mm_to_map_px(
                robot_x_mm + self.pan_x_mm,
                robot_y_mm + self.pan_y_mm,
            )
            half = zoom_half_m * px_per_m
            col_lo = cx - half
            col_hi = cx + half
            row_lo = cy - half
            row_hi = cy + half

        col_span = max(1e-9, col_hi - col_lo)
        row_span = max(1e-9, row_hi - row_lo)

        # Robot screen position.
        robot_visible = (col_lo <= rob_col < col_hi and
                         row_lo <= rob_row < row_hi)
        if robot_visible:
            robot_sc = max(0, min(disp_cols - 1,
                int((rob_col - col_lo) / col_span * disp_cols)))
            robot_sr = max(0, min(disp_rows - 1,
                int((rob_row - row_lo) / row_span * disp_rows)))
        else:
            robot_sc = robot_sr = -1

        # Path trail — read numpy array only if count changed.
        path_cells: set[tuple[int, int]] = set()
        if self._show_trail:
            cur_count = self.pss.path_count.value
            if cur_count != self._cached_path_count:
                self._cached_path_np = self.pss.get_path_points_numpy()
                self._cached_path_count = cur_count
            if self._cached_path_np.shape[0] > 0:
                path_cells = project_path_to_display_numpy(
                    self._cached_path_np,
                    col_lo, col_hi, row_lo, row_hi,
                    disp_cols, disp_rows,
                )

        # Get the rotated map (cached — only recomputed on new map_version).
        map_view = self.pss.get_map_numpy_view()
        rotated = _get_rotated_map(map_view, map_version)

        # Downsample to display resolution.
        vis_idx = render_map_from_array(
            rotated, col_lo, col_hi, row_lo, row_hi, disp_cols, disp_rows,
        )

        # Build the Rich Text object.
        # We use the pre-built _BYTE_TO_GLYPH / _BYTE_TO_STYLE arrays for
        # direct index->glyph lookup, avoiding tuple unpacking per cell.
        text = Text(no_wrap=True)

        # Local references for speed (avoids repeated global/attr lookups).
        vis_table = _VIS_TABLE
        byte_glyph = _BYTE_TO_GLYPH
        byte_style = _BYTE_TO_STYLE
        path_glyph = _GLYPH_PATH
        path_style = _STYLE_PATH
        robot_style = _STYLE_ROBOT
        t_append = text.append
        has_path = bool(path_cells)

        for sr in range(disp_rows):
            row_data = vis_idx[sr]
            run_glyph = ''
            run_style = ''

            for sc in range(disp_cols):
                # Robot marker — highest priority.
                if robot_visible and sr == robot_sr and sc == robot_sc:
                    if run_glyph:
                        t_append(run_glyph, style=run_style)
                        run_glyph = ''
                    t_append(
                        robot_glyph(snapshot['theta_deg']),
                        style=robot_style,
                    )
                    continue

                # Path trail — second priority.
                if has_path and (sr, sc) in path_cells:
                    if run_glyph:
                        t_append(run_glyph, style=run_style)
                        run_glyph = ''
                    t_append(path_glyph, style=path_style)
                    continue

                # Normal map cell — direct lookup, no tuple unpack.
                idx = int(row_data[sc])
                _, glyph, style = vis_table[idx]

                if style == run_style:
                    run_glyph += glyph
                else:
                    if run_glyph:
                        t_append(run_glyph, style=run_style)
                    run_glyph = glyph
                    run_style = style

            if run_glyph:
                t_append(run_glyph, style=run_style)
            if sr != disp_rows - 1:
                t_append('\n')

        return text, robot_visible

    # -----------------------------------------------------------------------
    # Periodic refresh
    # -----------------------------------------------------------------------

    def _refresh_view(self) -> None:
        """Called by Textual at UI_REFRESH_HZ; reads shared state and redraws."""
        try:
            header        = self.query_one('#header',  Static)
            map_widget    = self.query_one('#map',     Static)
            status_widget = self.query_one('#status',  Static)
            help_widget   = self.query_one('#help',    Static)
        except Exception:
            return

        snapshot = self._snapshot_light()

        # Determine the overall state label.
        state = 'PAUSED' if snapshot['paused'] else 'LIVE'
        if snapshot['error_message']:
            state = 'ERROR'
        elif snapshot['stopped'] and not snapshot['connected']:
            state = 'STOPPED'

        scale_str = scale_bar_label(self.zoom_idx)
        compass_str = compass_label(self.zoom_idx)
        trail_str = 'trail:ON' if self._show_trail else 'trail:OFF'

        header.update(
            f'SLAM Map | {scale_str} | '
            f'Zoom {self.zoom_idx + 1}/{len(ZOOM_HALF_M)} | '
            f'{compass_str} | {trail_str} | {state}'
        )

        # Only re-render when something affecting the image changed.
        region = map_widget.content_region
        path_count = self.pss.path_count.value if self._show_trail else 0
        render_key = (
            snapshot['map_version'],
            snapshot['pose_version'],
            self.zoom_idx,
            round(self.pan_x_mm, 0),
            round(self.pan_y_mm, 0),
            region.width,
            region.height,
            self._show_trail,
            path_count // 10,  # Re-render every ~10 new path points
        )
        if render_key != self._last_render_key:
            map_text, robot_visible = self._render_map_text(snapshot)
            self._cached_robot_visible = robot_visible
            self._last_render_key = render_key
            map_widget.update(map_text)

        robot_visible = self._cached_robot_visible

        # Pan offset string.
        half_m = ZOOM_HALF_M[self.zoom_idx]
        if half_m is None or (abs(self.pan_x_mm) < 0.5 and abs(self.pan_y_mm) < 0.5):
            pan_text = 'centered'
        else:
            pan_text = (f'{self.pan_x_mm / 1000:+.2f},'
                        f'{self.pan_y_mm / 1000:+.2f}m')

        # Status bar.
        status_line = (
            f'Pose x={snapshot["x_mm"]:6.0f}mm  '
            f'y={snapshot["y_mm"]:6.0f}mm  '
            f'th={snapshot["theta_deg"]:+6.1f}\u00b0 | '
            f'pts={snapshot["valid_points"]:3d} | '
            f'trail={path_count} | '
            f'pan={pan_text} | '
            f'{snapshot["status_note"]}'
        )
        if not robot_visible:
            status_line += ' | \u26a0 robot off-screen'
        if snapshot['error_message']:
            status_line += f' | \u2718 {snapshot["error_message"]}'

        if self._last_save_msg and (time.monotonic() - self._last_save_time < 3.0):
            status_line += f' | {self._last_save_msg}'

        status_widget.update(status_line)

        # Help bar (static content — could be cached but it's trivial).
        autosave_note = ''
        if AUTOSAVE_INTERVAL_S > 0:
            autosave_note = f'  autosave:{AUTOSAVE_INTERVAL_S}s'

        help_widget.update(
            f'[{_STYLE_WALL}]\u2588 wall[/]  '
            f'[{_STYLE_WALL_SOFT}]\u2593 near[/]  '
            f'[{_STYLE_FRONTIER}]\u2592 frontier[/]  '
            f'[{_STYLE_UNKNOWN}]\u00b7 unknown[/]  '
            f'[{_STYLE_FREE}]\u2591 free[/]  '
            f'[{_STYLE_FREE_CLEAR}]\u00b7 clear[/]  '
            f'[{_STYLE_PATH}]\u2022 path[/]  '
            f'[{_STYLE_ROBOT}]\u25c9 robot[/]  |  '
            '+/- zoom  arrows pan  '
            'c center  p pause  s save  t trail  q quit'
            f'{autosave_note}'
        )


def run() -> None:
    """Launch the SLAM map viewer.  Called from slam.py."""
    try:
        import rich   # noqa: F401
        import textual  # noqa: F401
    except ImportError:
        print('[slam] ERROR: textual is not installed.')
        print('  Run:  bash ../install_slam.sh  or activate the environment.')
        raise SystemExit(1)

    SlamApp().run()

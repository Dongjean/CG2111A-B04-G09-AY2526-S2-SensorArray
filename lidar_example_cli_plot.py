import shutil
import sys
import numpy as np
import time
from lidar.alex_lidar import lidarConnect, lidarDisconnect, lidarStatus, performSingleScan

# ==============================================================================
# GLOBAL CONFIGURATION
# ==============================================================================
PORT = "/dev/ttyUSB0"
BAUDRATE = 115200
GRID_WIDTH = 100  
GRID_HEIGHT = 60   
MAX_RANGE_MM = 2500 
CLI_ASPECT_RATIO = 2.2

DENSITY_CHARS = " ░▒▓█" 
AXIS_CHAR_H = "─"
AXIS_CHAR_V = "│"
AXIS_CHAR_CENTER = "┼"

# ==============================================================================
# TERMINAL UI UTILITIES
# ==============================================================================
ESC = "\033["
RESET = f"{ESC}0m"
CLEAR_DOWN = f"{ESC}J"
HIDE_CUR = f"{ESC}?25l"
SHOW_CUR = f"{ESC}?25h"

CLR_AXIS  = f"{ESC}2m"   
CLR_POINT = f"{ESC}32m"  
CLR_LABEL = f"{ESC}36m"  

def ui_hide_cursor():
    sys.stdout.write(HIDE_CUR)

def ui_show_cursor():
    sys.stdout.write(SHOW_CUR)

def ui_update_display(content_string, move_up_amount):
    """
    Moves cursor up by the exact number of newlines in the previous frame.
    """
    # \r resets to column 1, {move_up_amount}A moves up
    sys.stdout.write(f"\r{ESC}{move_up_amount}A{CLEAR_DOWN}")
    sys.stdout.write(content_string)
    sys.stdout.flush()

def ui_prepare_frame(frame_height=GRID_HEIGHT, frame_width=GRID_WIDTH):
    """
    Reserve terminal space for the LiDAR frame and hide the cursor for clean updates.
    Validates terminal dimensions before reserving space and returns the number of lines written.
    """
    term_cols, term_rows = shutil.get_terminal_size(fallback=(frame_width, frame_height + 2))
    if term_cols < frame_width or term_rows < frame_height:
        raise RuntimeError(
            f"Terminal too small for LiDAR grid. Requires at least {frame_width}x{frame_height}, "
            f"but detected {term_cols}x{term_rows}."
        )
    sys.stdout.write("\n" * frame_height)
    ui_hide_cursor()
    return frame_height

# ==============================================================================
# RENDERING LOGIC
# ==============================================================================
def points_to_grid(xs, ys, *, grid_width=GRID_WIDTH, grid_height=GRID_HEIGHT):
    """Convert Cartesian points into the discrete grid used by the CLI renderer."""

    # Initialize an empty grid. Each cell will count the number of points that fall into it.
    grid = np.zeros((grid_height, grid_width), dtype=int)
    
    # Determine the center of the grid in terms of row and column indices, and the scale factor to convert mm to grid cells.
    mid_row, mid_col = grid_height // 2, grid_width // 2

    # The scale factor converts from millimeters to grid cells. We want MAX_RANGE_MM to correspond to half the grid size (from center to edge).
    scale = MAX_RANGE_MM / (grid_height / 2)

    # Count points in each grid cell. We iterate through each point and put it into the appropriate cell in the grid.
    for x_mm, y_mm in zip(xs, ys):
        # Convert the (x_mm, y_mm) coordinates into grid indices (col, row). The center of the grid corresponds to (0, 0) in mm.
        # The aspect ratio is applied to the x_mm value to stretch it horizontally, since terminal characters are taller than they are wide.
        col = int((x_mm * CLI_ASPECT_RATIO) / scale + mid_col)
        row = int(y_mm / scale + mid_row)
        if 0 <= col < grid_width and 0 <= row < grid_height:
            grid[row, col] += 1
    return grid

def gridValue_to_char(count, max_idx=len(DENSITY_CHARS) - 1):
    """Convert a point count into its representative character based on density."""
    return DENSITY_CHARS[min(count, max_idx)]

def render_to_cli(grid):
    """Render the numeric grid into a CLI string by composing a temporary character grid."""
    grid_height, grid_width = grid.shape
    mid_row, mid_col = grid_height // 2, grid_width // 2
    max_idx = len(DENSITY_CHARS) - 1

    # Temporary grid that stores the final string for each cell.
    char_grid = [[" " for _ in range(grid_width)] for _ in range(grid_height)]

    # 1) Draw points first so later overlays can replace them.
    for r in range(grid_height):
        for c in range(grid_width):
            val = grid[r, c]
            if val > 0:
                char_grid[r][c] = f"{CLR_POINT}{gridValue_to_char(val, max_idx=max_idx)}{RESET}"

    # 2) Overlay axes so they always remain visible.
    axis_h = f"{CLR_AXIS}{AXIS_CHAR_H}{RESET}"
    axis_v = f"{CLR_AXIS}{AXIS_CHAR_V}{RESET}"
    axis_center = f"{CLR_AXIS}{AXIS_CHAR_CENTER}{RESET}"

    for c in range(grid_width):
        char_grid[mid_row][c] = axis_h
    for r in range(grid_height):
        char_grid[r][mid_col] = axis_v
    char_grid[mid_row][mid_col] = axis_center

    # 3) Overlay Y-axis labels adjacent to the vertical axis.
    def overlay_label(row_idx, text):
        start_col = min(mid_col + 1, grid_width)
        col = start_col
        for ch in text:
            if col >= grid_width:
                break
            char_grid[row_idx][col] = f"{CLR_LABEL}{ch}{RESET}"
            col += 1

    if grid_height > 0:
        overlay_label(grid_height - 1, f" +{MAX_RANGE_MM}mm")
        overlay_label(0, f" -{MAX_RANGE_MM}mm")

    # 4) Collapse the character grid into output lines (top row first).
    output_lines = []
    for r in reversed(range(grid_height)):
        output_lines.append("".join(char_grid[r]))

        # X-Labels line immediately under the x-axis
        if r == mid_row:
            l_lab, c_lab, r_lab = f"-{MAX_RANGE_MM}mm", "0mm", f"+{MAX_RANGE_MM}mm"
            gap1 = " " * max(0, mid_col - len(l_lab))
            gap2 = " " * max(0, grid_width - mid_col - len(c_lab) - len(r_lab))
            output_lines.append(f"{CLR_LABEL}{l_lab}{RESET}{gap1}{CLR_LABEL}{c_lab}{RESET}{gap2}{CLR_LABEL}{r_lab}{RESET}")

    # Join the lines. This creates exactly grid_height newlines.
    return "\n".join(output_lines)

# ==============================================================================
# MAIN EXECUTION
# ==============================================================================

def convert_to_cartesian(angles, distances):
    """
    Convert the scanAngles and scanDistances into X and Y coordinates
    Hint, Pythagoras' theorem can be used to convert polar to cartesian coordinates
    We've provided you with the value of PI
    """
    VAL_PI = np.pi

    Xs = []
    Ys = []
    for angle, distance in zip(angles, distances):
        # TODO: Fill in the code to convert the polar coordinates to cartesian coordinates
        # Hint, Pythagoras' theorem can be used to convert polar to cartesian coordinates
        rad_angle = angle * (VAL_PI / 180)
        cartesian_X = distance * np.cos(rad_angle)  # REPLACE ME with the correct formula
        cartesian_Y = distance * np.sin(rad_angle)  # REPLACE ME with the correct formula

        Xs.append(cartesian_X)
        Ys.append(cartesian_Y)

    return Xs, Ys



def plot_single_scan():
    """Renders a single frame and exits."""
    print("====== LiDAR Single Plot ======")
    lidar = lidarConnect(port=PORT, baudrate=BAUDRATE, wait=2)
    try:
        # Retrieve and print the LiDAR device information
        status = lidarStatus(lidar)
        
        print("====== Scanning ======")
        # Get a single scan using the library. This returns the scan data.
        scan_data = performSingleScan(lidar, status['typical_scan_mode'])
        
        # Convert the scan data polar coordinates into Cartesian coordinates (Xs and Ys) for plotting.
        xs, ys = convert_to_cartesian(scan_data[0], scan_data[1])
        
        # Convert continuous Cartesian coordinates into a discrete grid representation
        grid = points_to_grid(xs, ys)

        # Render the grid into a string and print it to the CLI
        print(render_to_cli(grid))
    finally:
        lidarDisconnect(lidar)
        print("\nSingle scan complete.")

def plot_live_scan():
    print("====== LiDAR Live Plot ======")
    lidar = lidarConnect(port=PORT, baudrate=BAUDRATE, wait=2)
    status = lidarStatus(lidar)
    mode = status['typical_scan_mode']
    print(f"Connected. Mode: {mode}\n")

    print("====== Scanning ======")
    # Reserve space for the CLI plot and hide the cursor.
    move_up_amount = ui_prepare_frame(GRID_HEIGHT)

    try:
        while True:
            scan_data = performSingleScan(lidar, mode)
            
            # Convert the scan data polar coordinates into Cartesian coordinates (Xs and Ys) for plotting.
            xs, ys = convert_to_cartesian(scan_data[0], scan_data[1])

            # Convert continuous Cartesian coordinates into a discrete grid representation
            grid = points_to_grid(xs, ys)

            # Render the grid into a string
            full_frame = render_to_cli(grid)

            # Update the CLI display with the new frame, moving the cursor up to overwrite the previous frame.
            ui_update_display(full_frame, move_up_amount)
            
            # Sleep briefly to allow for a smoother display and to prevent overwhelming the terminal with updates. Adjust as needed for performance.
            time.sleep(0.01)

    except KeyboardInterrupt:
        # Move to bottom of the scan area for clean exit
        sys.stdout.write("\n" * 2)
        print("Scan stopped by user.")
    finally:
        lidarDisconnect(lidar)
        ui_show_cursor()
        sys.stdout.flush()

if __name__ == "__main__":
    # Uncomment one of the following lines to run either the single scan plot or the live scan plot.
    plot_single_scan()
    # plot_live_scan()

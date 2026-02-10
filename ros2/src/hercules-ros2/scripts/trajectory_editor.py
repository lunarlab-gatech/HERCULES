#!/usr/bin/env python3
"""
trajectory_editor.py (with interactive adjusted-origin support + delete mode)

Adds:
 - Draws the map's current origin (0,0,0) and a user-selected adjusted origin.
 - Press 'o' to enter "set-origin" mode, then click on the map to choose the adjusted origin.
 - Press 'r' to clear the adjusted origin (back to None).
 - When an adjusted origin is set, newly added points are stored relative to that adjusted origin.
   (Existing points are not modified.)
 - Press 'd' to enter "delete-point" mode, then left-click near a point to delete it.

Usage example:
Run from the directory of the txt files e.g. /home/sgarimella34/multi-robot-coordination/trajectory_data/BEVP_customcity/
python3 /home/sgarimella34/multi-robot-coordination/HERCULES/ros2/src/hercules-ros2/scripts/trajectory_editor.py \
    --map /home/sgarimella34/multi-robot-coordination/trajectory_data/occupancy_grid_maps/customcity_0mAlt_OGM_0p5m.pgm \
    --traj Drone1_trajectory.txt Husky1_trajectory.txt
"""

import argparse
import os
import sys

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from matplotlib.widgets import CheckButtons


def _parse_yaml_for_map_params(map_img_path):
    base, _ = os.path.splitext(map_img_path)
    yaml_path = base + '.yaml'
    if not os.path.isfile(yaml_path):
        return None, None

    resolution = None
    origin = None
    with open(yaml_path, 'r') as f:
        for raw_line in f:
            line = raw_line.strip()
            if line.startswith('resolution:'):
                try:
                    resolution = float(line.split(':', 1)[1].strip())
                except ValueError:
                    pass
            elif line.startswith('origin:'):
                try:
                    bracket_start = line.find('[')
                    bracket_end = line.find(']')
                    if bracket_start != -1 and bracket_end != -1:
                        inside = line[bracket_start+1:bracket_end]
                        parts = [p.strip() for p in inside.split(',')]
                        if len(parts) >= 2:
                            ox = float(parts[0]); oy = float(parts[1])
                            origin = (ox, oy)
                except ValueError:
                    pass
    return origin, resolution


class Trajectory:
    def __init__(self, filename, color, marker):
        self.filename = filename
        self.color = color
        self.marker = marker
        self.points = self._load_from_file(filename)  # Nx4 array
        self.line = None
        self.scatter = None

    def _load_from_file(self, filename):
        data = []
        with open(filename, 'r') as f:
            for idx, line in enumerate(f):
                line = line.strip()
                if not line:
                    continue
                parts = line.split()
                if len(parts) != 4:
                    raise ValueError(f"Line {idx+1} in {filename} does not have 4 elements: {line}")
                x, y, z, t = map(float, parts)
                data.append([x, y, z, t])
        return np.array(data)

    def save_to_file(self):
        with open(self.filename, 'w') as f:
            for x, y, z, t in self.points:
                f.write(f"{x:.6f} {y:.6f} {z:.6f} {t:.6f}\n")


class TrajectoryEditor:
    def __init__(self, map_img_path, traj_files, cli_origin, cli_resolution):
        self.map_img_path = map_img_path
        self.traj_files = traj_files
        # Parse YAML or use CLI origin/resolution
        yaml_origin, yaml_resolution = _parse_yaml_for_map_params(self.map_img_path)
        if yaml_origin is not None:
            self.origin = yaml_origin
            print(f"Loaded origin from YAML: {self.origin}")
        else:
            self.origin = cli_origin
        if yaml_resolution is not None:
            self.resolution = yaml_resolution
            print(f"Loaded resolution from YAML: {self.resolution}")
        else:
            self.resolution = cli_resolution

        # Load and flip map image
        self.map_img = mpimg.imread(self.map_img_path)
        if self.map_img.dtype in (np.float32, np.float64):
            self.map_img = (self.map_img * 255).astype(np.uint8)
        self.map_img = np.flipud(self.map_img)

        # Compute map extents
        h, w = self.map_img.shape[:2]
        ox, oy = self.origin
        res = self.resolution
        self.extent = [ox, ox + w * res, oy, oy + h * res]

        # Prepare trajectories
        cmap = plt.cm.get_cmap('tab10', len(self.traj_files))
        self.trajectories = []
        for idx, tf in enumerate(self.traj_files):
            base = os.path.basename(tf)
            marker = '^' if base.lower().startswith('drone') else ('s' if base.lower().startswith('husky') else 'o')
            self.trajectories.append(Trajectory(tf, color=cmap(idx), marker=marker))

        # Interactive state
        self.selected_traj = None
        self.selected_pt_idx = None
        self.dragging = False
        self.offset = (0, 0)
        self.add_mode = False
        self.delete_mode = False
        self.current_traj_idx = 0

        # Adjusted-origin state
        # All trajectory coordinates are in the "internal" frame used by files and editor.
        # Display uses a simple swap: x_disp = y_internal, y_disp = x_internal.
        self.adjusted_origin_internal = None   # (x_int, y_int) or None
        self.set_origin_mode = False           # True when waiting for a click to set adjusted origin

        # Artists for origin markers (created on first draw)
        self.current_origin_artist = None
        self.adjusted_origin_artist = None

        # Set up figure and axes
        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        plt.subplots_adjust(left=0.1, right=0.75, top=0.9, bottom=0.1)
        self._draw_map_and_trajectories()
        self._create_toggle_buttons()

        # Connect event handlers
        self.fig.canvas.mpl_connect('pick_event',           self.on_pick)
        self.fig.canvas.mpl_connect('motion_notify_event', self.on_motion)
        self.fig.canvas.mpl_connect('button_release_event',self.on_release)
        self.fig.canvas.mpl_connect('button_press_event',  self.on_click)
        self.fig.canvas.mpl_connect('key_press_event',     self.on_key)

        # Instructions
        print("=== Trajectory Editor ===")
        print(" - Use the checkboxes on the right to show/hide individual trajectories.")
        print(" - Click and drag trajectory points to move them.")
        print(" - Press 'a' to toggle add-point mode.")
        print(f" - Press 1–{len(self.trajectories)} to select trajectory for new points. (Currently 1)")
        print(" - Press 'o' to set the adjusted origin (click on map after pressing 'o').")
        print(" - Press 'r' to clear the adjusted origin.")
        print(" - With an adjusted origin set, NEWLY ADDED points are stored relative to it.")
        print(" - Press 'd' to toggle delete-point mode; in this mode left-click near a point to delete it.")
        print(" - Press 's' to save all modified trajectories.")
        print(" - Press 'q' to quit without saving.")
        print("=========================")

    # ----- Coordinate helpers -----

    @staticmethod
    def internal_to_display(x_int, y_int):
        """Map internal (file/editor) coordinates to display coordinates."""
        # From original code's effective transform: x_disp = y_int, y_disp = x_int
        return y_int, x_int

    @staticmethod
    def display_to_internal(x_disp, y_disp):
        """Map display coords back to internal coords."""
        return y_disp, x_disp

    # ----- Drawing -----

    def _draw_map_and_trajectories(self):
        self.ax.clear()
        self.ax.imshow(self.map_img, origin='lower', extent=self.extent)
        self.ax.set_xlim(self.extent[0], self.extent[1])
        self.ax.set_ylim(self.extent[2], self.extent[3])
        self.ax.set_aspect('equal')
        self.ax.set_title("Occupancy Grid (Flipped) with Transformed Trajectories")

        # Draw all trajectories
        for traj in self.trajectories:
            pts = traj.points[:, :2]
            # Effective display mapping (see helpers): xs, ys = y, x
            xs, ys = pts[:,1], pts[:,0]
            traj.line = self.ax.plot(xs, ys, '-', color=traj.color, linewidth=1.5, alpha=0.8)[0]
            traj.scatter = self.ax.scatter(xs, ys, s=50, color=traj.color,
                                           marker=traj.marker, edgecolors='black', picker=5)

        # Draw current origin (0,0) and adjusted origin if set
        self._draw_origins()

        # Legend
        labels = [os.path.basename(t.filename).replace('_trajectory.txt','') for t in self.trajectories]
        legend_handles = [t.scatter for t in self.trajectories]

        # Add origin markers to legend (small proxy artists)
        proxy_current, = self.ax.plot([], [], marker='x', linestyle='None', markersize=8, color='black',
                                      label='Current origin (0,0)')
        legend_handles.append(proxy_current)
        if self.adjusted_origin_internal is not None:
            proxy_adjusted, = self.ax.plot([], [], marker='o', linestyle='None', markersize=8, fillstyle='none',
                                           color='red', label='Adjusted origin')
            legend_handles.append(proxy_adjusted)

        legend_labels = labels + ['Current origin (0,0)'] + (['Adjusted origin'] if self.adjusted_origin_internal is not None else [])
        self.ax.legend(legend_handles, legend_labels, loc='upper left', bbox_to_anchor=(0.01,0.99))

        self.fig.canvas.draw_idle()

    def _draw_origins(self):
        # Current origin is fixed at internal (0,0)
        x_disp_curr, y_disp_curr = self.internal_to_display(0.0, 0.0)
        # Draw/refresh current origin marker
        if self.current_origin_artist is None:
            self.current_origin_artist = self.ax.scatter([x_disp_curr], [y_disp_curr],
                                                         marker='x', s=80, color='black', linewidths=2, zorder=5)
            self.ax.annotate("Current origin (0,0)", (x_disp_curr, y_disp_curr),
                             textcoords="offset points", xytext=(8, 8), ha='left', color='black',
                             fontsize=9, zorder=6)
        else:
            self.current_origin_artist.set_offsets(np.c_[[x_disp_curr], [y_disp_curr]])

        # Adjusted origin marker
        if self.adjusted_origin_internal is not None:
            ax_int, ay_int = self.adjusted_origin_internal
            x_disp_adj, y_disp_adj = self.internal_to_display(ax_int, ay_int)
            if self.adjusted_origin_artist is None:
                self.adjusted_origin_artist = self.ax.scatter([x_disp_adj], [y_disp_adj],
                                                              marker='o', s=90, facecolors='none', edgecolors='red',
                                                              linewidths=2, zorder=5)
                self.ax.annotate("Adjusted origin", (x_disp_adj, y_disp_adj),
                                 textcoords="offset points", xytext=(8, 8), ha='left', color='red',
                                 fontsize=9, zorder=6)
            else:
                self.adjusted_origin_artist.set_offsets(np.c_[[x_disp_adj], [y_disp_adj]])
        else:
            # If cleared, remove the artist if it exists
            if self.adjusted_origin_artist is not None:
                self.adjusted_origin_artist.remove()
                self.adjusted_origin_artist = None

    # ----- Visibility toggles -----

    def _create_toggle_buttons(self):
        axbox = self.fig.add_axes([0.80, 0.1, 0.15, 0.8])
        labels = [os.path.basename(t.filename).replace('_trajectory.txt','') for t in self.trajectories]
        visibility = [True] * len(labels)
        self.check = CheckButtons(axbox, labels, visibility)
        self.check.on_clicked(self._toggle_visibility)

    def _toggle_visibility(self, label):
        for traj in self.trajectories:
            name = os.path.basename(traj.filename).replace('_trajectory.txt','')
            if name == label:
                vis = not traj.line.get_visible()
                traj.line.set_visible(vis)
                traj.scatter.set_visible(vis)
                break
        self.fig.canvas.draw_idle()

    # ----- Interactions -----

    def on_pick(self, event):
        for i, traj in enumerate(self.trajectories):
            if event.artist == traj.scatter:
                ind = event.ind
                if not len(ind):
                    return
                self.selected_traj   = i
                self.selected_pt_idx = ind[0]
                self.dragging = True
                x_c, y_c = event.mouseevent.xdata, event.mouseevent.ydata
                # Internal -> display is swap; so display coords of the selected point:
                x_o, y_o = traj.points[self.selected_pt_idx,0], traj.points[self.selected_pt_idx,1]
                disp_x, disp_y = y_o, x_o
                self.offset = (disp_x - x_c, disp_y - y_c)
                return

    def on_motion(self, event):
        if not self.dragging or self.selected_traj is None:
            return
        if event.xdata is None or event.ydata is None:
            return
        new_disp_x = event.xdata + self.offset[0]
        new_disp_y = event.ydata + self.offset[1]
        # display->internal: (x_int, y_int) = (y_disp, x_disp)
        x_new = new_disp_y
        y_new = new_disp_x
        traj = self.trajectories[self.selected_traj]
        traj.points[self.selected_pt_idx,0] = x_new
        traj.points[self.selected_pt_idx,1] = y_new
        self._update_plot(traj)
        self.fig.canvas.draw_idle()

    def on_release(self, event):
        if self.dragging:
            self.dragging = False
            self.selected_traj = None
            self.selected_pt_idx = None
            self.offset = (0,0)

    def on_key(self, event):
        if event.key == 'a':
            self.add_mode = not self.add_mode
            if self.add_mode and self.delete_mode:
                self.delete_mode = False
                print("Delete-point mode OFF.")
            print(f"Add-point mode {'ON' if self.add_mode else 'OFF'}")
        elif event.key == 'd':
            self.delete_mode = not self.delete_mode
            if self.delete_mode and self.add_mode:
                self.add_mode = False
                print("Add-point mode OFF.")
            print(f"Delete-point mode {'ON' if self.delete_mode else 'OFF'}")
            if self.delete_mode:
                print("Left-click near a point to delete it.")
        elif event.key in [str(i+1) for i in range(len(self.trajectories))]:
            idx = int(event.key)-1
            self.current_traj_idx = idx
            name = os.path.basename(self.traj_files[idx])
            print(f"Selected trajectory for adding: {event.key} ({name})")
        elif event.key == 'o':
            # Enter set-origin mode: next left click sets the adjusted origin
            self.set_origin_mode = True
            print("Set-origin mode ON: click on the map to choose the adjusted origin.")
        elif event.key == 'r':
            # Reset/clear adjusted origin
            self.adjusted_origin_internal = None
            self.set_origin_mode = False
            print("Adjusted origin CLEARED.")
            # Redraw origin markers/legend
            self._draw_map_and_trajectories()
        elif event.key == 's':
            for traj in self.trajectories:
                traj.save_to_file()
                print(f"Saved: {traj.filename}")
        elif event.key == 'q':
            print("Quitting without additional saves.")
            plt.close(self.fig)

    def on_click(self, event):
        # Ignore clicks outside axes
        if event.inaxes != self.ax:
            return

        # Handle set-origin mode first
        if self.set_origin_mode and event.button == 1:
            # Convert clicked display coords to internal coords
            x_int, y_int = self.display_to_internal(event.xdata, event.ydata)
            self.adjusted_origin_internal = (x_int, y_int)
            self.set_origin_mode = False
            print(f"Adjusted origin set at internal coords: ({x_int:.3f}, {y_int:.3f})")
            # Redraw to show the adjusted origin marker and legend entry
            self._draw_map_and_trajectories()
            return

        # Handle delete-point mode
        if self.delete_mode and event.button == 1:
            self._delete_nearest_point(event.xdata, event.ydata)
            return

        # Handle add-point mode
        if not self.add_mode or event.button != 1:
            return

        # Map display click -> internal click
        click_x_int, click_y_int = self.display_to_internal(event.xdata, event.ydata)

        # If adjusted origin exists, store NEW point relative to it
        if self.adjusted_origin_internal is not None:
            ox_int, oy_int = self.adjusted_origin_internal
            x_store = click_x_int - ox_int
            y_store = click_y_int - oy_int
        else:
            x_store = click_x_int
            y_store = click_y_int

        traj = self.trajectories[self.current_traj_idx]
        last_z = traj.points[-1,2]
        last_t = traj.points[-1,3]
        new_point = [x_store, y_store, last_z, last_t+1.0]
        traj.points = np.vstack([traj.points, new_point])
        self._update_plot(traj)
        self.fig.canvas.draw_idle()

    def _delete_nearest_point(self, x_disp, y_disp):
        """Delete the nearest trajectory point to the display click, within a threshold."""
        min_dist2 = None
        best_traj_idx = None
        best_pt_idx = None

        for ti, traj in enumerate(self.trajectories):
            if traj.points.shape[0] == 0:
                continue
            pts = traj.points[:, :2]
            xs = pts[:, 1]  # internal->display
            ys = pts[:, 0]
            dx = xs - x_disp
            dy = ys - y_disp
            dist2 = dx*dx + dy*dy
            local_idx = np.argmin(dist2)
            local_min = dist2[local_idx]
            if min_dist2 is None or local_min < min_dist2:
                min_dist2 = local_min
                best_traj_idx = ti
                best_pt_idx = local_idx

        if min_dist2 is None:
            print("No points available to delete.")
            return

        # Threshold radius in world units (squared); tweak multiplier if needed
        threshold = (self.resolution * 5.0)**2
        if min_dist2 > threshold:
            print("No point close enough to delete (click closer).")
            return

        traj = self.trajectories[best_traj_idx]
        if traj.points.shape[0] <= 1:
            print(f"Refusing to delete last remaining point in {traj.filename}")
            return

        deleted = traj.points[best_pt_idx].copy()
        traj.points = np.delete(traj.points, best_pt_idx, axis=0)
        self._update_plot(traj)
        print(f"Deleted point {best_pt_idx} from {traj.filename}: x={deleted[0]:.3f}, y={deleted[1]:.3f}")
        self.fig.canvas.draw_idle()

    def _update_plot(self, traj):
        pts = traj.points[:,:2]
        xs, ys = pts[:,1], pts[:,0]  # internal->display
        traj.line.set_data(xs, ys)
        traj.scatter.set_offsets(np.c_[xs, ys])

    def run(self):
        plt.show()


def main():
    parser = argparse.ArgumentParser(description="Interactive Trajectory Editor with Add-Point Mode, Selection, Adjusted Origin, and Delete Mode")
    parser.add_argument('--map', required=True, help="Path to occupancy grid map (PNG or PGM).")
    parser.add_argument('--traj', nargs='+', required=True, help="One or more trajectory text files.")
    parser.add_argument('--origin', nargs=2, type=float, default=[0.0, 0.0], metavar=('OX','OY'),
                        help="Map origin if no YAML found [default: 0 0].")
    parser.add_argument('--resolution', type=float, default=1.0,
                        help="Map resolution if no YAML found [default: 1.0].")
    args = parser.parse_args()
    if not os.path.isfile(args.map):
        print(f"Error: Map image file not found: {args.map}")
        sys.exit(1)
    for tf in args.traj:
        if not os.path.isfile(tf):
            print(f"Error: Trajectory file not found: {tf}")
            sys.exit(1)
    cli_origin = (args.origin[0], args.origin[1])
    cli_resolution = args.resolution
    editor = TrajectoryEditor(args.map, args.traj, cli_origin, cli_resolution)
    editor.run()


if __name__ == "__main__":
    main()

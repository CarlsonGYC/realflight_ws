import argparse
import signal
import sys
from pathlib import Path
from typing import Dict, List, Tuple

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.widgets import Slider
from mpl_toolkits.mplot3d import Axes3D, proj3d  # noqa: F401


def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully."""
    print("\nExiting...")
    plt.close("all")
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)


def find_traj_files(base_dir: Path, use_raw: bool) -> List[Tuple[int, Path]]:
    """Return sorted list of (drone_id, csv_path) for available trajectories."""
    pattern = "drone_*_traj_raw_20hz.csv" if use_raw else "drone_*_traj_smoothed_100hz.csv"
    files = []
    for csv_path in base_dir.glob(pattern):
        try:
            drone_id = int(csv_path.name.split("_")[1])
        except (IndexError, ValueError):
            continue
        files.append((drone_id, csv_path))
    return sorted(files, key=lambda x: x[0])


def load_xyz(csv_path: Path) -> Dict[str, np.ndarray]:
    """Load time and position columns from a trajectory CSV."""
    if not csv_path.exists():
        raise FileNotFoundError(f"CSV not found: {csv_path}")
    df = pd.read_csv(csv_path, sep=",")
    return {
        "time": df["time"].to_numpy(),
        "x": df["x"].to_numpy(),
        "y": df["y"].to_numpy(),
        "z": df["z"].to_numpy(),
    }


def prepare_trajectory(traj: Dict[str, np.ndarray]) -> Dict[str, np.ndarray]:
    """Sort trajectory by time to keep interpolation stable."""
    if traj["time"].size == 0:
        raise ValueError("Trajectory contains no samples.")
    order = np.argsort(traj["time"])
    if not np.all(order == np.arange(traj["time"].size)):
        traj = {key: traj[key][order] for key in traj}
    return traj


def set_equal_aspect_3d(ax, xs: List[np.ndarray], ys: List[np.ndarray], zs: List[np.ndarray]):
    """Force 3D axes to share the same scale."""
    all_x = np.concatenate(xs)
    all_y = np.concatenate(ys)
    all_z = np.concatenate(zs)
    x_min, x_max = np.min(all_x), np.max(all_x)
    y_min, y_max = np.min(all_y), np.max(all_y)
    z_min, z_max = np.min(all_z), np.max(all_z)
    max_range = max(x_max - x_min, y_max - y_min, z_max - z_min)
    half = max_range / 2.0
    x_mid = (x_max + x_min) / 2.0
    y_mid = (y_max + y_min) / 2.0
    z_mid = (z_max + z_min) / 2.0
    ax.set_xlim(x_mid - half, x_mid + half)
    ax.set_ylim(y_mid - half, y_mid + half)
    ax.set_zlim(z_mid - half, z_mid + half)


def data_radius_to_points(ax, radius_m: float) -> float:
    """Convert a data-space radius to points based on current axes scaling."""
    x0, x1 = ax.get_xlim()
    y0, y1 = ax.get_ylim()
    x_mid = 0.5 * (x0 + x1)
    y_mid = 0.5 * (y0 + y1)
    p0 = ax.transData.transform((x_mid, y_mid))
    p1 = ax.transData.transform((x_mid + radius_m, y_mid))
    pixel_radius = np.hypot(*(p1 - p0))
    return pixel_radius * 72.0 / ax.figure.dpi


def data_radius_to_points_3d(ax, radius_m: float) -> float:
    """Approximate a 3D data-space radius in points using the current projection."""
    x0, x1 = ax.get_xlim()
    y0, y1 = ax.get_ylim()
    z0, z1 = ax.get_zlim()
    x_mid = 0.5 * (x0 + x1)
    y_mid = 0.5 * (y0 + y1)
    z_mid = 0.5 * (z0 + z1)
    x_proj, y_proj, _ = proj3d.proj_transform(x_mid, y_mid, z_mid, ax.get_proj())
    x_proj_r, y_proj_r, _ = proj3d.proj_transform(
        x_mid + radius_m,
        y_mid,
        z_mid,
        ax.get_proj(),
    )
    p0 = ax.transData.transform((x_proj, y_proj))
    p1 = ax.transData.transform((x_proj_r, y_proj_r))
    pixel_radius = np.hypot(*(p1 - p0))
    return pixel_radius * 72.0 / ax.figure.dpi


def plot_trajectories(
    trajectories: List[Tuple[int, Dict[str, np.ndarray]]],
    title_suffix: str,
):
    """Plot 3D and XY projections for all drones with an interactive time slider."""
    fig = plt.figure(figsize=(12, 6))
    fig.subplots_adjust(bottom=0.18, wspace=0.25)
    ax3d = fig.add_subplot(1, 2, 1, projection="3d")
    ax_xy = fig.add_subplot(1, 2, 2)

    xs, ys, zs = [], [], []
    colors = plt.cm.tab10(np.linspace(0, 1, max(len(trajectories), 1)))
    marker_entries = []
    for (drone_id, traj), color in zip(trajectories, colors):
        ax3d.plot(traj["x"], traj["y"], traj["z"], label=f"drone {drone_id}", color=color)
        ax_xy.plot(traj["x"], traj["y"], label=f"drone {drone_id}", color=color)
        xs.append(traj["x"])
        ys.append(traj["y"])
        zs.append(traj["z"])

        dot3d, = ax3d.plot(
            [np.nan],
            [np.nan],
            [np.nan],
            marker="o",
            markersize=1,
            linestyle="None",
            color=color,
        )
        dot2d, = ax_xy.plot(
            [np.nan],
            [np.nan],
            marker="o",
            markersize=1,
            linestyle="None",
            color=color,
        )

        marker_entries.append(
            {
                "time": traj["time"],
                "x": traj["x"],
                "y": traj["y"],
                "z": traj["z"],
                "t_min": traj["time"][0],
                "t_max": traj["time"][-1],
                "dot3d": dot3d,
                "dot2d": dot2d,
            }
        )

    ax3d.set_title(f"3D Trajectories ({title_suffix})")
    ax3d.set_xlabel("X [m]")
    ax3d.set_ylabel("Y [m]")
    ax3d.set_zlabel("Z [m]")
    set_equal_aspect_3d(ax3d, xs, ys, zs)
    ax3d.legend()
    ax3d.grid(True)

    ax_xy.set_title(f"XY Projection ({title_suffix})")
    ax_xy.set_xlabel("X [m]")
    ax_xy.set_ylabel("Y [m]")
    ax_xy.set_aspect("equal", adjustable="datalim")
    ax_xy.legend(ncol=2, fontsize="small")
    ax_xy.grid(True)

    fig.canvas.draw()
    marker_radius_m = 0.125
    marker_size_2d = 2.0 * data_radius_to_points(ax_xy, marker_radius_m)
    marker_size_3d = 2.0 * data_radius_to_points_3d(ax3d, marker_radius_m)
    for entry in marker_entries:
        entry["dot2d"].set_markersize(marker_size_2d)
        entry["dot3d"].set_markersize(marker_size_3d)

    all_times = np.concatenate([traj["time"] for _, traj in trajectories])
    t_min = float(np.min(all_times))
    t_max = float(np.max(all_times))

    ax_slider = fig.add_axes([0.15, 0.08, 0.7, 0.03])
    slider = Slider(
        ax=ax_slider,
        label="time [s]",
        valmin=t_min,
        valmax=t_max,
        valinit=t_min,
        valfmt="%.2f",
    )
    time_text = fig.text(0.5, 0.02, f"t = {t_min:.2f} s", ha="center", va="center")

    def update_time(val):
        t = slider.val
        for entry in marker_entries:
            if t < entry["t_min"] or t > entry["t_max"]:
                x = y = z = np.nan
            else:
                x = float(np.interp(t, entry["time"], entry["x"]))
                y = float(np.interp(t, entry["time"], entry["y"]))
                z = float(np.interp(t, entry["time"], entry["z"]))
            entry["dot3d"].set_data_3d([x], [y], [z])
            entry["dot2d"].set_data([x], [y])
        time_text.set_text(f"t = {t:.2f} s")
        fig.canvas.draw_idle()

    slider.on_changed(update_time)
    update_time(t_min)

    plt.show(block=True)


def main():
    parser = argparse.ArgumentParser(
        description="Plot trajectories for all drones (3D + XY views with a time slider)."
    )
    parser.add_argument(
        "--base-dir",
        type=Path,
        default=Path("data/3drone_trajectories_new"),
        help="Directory containing drone_*_traj_*.csv files",
    )
    parser.add_argument(
        "--raw",
        action="store_true",
        help="Use raw 20 Hz files instead of smoothed 100 Hz files",
    )
    args = parser.parse_args()

    traj_files = find_traj_files(args.base_dir, use_raw=args.raw)
    if not traj_files:
        print(f"No trajectory CSVs found in {args.base_dir}")
        sys.exit(1)

    trajectories: List[Tuple[int, Dict[str, np.ndarray]]] = []
    for drone_id, csv_path in traj_files:
        traj = prepare_trajectory(load_xyz(csv_path))
        trajectories.append((drone_id, traj))
        print(f"Loaded drone {drone_id} from {csv_path}")

    title_suffix = "raw 20 Hz" if args.raw else "smoothed 100 Hz"
    print("Displaying plots. Press Ctrl+C to exit or close all plot windows.")
    plot_trajectories(trajectories, title_suffix)


if __name__ == "__main__":
    main()

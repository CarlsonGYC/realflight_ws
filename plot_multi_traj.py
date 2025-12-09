import argparse
import signal
import sys
from pathlib import Path
from typing import Dict, List, Tuple

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401


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


def plot_trajectories(
    trajectories: List[Tuple[int, Dict[str, np.ndarray]]],
    title_suffix: str,
):
    """Plot 3D and XY projections for all drones."""
    fig3d = plt.figure(figsize=(8, 6))
    ax3d = fig3d.add_subplot(111, projection="3d")
    xs, ys, zs = [], [], []
    for drone_id, traj in trajectories:
        ax3d.plot(traj["x"], traj["y"], traj["z"], label=f"drone {drone_id}")
        xs.append(traj["x"])
        ys.append(traj["y"])
        zs.append(traj["z"])
    ax3d.set_title(f"3D Trajectories ({title_suffix})")
    ax3d.set_xlabel("X [m]")
    ax3d.set_ylabel("Y [m]")
    ax3d.set_zlabel("Z [m]")
    set_equal_aspect_3d(ax3d, xs, ys, zs)
    ax3d.legend()
    ax3d.grid(True)

    fig_xy, ax_xy = plt.subplots(figsize=(8, 6))
    for drone_id, traj in trajectories:
        ax_xy.plot(traj["x"], traj["y"], label=f"drone {drone_id}")
        ax_xy.plot(traj["x"][0], traj["y"][0], "o", markersize=4, label=f"start {drone_id}")
        ax_xy.plot(traj["x"][-1], traj["y"][-1], "x", markersize=6, label=f"end {drone_id}")
    ax_xy.set_title(f"XY Projection ({title_suffix})")
    ax_xy.set_xlabel("X [m]")
    ax_xy.set_ylabel("Y [m]")
    ax_xy.set_aspect("equal", adjustable="datalim")
    ax_xy.legend(ncol=2, fontsize="small")
    ax_xy.grid(True)

    plt.tight_layout()
    plt.show(block=True)


def main():
    parser = argparse.ArgumentParser(
        description="Plot trajectories for all drones (3D with equal scale and XY projection)."
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
        traj = load_xyz(csv_path)
        trajectories.append((drone_id, traj))
        print(f"Loaded drone {drone_id} from {csv_path}")

    title_suffix = "raw 20 Hz" if args.raw else "smoothed 100 Hz"
    print("Displaying plots. Press Ctrl+C to exit or close all plot windows.")
    plot_trajectories(trajectories, title_suffix)


if __name__ == "__main__":
    main()

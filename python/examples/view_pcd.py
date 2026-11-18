"""View .pcd files recorded by record_pcd_tri.py (top-down view, metres).

Usage:
    python view_pcd.py scans                   # step through every .pcd in a folder
    python view_pcd.py scans/xxx_0003.pcd      # view one file
    python view_pcd.py scans --overlay         # draw all scans on top of each other

Keys: Right/Left arrow = next/previous scan, Q = quit.
Supports ASCII PCD files with x and y fields (binary PCD is not supported).
"""
import argparse
import glob
import os

import numpy as np
import matplotlib.pyplot as plt


def read_pcd(path):
    with open(path) as f:
        fields = None
        for line in f:
            key = line.split(" ", 1)[0]
            if key == "FIELDS":
                fields = line.split()[1:]
            elif key == "DATA":
                if line.split()[1] != "ascii":
                    raise ValueError(f"{path}: only ASCII PCD is supported")
                break
        data = np.loadtxt(f, ndmin=2)
    if fields is None or "x" not in fields or "y" not in fields:
        raise ValueError(f"{path}: missing x/y fields")
    return data[:, fields.index("x")], data[:, fields.index("y")]


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("path", nargs="?", default="scans", help="a .pcd file or a folder of them")
    ap.add_argument("--overlay", action="store_true", help="draw all scans together")
    args = ap.parse_args()

    if os.path.isdir(args.path):
        files = sorted(glob.glob(os.path.join(args.path, "*.pcd")))
    else:
        files = [args.path]
    if not files:
        print(f"No .pcd files found in {args.path}")
        return
    clouds = [read_pcd(f) for f in files]

    lim = max(max(np.abs(x).max(initial=0), np.abs(y).max(initial=0)) for x, y in clouds) * 1.05 or 1.0
    fig, ax = plt.subplots(figsize=(7, 7))
    fig.canvas.manager.set_window_title("PCD Viewer")

    def setup_axes(title):
        ax.clear()
        ax.set_aspect("equal")
        ax.set_xlim(-lim, lim)
        ax.set_ylim(-lim, lim)
        ax.grid(True, alpha=0.3)
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.plot(0, 0, "r^", markersize=10, label="lidar")
        ax.set_title(title)

    if args.overlay:
        setup_axes(f"{len(files)} scans overlaid")
        for x, y in clouds:
            ax.scatter(x, y, s=2, alpha=0.3, color="tab:blue")
        plt.show()
        return

    idx = [0]

    def draw():
        x, y = clouds[idx[0]]
        setup_axes(f"{os.path.basename(files[idx[0]])}  ({idx[0] + 1}/{len(files)}, {len(x)} points)")
        ax.scatter(x, y, s=4, c=np.hypot(x, y), cmap="viridis_r")
        fig.canvas.draw_idle()

    def on_key(event):
        if event.key == "right":
            idx[0] = (idx[0] + 1) % len(files)
        elif event.key == "left":
            idx[0] = (idx[0] - 1) % len(files)
        else:
            return
        draw()

    fig.canvas.mpl_connect("key_press_event", on_key)
    draw()
    plt.show()


if __name__ == "__main__":
    main()

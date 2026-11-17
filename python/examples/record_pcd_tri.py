"""Record scans from a single-channel triangle lidar (X2 / X2L style) to .pcd files.

Each scan (one full rotation) is saved as its own ASCII PCD file with fields
x y z intensity (metres; z is always 0 for a 2D lidar).

Usage:
    python record_pcd_tri.py                  # record 10 scans into ./scans
    python record_pcd_tri.py -n 50 -o myroom  # record 50 scans into ./myroom
    python record_pcd_tri.py -n 0             # record until Ctrl+C

View the result with view_pcd.py, CloudCompare, or Open3D.
"""
import argparse
import math
import os
import time

import ydlidar


def write_pcd(path, points):
    """points: list of (x, y, z, intensity) tuples."""
    with open(path, "w", newline="\n") as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z intensity\n")
        f.write("SIZE 4 4 4 4\n")
        f.write("TYPE F F F F\n")
        f.write("COUNT 1 1 1 1\n")
        f.write(f"WIDTH {len(points)}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {len(points)}\n")
        f.write("DATA ascii\n")
        for x, y, z, i in points:
            f.write(f"{x:.4f} {y:.4f} {z:.4f} {i:.1f}\n")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-n", "--frames", type=int, default=10,
                    help="number of scans to save (0 = until Ctrl+C), default 10")
    ap.add_argument("-o", "--out", default="scans",
                    help="output folder, default ./scans")
    ap.add_argument("-p", "--port", default=None,
                    help="serial port, e.g. COM3 (default: auto-detect)")
    args = ap.parse_args()

    ydlidar.os_init()
    port = args.port
    if port is None:
        port = "/dev/ydlidar"
        for _, value in ydlidar.lidarPortList().items():
            port = value
    print("Using port", port)

    laser = ydlidar.CYdLidar()
    laser.setlidaropt(ydlidar.LidarPropSerialPort, port)
    laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 115200)
    laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
    laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
    laser.setlidaropt(ydlidar.LidarPropScanFrequency, 10.0)
    laser.setlidaropt(ydlidar.LidarPropSampleRate, 3)
    laser.setlidaropt(ydlidar.LidarPropSingleChannel, True)
    laser.setlidaropt(ydlidar.LidarPropMaxAngle, 180.0)
    laser.setlidaropt(ydlidar.LidarPropMinAngle, -180.0)
    laser.setlidaropt(ydlidar.LidarPropMaxRange, 16.0)
    laser.setlidaropt(ydlidar.LidarPropMinRange, 0.08)
    laser.setlidaropt(ydlidar.LidarPropIntenstiy, False)

    if not (laser.initialize() and laser.turnOn()):
        print("Failed to start the lidar")
        laser.disconnecting()
        return

    os.makedirs(args.out, exist_ok=True)
    session = time.strftime("%Y%m%d_%H%M%S")
    scan = ydlidar.LaserScan()
    saved = 0
    try:
        while ydlidar.os_isOk() and (args.frames == 0 or saved < args.frames):
            if not laser.doProcessSimple(scan):
                print("Failed to get Lidar Data")
                continue
            pts = [(p.range * math.cos(p.angle), p.range * math.sin(p.angle), 0.0, p.intensity)
                   for p in scan.points if p.range > 0]
            if not pts:
                continue
            path = os.path.join(args.out, f"{session}_{saved:04d}.pcd")
            write_pcd(path, pts)
            saved += 1
            print(f"saved {path} ({len(pts)} points)")
    except KeyboardInterrupt:
        pass
    finally:
        laser.turnOff()
        laser.disconnecting()
    print(f"Done: {saved} scan(s) in {os.path.abspath(args.out)}")


if __name__ == "__main__":
    main()

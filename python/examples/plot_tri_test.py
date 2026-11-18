"""Live polar plot for single-channel triangle lidars (X2 / X2L style, 115200 baud).

Close the window to stop the lidar.
"""
import ydlidar
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

RMAX = 8.0  # metres shown on the plot (X2 range is ~8 m)

ydlidar.os_init()
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

fig = plt.figure(figsize=(7, 7))
fig.canvas.manager.set_window_title("YDLidar Monitor")
ax = plt.subplot(polar=True)
ax.set_rmax(RMAX)
ax.set_rlabel_position(135)
ax.grid(True)
dots = ax.scatter([], [], s=4, c=[], cmap="viridis_r", vmin=0, vmax=RMAX)
fig.colorbar(dots, ax=ax, shrink=0.7, label="distance (m)")
title = ax.set_title("waiting for data...")

scan = ydlidar.LaserScan()


def animate(_):
    if not laser.doProcessSimple(scan):
        return dots, title
    pts = [(p.angle, p.range) for p in scan.points if p.range > 0]
    if pts:
        data = np.array(pts)
        # plot as (theta, r); matplotlib polar uses offsets in (theta, r)
        dots.set_offsets(data)
        dots.set_array(data[:, 1])
    hz = 1.0 / scan.config.scan_time if scan.config.scan_time > 0 else 0.0
    title.set_text(f"{len(pts)} points  |  {hz:.1f} Hz")
    return dots, title


if laser.initialize() and laser.turnOn():
    ani = animation.FuncAnimation(fig, animate, interval=50, cache_frame_data=False)
    try:
        plt.show()
    finally:
        laser.turnOff()
else:
    print("Failed to start the lidar")
laser.disconnecting()

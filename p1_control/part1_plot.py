import numpy as np
import matplotlib.pyplot as plt
import os
import config

log = np.load("run_log.npz")

cmd = log["cmd"]
t = np.arange(len(cmd)) * config.DT

os.makedirs("analysis", exist_ok=True)

plt.figure(figsize=(9, 5))

plt.plot(t, cmd[:, 0], label="Linear velocity (m/s)")
plt.plot(t, cmd[:, 1], label="Angular velocity (rad/s)")

plt.xlabel("Time (s)")
plt.ylabel("Control command")
plt.title("LQR Control Effort")
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.savefig("analysis/control_effort.png")
plt.show()


tracking = np.load("analysis/p1_tracking_log.npz")

e_y = tracking["lateral_error"]
e_theta = tracking["heading_error"]

# Sample numbers for LQR-active steps
steps = np.arange(len(e_y))

fig, ax = plt.subplots(2, 1, figsize=(9, 6), sharex=True)

ax[0].plot(steps, e_y)
ax[0].set_ylabel("Lateral Error (m)")
ax[0].grid(True)

ax[1].plot(steps, np.degrees(e_theta))
ax[1].set_ylabel("Heading Error (deg)")
ax[1].set_xlabel("LQR Control Step")
ax[1].grid(True)

plt.suptitle("LQR Tracking Errors")
plt.tight_layout()
plt.savefig("analysis/tracking_error.png", dpi=150)
plt.show()
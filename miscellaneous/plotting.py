from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt


root_dir = Path(__file__).parents[2]
file = root_dir / "recording-components-7.txt"
with open(file, "r") as f:
    start_time = int(f.readline().split()[1].strip())

data = np.loadtxt(file, skiprows=1)
data[:, 1] -= start_time
data[:, 1] /= 1e6

rates = data[data[:, 0] == 0][:, 1:3]
outputs = data[data[:, 0] == 1][:, 1:]


fig, ax1 = plt.subplots(figsize=(12, 8))
ax1.grid()
ax1.plot(rates[:, 0], rates[:, 1], c="tab:blue", label="Rotation rate, dps")
ax1.tick_params(axis="y", labelcolor="tab:blue")
ax1.set_ylabel("Rotation rate [deg/s]")
ax1.set_ylim(bottom=-260, top=260)

ax2 = ax1.twinx()
ax2.grid()
ax2.plot(outputs[:, 0], outputs[:, 2], c="tab:green", label="P")
ax2.plot(outputs[:, 0], outputs[:, 3], c="tab:purple", label="I")
ax2.plot(outputs[:, 0], outputs[:, 4], c="tab:brown", label="D")
ax2.plot(outputs[:, 0], np.clip(outputs[:, 1], -0.15, 0.15), c="tab:orange", label="Total output")
ax2.tick_params(axis="y", labelcolor="tab:orange")
ax2.set_ylabel("PID output [-]")
ax2.set_ylim(bottom=-0.20, top=0.20)

ax1.set_xlabel("t [s]")
fig.legend()
fig.tight_layout()
plt.show()


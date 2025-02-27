from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt


root_dir = Path(__file__).parent
file = root_dir / "detumble_negative.txt"

with open(file, "r") as f:
    start_time = int(f.readline().split()[1].strip())

data = np.loadtxt(file, skiprows=1)
data[:, 1] -= start_time
data[:, 1] /= 1e6

rates = data[data[:, 0] == 0][:, 1:3]
outputs = data[data[:, 0] == 1][:, 1:]


fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 6), sharex=True)
ax1.grid()
ax1.plot(rates[:, 0], rates[:, 1], c="tab:blue", label=r"Rotation rate, $^\circ$/s")
ax1.set_ylabel(r"Rotation rate [$^\circ$/s]")

ax2.grid()
ax2.plot(outputs[:, 0], outputs[:, 2] * 100, c="tab:green", label="Proportional term")
ax2.plot(outputs[:, 0], outputs[:, 3] * 100, c="tab:purple", label="Integral term")
ax2.plot(outputs[:, 0], outputs[:, 4] * 100, c="tab:gray", label="Derivative term")
ax2.plot(outputs[:, 0], np.clip(outputs[:, 1] * 100, -15, 15), c="tab:orange", label="Total PID output")
ax2.set_ylabel("PID output [% throttle]")
ax2.legend()

ax2.set_xlabel("t [s]")
fig.tight_layout()
fig.savefig(root_dir / "detumble_negative.pdf", bbox_inches="tight", dpi=300)
plt.show()


import matplotlib.pyplot as plt
import numpy as np
import re

# Load the file
with open("/mnt/data/ZA.txt", "r", encoding="utf-8", errors="ignore") as f:
    lines = f.readlines()

# Extract all frames
frames = []
for line in lines:
    match = re.search(r"Frame \d+:\s*(.*)", line)
    if match:
        try:
            values = list(map(float, match.group(1).split(",")))
            if len(values) == 256:
                frames.append(np.array(values))
        except ValueError:
            continue

# Plot all frames in subplots (8 per row)
num_frames = len(frames)
cols = 4
rows = int(np.ceil(num_frames / cols))

fig, axes = plt.subplots(rows, cols, figsize=(16, rows * 2.5))
for i, ax in enumerate(axes.flat):
    if i < num_frames:
        ax.plot(frames[i])
        ax.set_title(f"Frame {i}")
        ax.set_xlim(0, 255)
        ax.set_ylim(np.min(frames[i]), np.max(frames[i]))
    else:
        ax.axis('off')

plt.tight_layout()
plt.show()

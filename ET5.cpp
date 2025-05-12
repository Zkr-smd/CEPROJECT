import numpy as np
import matplotlib.pyplot as plt
import scipy.io.wavfile as wav
from scipy.signal import resample

# --- Load WAV File ---
filename = 'electronique.wav'
sample_rate, data = wav.read(filename)

# Convert to mono if stereo
if len(data.shape) == 2:
    data = data.mean(axis=1)

# Normalize to float [-1, 1]
data = data / np.max(np.abs(data))

# Resample to 8000 Hz if needed
target_rate = 8000
if sample_rate != target_rate:
    duration = len(data) / sample_rate
    num_samples = int(duration * target_rate)
    data = resample(data, num_samples)
    print(f"Resampled from {sample_rate} Hz to {target_rate} Hz.")
else:
    print("No resampling needed.")

# --- Framing Parameters ---
frame_size = 256
hop_size = 128
num_frames = int((len(data) - frame_size) / hop_size) + 1

frames = np.zeros((num_frames, frame_size))
for i in range(num_frames):
    start = i * hop_size
    frames[i] = data[start:start + frame_size]

print(f"Generated {num_frames} frames of 256 samples with 50% overlap.")

# --- Validation: Plot first 3 frames ---
plt.figure(figsize=(12, 6))
for i in range(3):
    plt.plot(frames[i], label=f'Frame {i}')
plt.title('First 3 Overlapping Frames (256 samples)')
plt.xlabel('Sample index')
plt.ylabel('Amplitude')
plt.legend()
plt.grid(True)
plt.show()

# --- Optional: Save frames to CSV if needed ---
# np.savetxt("frames.csv", frames, delimiter=",")

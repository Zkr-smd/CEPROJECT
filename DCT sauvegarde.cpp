import serial
import numpy as np
import matplotlib.pyplot as plt
import librosa
import librosa.display
import scipy.fftpack

port = 'COM9'  # ⚠️ Adapter au port utilisé
baud = 115200
samples = []

print("Lecture série en cours...")
with serial.Serial(port, baud, timeout=2) as ser:
    while len(samples) < 8000:
        try:
            line = ser.readline().decode().strip()
            if line:
                samples.append(int(line))
        except:
            continue

samples = np.array(samples, dtype=np.float32)
samples -= 2048.0  # Recentre
samples /= np.max(np.abs(samples))  # Normalisation

pre_emphasis = 0.97
emphasized = np.append(samples[0], samples[1:] - pre_emphasis * samples[:-1])

frame_size = 0.032  # 32 ms
frame_stride = 0.016  # 16 ms
sample_rate = 8000
frame_len = int(round(frame_size * sample_rate))
frame_step = int(round(frame_stride * sample_rate))

frames = librosa.util.frame(emphasized, frame_length=frame_len, hop_length=frame_step).T
frames *= np.hamming(frame_len)

NFFT = 512
mag_frames = np.abs(np.fft.rfft(frames, NFFT))
pow_frames = ((1.0 / NFFT) * (mag_frames ** 2))

nfilt = 26
mel_fb = librosa.filters.mel(sr=sample_rate, n_fft=NFFT, n_mels=nfilt)
mel_energy = np.dot(pow_frames, mel_fb.T)
mel_energy = np.where(mel_energy == 0, np.finfo(float).eps, mel_energy)

mfcc = scipy.fftpack.dct(np.log(mel_energy), type=2, axis=1, norm='ortho')[:, :13]

plt.figure(figsize=(10, 4))
img = plt.imshow(mfcc.T, aspect='auto', origin='lower', cmap='coolwarm',
                 extent=[0, mfcc.shape[0], 0, 13])
plt.xlabel("Time")
plt.ylabel("MFCC Coefficients")
cbar = plt.colorbar(img)
cbar.set_label("dB")
plt.title("MFCC - Arduino Signal")
plt.tight_layout()
plt.show()

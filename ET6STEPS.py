import numpy as np
import matplotlib.pyplot as plt
from collections import Counter

def load_csv(fname):
    """
    Lit un CSV où chaque ligne peut être :
      - idx,val1,val2,...,valN
      - val1,val2,...,valN
      - ou finir par 'DONE'
    On :
      • skip les lignes vides
      • skip les tokens non-convertibles en float en tête (header)
      • skip toute ligne où la conversion de TOUTES les colonnes en float échoue
      • conserve uniquement les lignes ayant la longueur la plus fréquente
    Retourne un array (n_rows, n_cols)
    """
    raw = []
    with open(fname, 'r') as f:
        for ln in f:
            ln = ln.strip()
            if not ln or ln.upper().startswith('DONE'):
                continue

            parts = ln.split(',')
            try:
                float(parts[0])
            except ValueError:
                parts = parts[1:]

            if not parts:
                continue

            vals = []
            ok = True
            for x in parts:
                try:
                    vals.append(float(x))
                except ValueError:
                    ok = False
                    break
            if ok:
                raw.append(vals)

    if not raw:
        raise ValueError(f"Aucune ligne numérique valable dans '{fname}'")

    lengths = [len(r) for r in raw]
    expected_len, _ = Counter(lengths).most_common(1)[0]
    return np.array([r for r in raw if len(r) == expected_len])

# Create figure with two subplots
plt.figure(figsize=(12, 8))

# === 1) Raw Signal Plot ===
try:
    raw_data = load_csv('raw.txt')
    raw_signal = raw_data[:,-1] if raw_data.shape[1] > 1 else raw_data[:,0]
    raw_idx = raw_data[:,0] if raw_data.shape[1] > 2 else np.arange(len(raw_signal))

    plt.subplot(2, 1, 1)
    plt.plot(raw_idx, raw_signal, color='blue', linewidth=1)
    plt.title("Signal Brut Normalisé", fontweight='bold')
    plt.xlabel("Index de l'échantillon", fontsize=10)
    plt.ylabel("Amplitude", fontsize=10)
    plt.ylim(-1.1, 1.1)
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.axhline(0, color='black', linewidth=0.5)
except Exception as e:
    print("🔴 Erreur lors du chargement du signal brut :", e)

# === 2) Pre-emphasized Signal Plot ===
try:
    pre_data = load_csv('pre.txt')
    pre_signal = pre_data[:,-1] if pre_data.shape[1] > 1 else pre_data[:,0]
    pre_idx = pre_data[:,0] if pre_data.shape[1] > 2 else np.arange(len(pre_signal))

    plt.subplot(2, 1, 2)
    plt.plot(pre_idx, pre_signal, color='orange', linewidth=1)
    plt.title("Signal Après Pré-emphasis", fontweight='bold')
    plt.xlabel("Index de l'échantillon", fontsize=10)
    plt.ylabel("Amplitude", fontsize=10)
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.axhline(0, color='black', linewidth=0.5)
except Exception as e:
    print("🔴 Erreur lors du chargement du signal pré-emphasis :", e)

plt.tight_layout(pad=3.0)
plt.show()
plt.tight_layout()
plt.show()
# === 2) Spectre FFT ===
try:
    fft_data = load_csv('fft.txt')
    N = fft_data.shape[1]  # Number of frequency bins
    sample_rate = 8000  # 8kHz sampling rate
    freqs = np.linspace(0, sample_rate/2, N)

    plt.figure(figsize=(12,6))
    plt.plot(freqs, fft_data[0])  # Plot first frame
    plt.title("Spectre FFT d'une frame (fenêtrée)")
    plt.xlabel("Fréquence (Hz)")
    plt.ylabel("Magnitude")
    plt.ylim(0, 10)  # This line sets the Y-axis limits
    plt.grid(True)
    plt.tight_layout()
    plt.show()
except Exception as e:
    print("🔴 Étape FFT échouée :", e)

# === 3) Banc MEL (log) ===
try:
    mel_data = load_csv('mel.txt')
    n_frames = mel_data.shape[0]
    n_filters = mel_data.shape[1]

    plt.figure(figsize=(12,6))
    plt.imshow(mel_data.T, aspect='auto', origin='lower',
               extent=[0, n_frames, 0, n_filters],
               cmap='viridis')
    plt.title("Énergie des filtres MEL (échelle log) par frame")
    plt.xlabel("Index de frame")
    plt.ylabel("Index de filtre MEL")
    plt.colorbar(label="Log énergie")
    plt.tight_layout()
    plt.show()
except Exception as e:
    print("🔴 Étape MEL échouée :", e)


    # Also plot first frame's coefficients
    plt.figure(figsize=(12,6))
    plt.bar(range(n_coeffs), mfcc_data[0])
    plt.title("Coefficients MFCC pour la première frame")
    plt.xlabel("Index de coefficient MFCC")
    plt.ylabel("Valeur")
    plt.grid(True)
    plt.tight_layout()
    plt.show()
except Exception as e:
    print("🔴 Étape DCT échouée :", e)

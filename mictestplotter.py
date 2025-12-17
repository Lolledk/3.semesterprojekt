#!/usr/bin/env python3
"""
Plot Intensity vs Distance from existing recordings.
Assumes WAV files are named as in mic_distance_test.py: 'recording_{distance}cm_{timestamp}.wav'
"""

import os
import re
import wave
import numpy as np
import matplotlib.pyplot as plt

RECORDINGS_DIR = "mic_test_recordings"  # Same as save_recording
SAMPLE_RATE = 16000  # Match your original config
CHUNK_SIZE = 1024

def extract_distance(filename):
    # Looks for 'recording_{distance}cm_' in the filename
    m = re.search(r'recording_(\d+)cm', filename)
    return int(m.group(1)) if m else None

def analyze_wav(filename):
    with wave.open(filename, "rb") as wf:
        channels = wf.getnchannels()
        sampwidth = wf.getsampwidth()
        nframes = wf.getnframes()
        pcm_bytes = wf.readframes(nframes)
        audio = np.frombuffer(pcm_bytes, dtype=np.int16)
    # RMS calculation over chunks
    intensities = []
    for i in range(0, len(audio), CHUNK_SIZE):
        chunk = audio[i:i+CHUNK_SIZE]
        if len(chunk) > 0:
            rms = np.sqrt(np.mean(chunk.astype(np.float64)**2))
            intensities.append(rms)
    avg_intensity = np.mean(intensities)
    return avg_intensity

def main():
    results = []
    for fname in os.listdir(RECORDINGS_DIR):
        if fname.endswith(".wav"):
            distance = extract_distance(fname)
            if distance is not None:
                filepath = os.path.join(RECORDINGS_DIR, fname)
                avg_int = analyze_wav(filepath)
                results.append((distance, avg_int))
    if not results:
        print("No recordings found.")
        return
    # Sort by distance
    results.sort()
    distances, intensities = zip(*results)
    # Plot
    plt.figure(figsize=(8, 4.5))
    plt.plot(distances, intensities, 'bo-', linewidth=2, markersize=8)
    plt.xlabel("Distance (cm)", fontsize=12)
    plt.ylabel("Average Intensity (RMS)", fontsize=12)
    plt.title("Microphone Intensity vs Distance", fontsize=14, fontweight='bold')
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig("intensity_vs_distance.pdf", dpi=300, bbox_inches='tight')
    print("Plot saved as intensity_vs_distance.pdf")
    plt.show()

if __name__ == "__main__":
    main()
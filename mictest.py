#!/usr/bin/env python3
"""
Microphone Distance Testing Script
Tests the relationship between distance from microphone and recording intensity
Matches the project configuration:  16000 Hz sampling using arecord
"""

import subprocess
import numpy as np
import wave
import io
import os
from datetime import datetime
import matplotlib.pyplot as plt

# Audio configuration - matching your project
DEVICE = "plughw:1,0"  # Same as your stream_server.py
SAMPLE_RATE = 16000
CHANNELS = 1
RECORD_SECONDS = 3

class MicDistanceTest:
    def __init__(self):
        self.results = []
        
    def test_device(self):
        """Test if the microphone device is accessible"""
        try:
            cmd = ["arecord", "-D", DEVICE, "-d", "1", "-f", "S16_LE", 
                   "-c", str(CHANNELS), "-r", str(SAMPLE_RATE), "-t", "raw"]
            subprocess.check_output(cmd, stderr=subprocess.DEVNULL)
            print(f"✓ Device {DEVICE} is accessible and working")
            return True
        except subprocess.CalledProcessError as e:
            print(f"✗ Error accessing device {DEVICE}: {e}")
            return False
    
    def capture_raw(self, duration=RECORD_SECONDS):
        """Capture raw PCM audio using arecord (same as your project)"""
        cmd = ["arecord", "-D", DEVICE, "-f", "S16_LE", "-c", str(CHANNELS),
               "-r", str(SAMPLE_RATE), "-d", str(duration), "-t", "raw"]
        return subprocess. check_output(cmd, stderr=subprocess.DEVNULL)
    
    def pcm_to_wav_bytes(self, pcm_bytes):
        """Convert raw PCM to WAV format (same as your project)"""
        b = io.BytesIO()
        with wave.open(b, "wb") as w:
            w.setnchannels(CHANNELS)
            w.setsampwidth(2)  # 16-bit
            w.setframerate(SAMPLE_RATE)
            w.writeframes(pcm_bytes)
        return b.getvalue()
    
    def analyze_audio(self, pcm_bytes):
        """Analyze the audio intensity with better error handling"""
    # Convert bytes to numpy array
        audio_data = np.frombuffer(pcm_bytes, dtype=np. int16)
    
    # Check if we got valid data
        if len(audio_data) == 0:
            print("⚠️  No audio data captured!")
            return None
    
        if np.all(audio_data == 0):
            print("⚠️  All audio samples are zero - mic may not be working!")
        
        # Calculate RMS intensity over time
        chunk_size = 1024
        intensities = []
        for i in range(0, len(audio_data), chunk_size):
            chunk = audio_data[i:i+chunk_size]
            if len(chunk) > 0:
                # Convert to float to avoid overflow
                chunk_float = chunk.astype(np.float64)
                rms = np. sqrt(np.mean(chunk_float**2))
                intensities.append(rms)
        
        # Filter out any NaN or inf values
        intensities = [x for x in intensities if np.isfinite(x)]
        
        if len(intensities) == 0:
            print("⚠️  No valid intensity values calculated!")
            return None
        
        # Overall metrics
        avg_intensity = np. mean(intensities)
        max_intensity = np.max(intensities)
        min_intensity = np.min(intensities)
        std_intensity = np.std(intensities)
        
        # Convert to decibels (approximate)
        if avg_intensity > 0:
            avg_db = 20 * np.log10(avg_intensity / 32768.0) + 90
        else:
            avg_db = -np.inf
        
        return {
            'intensities': intensities,
            'avg_intensity': avg_intensity,
            'max_intensity': max_intensity,
            'min_intensity': min_intensity,
            'std_intensity': std_intensity,
            'avg_db':  avg_db,
            'audio_data': audio_data
        }
    
    def record_audio(self, distance_cm):
        """Record audio at a specified distance and analyze it"""
        print(f"\nRecording at {distance_cm}cm distance...")
        print(f"Recording for {RECORD_SECONDS} seconds.. .", end=" ", flush=True)
        
        try:
            # Capture raw PCM
            pcm_bytes = self.capture_raw()
            print("✓ Done!")
            
            # Analyze audio
            analysis = self.analyze_audio(pcm_bytes)
            
            # Convert to WAV for saving
            wav_bytes = self. pcm_to_wav_bytes(pcm_bytes)
            
            result = {
                'distance_cm': distance_cm,
                'pcm_bytes': pcm_bytes,
                'wav_bytes': wav_bytes,
                **analysis  # Unpack analysis results
            }
            
            self.results.append(result)
            
            print(f"Average Intensity: {analysis['avg_intensity']:.2f}")
            print(f"Max Intensity: {analysis['max_intensity']:.2f}")
            print(f"Approximate dB: {analysis['avg_db']:.2f}")
            
            return result
            
        except subprocess.CalledProcessError as e:
            print(f"\n✗ Error during recording: {e}")
            return None
        except Exception as e:
            print(f"\n✗ Unexpected error: {e}")
            return None
    
    def save_recording(self, result, output_dir="mic_test_recordings"):
        """Save the recording to a WAV file"""
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{output_dir}/recording_{result['distance_cm']}cm_{timestamp}.wav"
        
        with open(filename, 'wb') as f:
            f.write(result['wav_bytes'])
        
        print(f"Recording saved to: {filename}")
        return filename
    
    def plot_results(self, save_plot=True):
        """Plot the test results"""
        if not self.results:
            print("No results to plot!")
            return
        
        distances = [r['distance_cm'] for r in self.results]
        avg_intensities = [r['avg_intensity'] for r in self.results]
        avg_dbs = [r['avg_db'] for r in self.results if r['avg_db'] != -np.inf]
        valid_distances = [r['distance_cm'] for r in self.results if r['avg_db'] != -np.inf]
        
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        
        # Plot 1: Intensity vs Distance
        axes[0, 0].plot(distances, avg_intensities, 'bo-', linewidth=2, markersize=8)
        axes[0, 0].set_xlabel('Distance (cm)', fontsize=12)
        axes[0, 0].set_ylabel('Average Intensity (RMS)', fontsize=12)
        axes[0, 0].set_title('Microphone Intensity vs Distance', fontsize=14, fontweight='bold')
        axes[0, 0].grid(True, alpha=0.3)
        
        # Plot 2: dB vs Distance
        if avg_dbs:
            axes[0, 1].plot(valid_distances, avg_dbs, 'ro-', linewidth=2, markersize=8)
            axes[0, 1].set_xlabel('Distance (cm)', fontsize=12)
            axes[0, 1].set_ylabel('Approximate Level (dB)', fontsize=12)
            axes[0, 1].set_title('Sound Level vs Distance', fontsize=14, fontweight='bold')
            axes[0, 1].grid(True, alpha=0.3)
        
        # Plot 3: Intensity over time for first recording
        if self.results:
            first = self.results[0]
            time_axis = np.arange(len(first['intensities'])) * (1024 / SAMPLE_RATE)
            axes[1, 0].plot(time_axis, first['intensities'], 'g-', linewidth=1)
            axes[1, 0].set_xlabel('Time (seconds)', fontsize=12)
            axes[1, 0].set_ylabel('Intensity (RMS)', fontsize=12)
            axes[1, 0].set_title(f'Intensity Over Time - {first["distance_cm"]}cm', fontsize=14, fontweight='bold')
            axes[1, 0].grid(True, alpha=0.3)
        
        # Plot 4: Waveform for first recording
        if self.results:
            first = self.results[0]
            time_axis = np.arange(len(first['audio_data'])) / SAMPLE_RATE
            axes[1, 1].plot(time_axis, first['audio_data'], 'purple', linewidth=0.5, alpha=0.7)
            axes[1, 1].set_xlabel('Time (seconds)', fontsize=12)
            axes[1, 1].set_ylabel('Amplitude', fontsize=12)
            axes[1, 1].set_title(f'Waveform - {first["distance_cm"]}cm', fontsize=14, fontweight='bold')
            axes[1, 1].grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        if save_plot:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            plot_filename = f"mic_test_results_{timestamp}.png"
            plt.savefig(plot_filename, dpi=300, bbox_inches='tight')
            print(f"\nPlot saved to: {plot_filename}")
        
        plt.show()
    
    def print_summary(self):
        """Print a summary of all test results"""
        if not self.results:
            print("No results available!")
            return
        
        print("\n" + "=" * 70)
        print("TEST RESULTS SUMMARY")
        print("=" * 70)
        print(f"{'Distance (cm)':<15} {'Avg Intensity':<15} {'Max Intensity':<15} {'Std Dev':<12} {'Approx dB':<12}")
        print("-" * 70)
        
        for r in self.results:
            print(f"{r['distance_cm']: <15} {r['avg_intensity']:<15.2f} {r['max_intensity']:<15.2f} "
                  f"{r['std_intensity']:<12.2f} {r['avg_db']:<12.2f}")
        
        print("=" * 70 + "\n")

def main():
    print("=" * 70)
    print("MICROPHONE DISTANCE TESTING TOOL")
    print(f"Device: {DEVICE} | Sample Rate: {SAMPLE_RATE} Hz | Duration: {RECORD_SECONDS}s")
    print("=" * 70)
    
    tester = MicDistanceTest()
    
    # Test device
    print("\nTesting microphone device...")
    if not tester.test_device():
        print("\n⚠ Could not access microphone.  Please check:")
        print(f"  1. Device name is correct:  {DEVICE}")
        print("  2. USB microphone is connected")
        print("  3. Run 'arecord -l' to list available devices")
        return
    
    print("\n--- Test Instructions ---")
    print("1. Position the microphone at the specified distance")
    print("2. Speak consistently (same volume/phrase) for each test")
    print("3. Minimize background noise")
    print("4. Suggestion: Use a consistent phrase like 'Testing one two three'")
    print("-" * 70)
    
    # Run tests at different distances
    while True:
        distance_input = input("\nEnter distance in cm (or 'done' to finish): ").strip()
        
        if distance_input.lower() in ['done', 'quit', 'exit', 'q']:
            break
        
        try:
            distance = int(distance_input)
            
            input(f"\n→ Position speaker at {distance}cm and press Enter to start recording...")
            
            result = tester.record_audio(distance)
            
            if result: 
                # Ask if user wants to save recording
                save_choice = input("Save this recording? (y/n): ").strip().lower()
                if save_choice == 'y': 
                    tester.save_recording(result)
            
        except ValueError:
            print("❌ Invalid input!  Please enter a number or 'done'.")
        except KeyboardInterrupt:
            print("\n\nTest interrupted by user.")
            break
    
    # Show results
    if tester.results:
        tester.print_summary()
        
        plot_choice = input("\nGenerate plots? (y/n): ").strip().lower()
        if plot_choice == 'y':
            tester.plot_results(save_plot=True)
    else:
        print("\nNo recordings were made.")
    
    print("\nTesting complete!")

if __name__ == "__main__":
    main()
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt

from vmdpy import VMD

# ==========================================
# DISPLAY CONFIGURATION
# ==========================================
DISPLAY_ENABLED = True

# ==========================================
# 0. SAFETY FUNCTIONS (SQI Shield)
# ==========================================
def is_signal_valid1(signal, tolerance_threshold=8.0):
    """
    Checks if the signal contains massive parasitic movements.
    Returns True if the signal is usable, False if it is corrupted.
    """
    highest_point = np.max(signal)
    lowest_point = np.min(signal)
    total_amplitude = highest_point - lowest_point
    
    # If the gap between max and min is too large, it is an artifact
    if total_amplitude > tolerance_threshold:
        return False, total_amplitude
    return True, total_amplitude

def is_signal_valid2(signal, jump_threshold=1.5):
    """
    Instead of looking at the total height, this checks the severity of sudden jumps.
    np.diff calculates the instantaneous gap between each radar frame.
    """
    # Calculate the rate of change of the signal
    rate_of_change = np.abs(np.diff(signal))
    
    # Find the most severe jump across the entire recording
    max_jump = np.max(rate_of_change)
    
    # If the signal jumps suddenly above the threshold, it is a destructive artifact
    if max_jump > jump_threshold:
        return False, max_jump
    
    return True, max_jump

def is_signal_valid3(isolated_signal, fs=50.0, search_range=(0.1, 4.0), cardiac_range=(0.8, 2.5)):
    """
    Frequency Quality Control (SQI 3):
    Checks if the signal isolated by the VMD is genuinely a heartbeat.
    Analyzes the FFT to ensure the main energy peak falls within the human cardiac range.
    
    Returns: (True/False, Found_Frequency, Calculated_BPM)
    """
    num_frames = len(isolated_signal)
    
    # FFT Calculation
    frequencies = np.fft.rfftfreq(num_frames, d=1/fs)
    spectrum = np.abs(np.fft.rfft(isolated_signal))
    
    # 1. Mask the spectrum to search for energy in a broad zone (0.1 to 4.0 Hz)
    search_mask = (frequencies >= search_range[0]) & (frequencies <= search_range[1])
    useful_freqs = frequencies[search_mask]
    useful_spectrum = spectrum[search_mask]
    
    # 2. Find the dominant frequency (the peak with the highest energy)
    dominant_peak_freq = useful_freqs[np.argmax(useful_spectrum)]
    calculated_bpm = dominant_peak_freq * 60.0
    
    # 3. Verification: Is this peak within the acceptable cardiac zone (0.8 to 2.5 Hz)?
    if cardiac_range[0] <= dominant_peak_freq <= cardiac_range[1]:
        return True, dominant_peak_freq, calculated_bpm
    else:
        # It is likely noise or residual respiration
        return False, dominant_peak_freq, calculated_bpm
    
# ==========================================
# 1. LOADING AND PREPARATION
# ==========================================
for iter_idx in range(6):
    file_path = f"data/tripod/240502_userA_tripod_02_{iter_idx}.csv" 
    fs = 50.0  

    print(f"Processing data index: {iter_idx}")
    raw_data = np.loadtxt(file_path, delimiter=',', skiprows=0)

    uwb_data = raw_data[:, 12:252]
    complex_signal = uwb_data[:, :120] + 1j * uwb_data[:, 120:]
    clutter_free_signal = complex_signal - np.mean(complex_signal, axis=0)

    dynamic_amplitude = np.abs(clutter_free_signal)
    dynamic_phase = np.unwrap(np.angle(clutter_free_signal), axis=0)

    # Dynamic tracking 
    window_size = int(fs * 1)
    num_frames = dynamic_amplitude.shape[0]
    raw_phase_signal = np.zeros(num_frames)
    prev_bin = None

    for start in range(0, num_frames, window_size):
        end = min(start + window_size, num_frames)
        local_variance = np.var(dynamic_amplitude[start:end, :], axis=0)
        best_bin = np.argmax(local_variance)
        
        if prev_bin is None: prev_bin = best_bin
        if abs(prev_bin - best_bin) <= 1: prev_bin = best_bin
        
        raw_phase_signal[start:end] = dynamic_phase[start:end, prev_bin]

    heart_reference = raw_data[:, -1] 
    raw_phase_signal -= np.mean(raw_phase_signal)
    heart_reference -= np.mean(heart_reference)

    # ==========================================
    # QUALITY VERIFICATION (Garbage In, Garbage Out)
    # ==========================================
    tolerance_threshold = 20.0
    jump_threshold = 4
    signal_ok, calculated_amplitude = is_signal_valid1(raw_phase_signal, tolerance_threshold)
  
    if not signal_ok:
        signal_ok, max_jump_diff = is_signal_valid2(raw_phase_signal, jump_threshold)

    if not signal_ok:
        print(f"File rejected: Total amplitude of {calculated_amplitude:.1f} exceeds the tolerance threshold ({tolerance_threshold}).")
        print(f"File rejected: Maximum jump of {max_jump_diff:.1f} exceeds the jump threshold ({jump_threshold}).")
        print("File rejected: Garbage In, Garbage Out. VMD calculation aborted.")
        
        # Display a warning plot for the user/clinician
        plt.figure(iter_idx, figsize=(14, 4))
        time_axis = np.arange(num_frames) / fs
        plt.plot(time_axis, raw_phase_signal, color='black')
        plt.axhspan(-calculated_amplitude, calculated_amplitude, color='red', alpha=0.2)
        plt.title(f"ANALYSIS ABORTED: Excessive patient motion (Amplitude: {calculated_amplitude:.1f})", color='red', fontweight='bold')
        plt.xlabel("Time (s)")
        plt.ylabel("Phase Amplitude")
        plt.grid(True, linestyle=':')
        plt.tight_layout()
 
    else:
        print(f"Clean signal validated (Amplitude: {calculated_amplitude:.1f}). Starting analysis...")
        
        # ==========================================
        # 1.5 PRE-VMD CLEANING
        # ==========================================
        def apply_pre_vmd_filter(data, freq_min, freq_max, fs, order=4):
            nyquist = 0.5 * fs
            b, a = butter(order, [freq_min / nyquist, freq_max / nyquist], btype='band')
            return filtfilt(b, a, data)

        cleaned_phase_signal = apply_pre_vmd_filter(raw_phase_signal, 0.2, 5.0, fs)

        # ==========================================
        # 2. VMD DECOMPOSITION
        # ==========================================
        alpha, tau, K, DC, init, tol = 2000, 0., 5, 0, 1, 1e-7         
        u, u_hat, omega = VMD(cleaned_phase_signal, alpha, tau, K, DC, init, tol)

        # ==========================================
        # 3. HEART SIGNAL EXTRACTION
        # ==========================================
        heart_mode_index = 1 
        max_heart_energy = 0

        for i in range(1, K): 
            mode_spectrum = np.abs(np.fft.rfft(u[i, :]))
            mode_frequencies = np.fft.rfftfreq(num_frames, d=1/fs)
            
            mask = (mode_frequencies >= 0.8) & (mode_frequencies <= 2.5)
            energy_in_heart_zone = np.max(mode_spectrum[mask])
            
            if energy_in_heart_zone > max_heart_energy:
                max_heart_energy = energy_in_heart_zone
                heart_mode_index = i

        isolated_cardiac_signal = u[heart_mode_index, :]
        
        # Test the VMD result
        freq_ok, freq_hz, bpm = is_signal_valid3(isolated_cardiac_signal, fs=50.0)

        if not freq_ok:
            print(f"File rejected after analysis: The main peak is at {bpm:.1f} BPM.")
            print("VMD failed to isolate the heart (dominant noise).")
            
            # --- ERROR PLOT CREATION ---
            # 1. Quickly recalculate FFT for visualization
            frequencies = np.fft.rfftfreq(len(isolated_cardiac_signal), d=1/fs)
            error_spectrum = np.abs(np.fft.rfft(isolated_cardiac_signal))
            error_spectrum = error_spectrum / np.max(error_spectrum) # Normalize from 0 to 1
            
            # 2. Figure creation
            plt.figure(num=f"Frequency_Error_{iter_idx}", figsize=(12, 5))
            
            # Limit display from 0 to 4.0 Hz to clearly highlight the anomaly
            display_mask = (frequencies >= 0.0) & (frequencies <= 4.0)
            plt.plot(frequencies[display_mask], error_spectrum[display_mask], color='black', linewidth=1.5, label='VMD Spectrum')
            
            # 3. Draw the correct cardiac zone
            plt.axvspan(0.8, 2.5, color='green', alpha=0.1, label="Valid Cardiac Zone (0.8 - 2.5 Hz)")
            plt.axvline(0.8, color='green', linestyle='--', alpha=0.5)
            plt.axvline(2.5, color='green', linestyle='--', alpha=0.5)
            
            # 4. Place a red marker on the detected peak
            plt.plot(freq_hz, 1.0, 'ro', markersize=10)
            
            # 5. Final aesthetics
            plt.title(f"SQI 3 FAILURE: Dominant noise outside cardiac zone", color='red', fontweight='bold', fontsize=14)
            plt.xlabel("Frequency (Hz) - Note: Multiply by 60 for BPM")
            plt.ylabel("Energy (Spectral Density)")
            plt.legend(loc='upper right')
            plt.grid(True, linestyle=':', alpha=0.6)
            plt.tight_layout()
            
            # Display without blocking the loop
            plt.show(block=False)
            
        else:
            print(f"SUCCESS: Heart detected and validated at {bpm:.1f} BPM.")
            # ==========================================
            # 4. FINAL FFT ANALYSIS
            # ==========================================
            frequencies = np.fft.rfftfreq(num_frames, d=1/fs)
            radar_spectrum = np.abs(np.fft.rfft(isolated_cardiac_signal))
            radar_spectrum = radar_spectrum / np.max(radar_spectrum) 

            ref_spectrum = np.abs(np.fft.rfft(heart_reference))
            ref_spectrum = ref_spectrum / np.max(ref_spectrum) 

            heart_mask = (frequencies >= 0.8) & (frequencies <= 2.5)
            useful_freqs = frequencies[heart_mask]

            bpm_radar = useful_freqs[np.argmax(radar_spectrum[heart_mask])] * 60
            bpm_ref = useful_freqs[np.argmax(ref_spectrum[heart_mask])] * 60

            print(f"Extracted Radar BPM : {bpm_radar:.1f} BPM")
            print(f"Ground Truth BPM    : {bpm_ref:.1f} BPM")

            # ==========================================
            # 5. COMPREHENSIVE DASHBOARD DISPLAY
            # ==========================================
            fig, axs = plt.subplots(3, 1, num=iter_idx, figsize=(14, 12))
            time_axis = np.arange(num_frames) / fs

            axs[0].plot(time_axis, raw_phase_signal, color='gray')
            axs[0].set_title(f"1. Valid Raw Signal (Amplitude: {calculated_amplitude:.1f})", color='green', fontweight='bold')
            axs[0].grid(True, linestyle=':', alpha=0.6)

            axs[1].plot(time_axis, (heart_reference/np.max(heart_reference)), label="ECG Reference", color='blue', alpha=0.3)
            axs[1].plot(time_axis, (isolated_cardiac_signal/np.max(isolated_cardiac_signal)), label=f"Isolated VMD Mode", color='red')
            axs[1].set_title(f"2. Temporal Domain: Isolated Cardiac Movement (Mode {heart_mode_index + 1})", fontweight='bold')
            axs[1].legend()
            axs[1].grid(True, linestyle=':', alpha=0.6)

            axs[2].plot(frequencies, ref_spectrum, label=f'Reference ({bpm_ref:.1f} BPM)', color='blue', alpha=0.5, linewidth=2)
            axs[2].plot(frequencies, radar_spectrum, label=f'Radar UWB VMD ({bpm_radar:.1f} BPM)', color='red', linewidth=2)
            axs[2].axvspan(0.8, 2.5, color='green', alpha=0.05, label="Cardiac Zone")
            axs[2].set_xlim(0.5, 3.0)
            axs[2].set_title("3. Frequency Domain (FFT): Comparison", fontweight='bold')
            axs[2].legend()
            axs[2].grid(True, linestyle=':', alpha=0.6)

            plt.tight_layout()

if DISPLAY_ENABLED:
    plt.show()

# NOK : fig 1, fig 3
import os
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt, find_peaks

def RPM(file_name, save_fig=False):

    raw_data_list = []

    for i in range(8):
        full_file_path = f"data/tripod/{file_name}_{i}.csv"
        # print(f"Loading file: {full_file_path}")
        if os.path.exists(full_file_path):
            data = np.loadtxt(full_file_path, delimiter=',', skiprows=1)
            raw_data_list.append(data)
        
    global_data = np.vstack(raw_data_list)

    filtered_data = global_data[:, 12:252]

    clutter_mean = np.mean(filtered_data, axis=0)
    clutter_free_data = filtered_data - clutter_mean

    I = clutter_free_data[:, :120]
    Q = clutter_free_data[:, 120:240]

    dynamic_amplitude = np.sqrt(I**2 + Q**2)

    """
    # 5. Display the result
    plt.figure(figsize=(10, 6))
    # Limit colors (vmax) to highlight micro-movements
    plt.imshow(dynamic_amplitude, aspect='auto', cmap='jet', vmax=np.percentile(dynamic_amplitude, 99))
    plt.title("UWB Amplitude with Clutter Filter (Pure Motion)")
    plt.xlabel("Distance (Bins 0 to 119)")
    plt.ylabel("Time (Frames)")
    plt.colorbar(label="Motion Amplitude")
    plt.show()
    """

    variance_per_bin = np.var(dynamic_amplitude, axis=0)
    target_bin = np.argmax(variance_per_bin)

    fs = 50.0  # 50 frames per second
    window_size = int(fs * 1)  # 1-second window (50 frames)
    num_frames = dynamic_amplitude.shape[0]

    # --- 1. Dynamic Extraction ---
    dynamic_radar_signal = np.zeros(num_frames)
    bin_history = [] # Track how the target moved over time
    prev_bin = None

    # Iterate through the signal second by second
    for start in range(0, num_frames, window_size):
        end = min(start + window_size, num_frames)
        
        # Isolate the matrix for the current second
        time_window = dynamic_amplitude[start:end, :]
        
        # Find the best bin in this short timeframe (using variance)
        local_variance = np.var(time_window, axis=0)
        local_variance[:10] = 0
        best_local_bin = np.argmax(local_variance)
        
        if prev_bin is None:
            bin_history.append(best_local_bin)
            dynamic_radar_signal[start:end] = time_window[:, best_local_bin]
            prev_bin = best_local_bin
            
        # Save the choice and extract the signal
        if (abs(prev_bin - best_local_bin) <= 1):
            bin_history.append(best_local_bin)
            dynamic_radar_signal[start:end] = time_window[:, best_local_bin]
            prev_bin = best_local_bin
        else:
            # If the best bin jumps too much, keep the previous one (stability)
            bin_history.append(prev_bin)
            dynamic_radar_signal[start:end] = time_window[:, prev_bin]

    # print(f"Bins crossed by the subject over time: {bin_history}")

    # --- 2. Filtering to smooth jumps and extract respiration ---
    def apply_filter(data, freq_min, freq_max, fs, order=4):
        nyquist = 0.5 * fs
        b, a = butter(order, [freq_min / nyquist, freq_max / nyquist], btype='band')
        return filtfilt(b, a, data)

    filtered_resp_signal = apply_filter(dynamic_radar_signal, 0.1, 0.5, fs)

    # --- 3. Ground Truth and Normalization ---
    reference = global_data[:, -2] # Respiration column
    # Filter the reference to remove its natural drift (optional but cleaner)
    filtered_reference = apply_filter(reference, 0.1, 0.5, fs)

    # Normalization (Z-score)
    norm_resp_signal = (filtered_resp_signal - np.mean(filtered_resp_signal)) / np.std(filtered_resp_signal)
    norm_reference = (filtered_reference - np.mean(filtered_reference)) / np.std(filtered_reference)

    # --- 4. Result Display (Commented out) ---
    """
    plt.figure(figsize=(12, 5))
    plt.plot(norm_resp_signal, label="UWB Radar (Dynamic Tracking + Filter)", color='blue', linewidth=2)
    plt.plot(norm_reference, label="Ground Truth", color='red', alpha=0.7, linestyle='--')

    plt.title("UWB Respiration with Dynamic Target Tracking vs Ground Truth")
    plt.xlabel("Time (Frames)")
    plt.ylabel("Normalized Amplitude")
    plt.legend()
    plt.grid(True)
    plt.show()
    """

    # ==========================================
    # 7. Peak Detection and RPM Calculation
    # ==========================================

    # Find the maximums (inspiration peaks)
    # distance=95: minimum time between 2 breaths (prevents double detections)
    # prominence: peak must stand out clearly (on a Z-score normalized curve)

    peaks, properties = find_peaks(norm_resp_signal, distance=95, prominence=0.03)
    ref_peaks, ref_properties = find_peaks(norm_reference, distance=95, prominence=0.1)

    # Calculations
    num_breaths = len(peaks)
    total_duration_seconds = len(norm_resp_signal) / fs  
    rpm_radar = (num_breaths / total_duration_seconds) * 60
    
    num_breaths_ref = len(ref_peaks)
    total_duration_seconds_ref = len(norm_reference) / fs
    rpm_ref = (num_breaths_ref / total_duration_seconds_ref) * 60

    # ==========================================
    # 8. Verification Display
    # ==========================================
    if save_fig:
        plt.figure(figsize=(12, 5))
        plt.plot(norm_resp_signal, label="Respiratory Signal (Radar)", color='blue')

        # Draw a marker on each detected peak
        plt.plot(peaks, norm_resp_signal[peaks], "X", color='blue', markersize=10, label="Peaks (Inspirations)")
        plt.plot(norm_reference, label="Ground Truth", color='red', alpha=0.7, linestyle='--')
        plt.plot(ref_peaks, norm_reference[ref_peaks], "X", color='red', markersize=10, label="Reference Peaks")

        plt.title(f"Automatic Detection: {rpm_radar:.1f} Breaths/min")
        plt.xlabel("Time (Frames)")
        plt.ylabel("Normalized Amplitude")
        plt.legend()
        plt.grid(True)
        
        # Ensure the 'fig' directory exists before saving
        os.makedirs("fig", exist_ok=True)
        plt.savefig(f"fig/{file_name}_RPM.png") 
        plt.close('all')

    return rpm_radar, rpm_ref, num_breaths, num_breaths_ref
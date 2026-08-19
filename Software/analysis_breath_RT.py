import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import serial
import struct
import numpy as np
import threading
import traceback
import time
import scipy.ndimage as ndimage
from collections import deque
from vmdpy import VMD

# ==========================================
# HARDWARE CONFIGURATION
# ==========================================
PORT = '/dev/cu.usbmodem0007602221481'  
BAUDRATE = 460800
HEADER = b'\xde\xca\xad\xde'
PAYLOAD_SIZE = 1024
NUM_SAMPLES = 256
DISPLAY_BINS = 100

# ==========================================
# CIR & TARGET PARAMETERS
# ==========================================
ALPHA_FAST = 0.1         
ALPHA_ENV = 0.02 
MOTION_THRESHOLD = 800      
TARGET_BIN = 19  # Fixed target at 60 cm
PHASE_PRESENCE_THRESHOLD = 900

# ==========================================
# RESPIRATION & VMD PARAMETERS
# ==========================================
ESTIMATED_FPS = 15
HISTORY_SIZE = 300
DISPLAY_MARGIN = 15
ALPHA_PHASE_SMOOTHING = 0.15
REFERENCE_THRESHOLD = 400

# ==========================================
# GLOBAL VARIABLES
# ==========================================
lock = threading.Lock()
stop_requested = False
first_sample_received = False
REFERENCE_BIN = 0  

# CIR Buffers
cir_median_buffer = deque(maxlen=3) # NEW: Buffer for median filter
raw_cir = np.zeros(NUM_SAMPLES)
smooth_cir = np.zeros(NUM_SAMPLES)
motion_cir = np.zeros(NUM_SAMPLES)
fast_amp = np.zeros(NUM_SAMPLES)
slow_amp = np.zeros(NUM_SAMPLES)

# Temporal Buffers
history_raw_target_phases = deque(maxlen=HISTORY_SIZE)
history_corrected_phases = deque(maxlen=HISTORY_SIZE)

centered_raw_target_signal = np.zeros(HISTORY_SIZE) 
centered_corrected_phase_signal = np.zeros(HISTORY_SIZE) 
vmd_respiratory_signal = np.zeros(HISTORY_SIZE) 

smooth_I_phase, smooth_Q_phase = 0.0, 0.0
first_phase_sample = False

smooth_I_ref, smooth_Q_ref = 0.0, 0.0
first_ref_sample = False

# Buffers for complex smoothing of the difference
smooth_I_diff, smooth_Q_diff = 0.0, 0.0
first_diff_sample = False

# ==========================================
# THREAD 1: RADAR PROCESSING (REAL-TIME)
# ==========================================
def read_serial():
    global raw_cir, smooth_cir, motion_cir, fast_amp, slow_amp
    global smooth_I_phase, smooth_Q_phase, first_phase_sample
    global smooth_I_ref, smooth_Q_ref, first_ref_sample
    global smooth_I_diff, smooth_Q_diff, first_diff_sample
    global first_sample_received, stop_requested
    global centered_raw_target_signal, centered_corrected_phase_signal
    global REFERENCE_BIN, cir_median_buffer
    
    try:
        ser = serial.Serial(PORT, BAUDRATE, timeout=1)
        print("Radar connected. Reading in progress...")
        ser.reset_input_buffer()
    except Exception as e:
        print(f"Connection error: {e}")
        return

    while not stop_requested:
        try:
            ser.read_until(HEADER)
            payload = ser.read(PAYLOAD_SIZE)
            
            if len(payload) == PAYLOAD_SIZE:
                iq_values = struct.unpack('<512h', payload)
                I = np.array(iq_values[0::2], dtype=float)
                Q = np.array(iq_values[1::2], dtype=float)
                
                # --- PRE-PROCESSING ---
                I = I - np.mean(I)
                Q = Q - np.mean(Q)
                I = ndimage.gaussian_filter1d(I, sigma=1.0)
                Q = ndimage.gaussian_filter1d(Q, sigma=1.0)
                
                amplitude = np.sqrt(I**2 + Q**2)

                # --- SOLUTION 2: TEMPORAL MEDIAN FILTER (SLOW-TIME) ---
                cir_median_buffer.append(amplitude)
                if len(cir_median_buffer) == 3:
                    # Apply median on the last 3 received CIRs
                    filtered_amplitude = np.median(cir_median_buffer, axis=0)
                else:
                    filtered_amplitude = amplitude

                with lock:
                    # We now use the stabilized amplitude
                    raw_cir = filtered_amplitude.copy()
                    
                    if not first_sample_received:
                        fast_amp = filtered_amplitude.copy()
                        slow_amp = filtered_amplitude.copy()
                        first_sample_received = True
                    else:
                        fast_amp = (ALPHA_FAST * filtered_amplitude) + ((1 - ALPHA_FAST) * fast_amp)
                        slow_amp = (ALPHA_ENV * filtered_amplitude) + ((1 - ALPHA_ENV) * slow_amp)
                        
                    smooth_cir = slow_amp.copy()
                    motion_cir = np.abs(fast_amp - slow_amp)
                    motion_cir[motion_cir < MOTION_THRESHOLD] = 0

                    # --- REFERENCE LOCKING ---
                    search_amp = slow_amp[:DISPLAY_BINS].copy()
                    idx_peak1 = np.argmax(search_amp)
                    upper_bound = min(DISPLAY_BINS, idx_peak1 + 5)
                    search_amp[:upper_bound] = 0 
                    
                    new_ref_bin = np.argmax(search_amp)
                    max_search_val = search_amp[new_ref_bin]
                    current_ref_val = slow_amp[REFERENCE_BIN] if REFERENCE_BIN < DISPLAY_BINS else 0

                    if max_search_val > REFERENCE_THRESHOLD:
                        if REFERENCE_BIN == 0 or current_ref_val < REFERENCE_THRESHOLD or max_search_val > current_ref_val * 1.50:
                            if new_ref_bin != REFERENCE_BIN:
                                REFERENCE_BIN = new_ref_bin
                                first_ref_sample = False
                                first_diff_sample = False 
                                history_raw_target_phases.clear()
                                history_corrected_phases.clear()
                                centered_raw_target_signal.fill(0)
                                centered_corrected_phase_signal.fill(0)

                    # --- I/Q EXTRACTION AND NOISE GATE ---
                    target_I, target_Q = I[TARGET_BIN], Q[TARGET_BIN]
                    target_motion = motion_cir[TARGET_BIN]
                    target_amp = filtered_amplitude[TARGET_BIN] # Using stabilized amplitude
                    
                    secure_ref_bin = REFERENCE_BIN if REFERENCE_BIN > 0 else TARGET_BIN
                    I_ref, Q_ref = I[secure_ref_bin], Q[secure_ref_bin]
                    
                    # PRESENCE CONDITION
                    if target_amp > PHASE_PRESENCE_THRESHOLD:
                        if not first_phase_sample:
                            smooth_I_phase, smooth_Q_phase = target_I, target_Q
                            first_phase_sample = True
                        else:
                            smooth_I_phase = (ALPHA_PHASE_SMOOTHING * target_I) + ((1 - ALPHA_PHASE_SMOOTHING) * smooth_I_phase)
                            smooth_Q_phase = (ALPHA_PHASE_SMOOTHING * target_Q) + ((1 - ALPHA_PHASE_SMOOTHING) * smooth_Q_phase)

                        if not first_ref_sample:
                            smooth_I_ref, smooth_Q_ref = I_ref, Q_ref
                            first_ref_sample = True
                        else:
                            smooth_I_ref = (ALPHA_PHASE_SMOOTHING * I_ref) + ((1 - ALPHA_PHASE_SMOOTHING) * smooth_I_ref)
                            smooth_Q_ref = (ALPHA_PHASE_SMOOTHING * Q_ref) + ((1 - ALPHA_PHASE_SMOOTHING) * smooth_Q_ref)
                            
                        raw_target_phase = np.arctan2(smooth_Q_phase, smooth_I_phase)
                        
                        if REFERENCE_BIN > 0:
                            # 1. Calculation of raw difference
                            raw_I_diff = (smooth_I_phase * smooth_I_ref) + (smooth_Q_phase * smooth_Q_ref)
                            raw_Q_diff = (smooth_Q_phase * smooth_I_ref) - (smooth_I_phase * smooth_Q_ref)
                            
                            # 2. Smoothing of complex components I_diff and Q_diff
                            if not first_diff_sample:
                                smooth_I_diff = raw_I_diff
                                smooth_Q_diff = raw_Q_diff
                                first_diff_sample = True
                            else:
                                smooth_I_diff = (ALPHA_PHASE_SMOOTHING * raw_I_diff) + ((1 - ALPHA_PHASE_SMOOTHING) * smooth_I_diff)
                                smooth_Q_diff = (ALPHA_PHASE_SMOOTHING * raw_Q_diff) + ((1 - ALPHA_PHASE_SMOOTHING) * smooth_Q_diff)
                            
                            # 3. Extraction of the cleaned angle
                            corrected_phase = np.arctan2(smooth_Q_diff, smooth_I_diff)
                        else:
                            corrected_phase = raw_target_phase
                    else:
                        first_phase_sample = False
                        first_ref_sample = False
                        first_diff_sample = False 
                        raw_target_phase = 0.0
                        corrected_phase = 0.0
                        
                    history_raw_target_phases.append(raw_target_phase)
                    history_corrected_phases.append(corrected_phase)
                    
                    if len(history_corrected_phases) > 50:
                        target_phases_array = np.array(history_raw_target_phases)
                        target_phases_unwrapped = np.unwrap(target_phases_array)
                        centered_raw_target_signal = target_phases_unwrapped - np.mean(target_phases_unwrapped)

                        corr_phases_array = np.array(history_corrected_phases)
                        corr_phases_unwrapped = np.unwrap(corr_phases_array)
                        centered_corrected_phase_signal = corr_phases_unwrapped - np.mean(corr_phases_unwrapped)

        except Exception:
            pass
            
    ser.close()

# ==========================================
# THREAD 2: VMD ENGINE (ASYNCHRONOUS)
# ==========================================
def asynchronous_vmd_processing():
    global vmd_respiratory_signal, stop_requested
    
    alpha = 2000       
    tau = 0.           
    K = 3              
    DC = 0             
    init = 1           
    tol = 1e-7         
    
    while not stop_requested:
        time.sleep(0.2) 
        
        with lock:
            if len(history_corrected_phases) < 100:
                continue
            phases_to_process = np.array(centered_corrected_phase_signal)
            
        if np.max(np.abs(phases_to_process)) < 0.01:
            with lock:
                vmd_respiratory_signal = np.zeros(len(phases_to_process))
            continue
            
        try:
            u, u_hat, omega = VMD(phases_to_process, alpha, tau, K, DC, init, tol)
            frequencies_hz = omega[-1] * ESTIMATED_FPS
            resp_idx = np.argmin(np.abs(frequencies_hz - 0.3))
            respiration_mode = u[resp_idx, :]
            
            with lock:
                vmd_respiratory_signal = respiration_mode
                
        except Exception as e:
            pass

# ==========================================
# GUI (MATPLOTLIB DASHBOARD)
# ==========================================
fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, 1, figsize=(10, 12))
fig.canvas.manager.set_window_title('UWB Radar - VMD Processing Dashboard')

# --- PLOT 1: CIR ---
line_raw, = ax1.plot(range(DISPLAY_BINS), np.zeros(DISPLAY_BINS), color='lightgray', linewidth=1, label='Raw CIR', alpha=0.5)
line_smooth, = ax1.plot(range(DISPLAY_BINS), np.zeros(DISPLAY_BINS), color='blue', linewidth=2, label='Static Env', alpha=0.7)
line_motion, = ax1.plot(range(DISPLAY_BINS), np.zeros(DISPLAY_BINS), color='red', linewidth=2, label='Motion')

line_target = ax1.axvline(x=TARGET_BIN, color='green', linestyle='--', label=f'Target Bin ({TARGET_BIN})')
line_ref = ax1.axvline(x=0, color='orange', linestyle='--', label='Ref Bin (Locked)')

ax1.set_ylim(0, 8000)
ax1.set_xlim(0, DISPLAY_BINS)
ax1.set_title("Spatial Analysis: CIR (Green: Fixed Target, Orange: Locked Reference)")
ax1.set_ylabel("Amplitude")
ax1.legend(loc="upper right")
ax1.grid(True, linestyle='--', alpha=0.5)

# --- PLOT 2: Raw Target Phase ---
line_raw_target, = ax2.plot(np.zeros(HISTORY_SIZE), color='magenta', linewidth=2)
ax2.set_xlim(0, HISTORY_SIZE)
ax2.set_ylim(-20, 20)
ax2.set_title(f"Temporal Analysis: Raw Phase of Target (Bin {TARGET_BIN})")
ax2.set_ylabel("Phase (Rads)")
ax2.grid(True, linestyle='--', alpha=0.5)

# --- PLOT 3: Corrected Phase (Target - Ref) ---
line_corr_phase, = ax3.plot(np.zeros(HISTORY_SIZE), color='purple', linewidth=2)
ax3.set_xlim(0, HISTORY_SIZE)
ax3.set_ylim(-10, 10)
ax3.set_title("Temporal Analysis: Corrected Phase (Phase Referencing)")
ax3.set_ylabel("Phase (Rads)")
ax3.grid(True, linestyle='--', alpha=0.5)

# --- PLOT 4: VMD Respiration Signal ---
line_resp, = ax4.plot(np.zeros(HISTORY_SIZE), color='darkorange', linewidth=2, label='VMD IMF (Respiration Mode)')
ax4.set_xlim(0, HISTORY_SIZE - DISPLAY_MARGIN)
ax4.set_ylim(-5, 5)
ax4.set_title("Temporal Analysis: VMD Extracted Respiration Signal")
ax4.set_xlabel("Time (frames)")
ax4.set_ylabel("Phase (Rads)")
ax4.legend(loc="upper right")
ax4.grid(True, linestyle='--', alpha=0.5)

plt.tight_layout()

# ==========================================
# ANIMATION LOOP
# ==========================================
def update_plot(frame):
    with lock:
        line_raw.set_ydata(raw_cir[:DISPLAY_BINS])
        line_smooth.set_ydata(smooth_cir[:DISPLAY_BINS])
        line_motion.set_ydata(motion_cir[:DISPLAY_BINS])
        line_ref.set_xdata([REFERENCE_BIN, REFERENCE_BIN])
        
        label_ref = f'Ref Bin ({REFERENCE_BIN})' if REFERENCE_BIN > 0 else 'Ref Bin (No Ref > 400)'
        line_ref.set_label(label_ref)
        ax1.legend(loc="upper right")

        def pad_signal(signal_in):
            current_size = len(signal_in)
            return np.pad(signal_in, (0, HISTORY_SIZE - current_size), 'constant', constant_values=0)

        line_raw_target.set_ydata(pad_signal(centered_raw_target_signal))
        line_corr_phase.set_ydata(pad_signal(centered_corrected_phase_signal))
        
        data_to_display = np.copy(vmd_respiratory_signal)
        if len(history_corrected_phases) > DISPLAY_MARGIN * 2:
            clean_data = data_to_display[:-DISPLAY_MARGIN]
        else:
            clean_data = data_to_display
            
        line_resp.set_ydata(pad_signal(clean_data))
        
    return line_raw, line_smooth, line_motion, line_target, line_ref, line_raw_target, line_corr_phase, line_resp

if __name__ == '__main__':
    serial_thread = threading.Thread(target=read_serial)
    serial_thread.daemon = True
    serial_thread.start()
    
    thread_vmd = threading.Thread(target=asynchronous_vmd_processing)
    thread_vmd.daemon = True
    thread_vmd.start()

    try:
        print("Starting graphical interface...")
        ani = animation.FuncAnimation(fig, update_plot, interval=50, blit=False)
        plt.show()
    except KeyboardInterrupt:
        print("\nStop requested by user.")
    finally:
        stop_requested = True
        serial_thread.join(timeout=1)
        thread_vmd.join(timeout=1)
        print("Program terminated properly.")
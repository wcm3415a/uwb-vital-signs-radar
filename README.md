# UWB Radar for Contactless Vital Sign Detection

![Project Status](https://img.shields.io/badge/Status-Completed-success)
![Language](https://img.shields.io/badge/Language-C++%20|%20Python-blue)
![Hardware](https://img.shields.io/badge/Hardware-DWM1001--DEV-orange)

## 📋 Overview
This repository contains the hardware configuration, firmware, and signal processing software for a contactless physiological micro-movement detection system. The project repurposes low-cost Ultra-Wideband (UWB) modules (DWM1001-DEV) into a highly sensitive radar capable of extracting human respiratory rates through obstacles. 

This research was conducted as part of an engineering internship at the **HCR Laboratory (Robotics and Embedded Systems)** at **Ostfalia University of Applied Sciences, Germany**.

## ✨ Key Features
* **Bistatic Radar Configuration:** Utilizes two asynchronous DWM1001-DEV boards (TX/RX) to extract the raw Channel Impulse Response (CIR)[cite: 3].
* **Custom RF Shielding:** Includes 3D-printable CAD files for a test bench lined with aluminum to eliminate direct line-of-sight antenna coupling (hardware saturation)[cite: 3].
* **High-Speed Data Acquisition:** Multithreaded Python architecture capable of parsing a 460,800-baud serial stream in real-time without packet loss[cite: 3].
* **Advanced Signal Processing (DSP):**
  * Static environment filtering via Exponential Moving Average (Clutter Removal)[cite: 3].
  * Sub-millimeter spatial targeting and Phase Extraction (`arctan(Q/I)`)[cite: 3].
  * Physiological mode separation using **Variational Mode Decomposition (VMD)**[cite: 3].

## 🛠️ Hardware Architecture
The system is built on the Decawave DW1000 UWB transceiver. Instead of using the factory Time-of-Flight (ToF) localization firmware, the chips are flashed with a custom low-level C++ firmware developed via PlatformIO[cite: 3].

* **Transmitter (TX):** Configured on Channel 5 (6.5 GHz) with a 64 MHz Pulse Repetition Frequency (PRF), broadcasting dummy frames at 20 Hz[cite: 3].
* **Receiver (RX):** Dynamically tracks the First Path Index, steps back by 10 samples to capture the rising edge of the echo, and extracts a 1024-byte accumulator memory window (CIR)[cite: 3]. SPI communication is intentionally throttled to 1 MHz to ensure data integrity[cite: 3].

## 💻 Software & DSP Pipeline
The real-time data processing is handled by a Python application implementing the following pipeline:

1. **Serial Synchronization:** Detects the `0xDECAADDE` hexadecimal header to align and extract payload frames asynchronously[cite: 3].
2. **Background Subtraction:** Applies an Exponential Moving Average to dynamically map and subtract the static room environment (walls, furniture) from the raw CIR[cite: 3].
3. **Phase Unwrapping:** Locks onto a fixed spatial range bin (e.g., 60 cm away, accounting for a 19-bin hardware delay) to extract the complex phase of the wave, magnifying millimeter-scale chest displacements[cite: 3].
4. **VMD Extraction:** Decomposes the phase signal into Intrinsic Mode Functions (IMFs). The algorithm enforces strict frequency filtering (high penalty factor α) to isolate the respiratory band (0.2 - 0.5 Hz) from background noise[cite: 3].

## 📊 Results & Limitations
The software processing chain was preliminary validated against the medical-grade **MobiVital** dataset, achieving a respiratory rate estimation error of **< 8%**[cite: 3].

**Real-World Prototype Performance:**
* **Respiration:** Successfully extracts stable, highly accurate inhalation/exhalation cycles in real-world static setups[cite: 3].
* **Heart Rate:** The system reached the physical limitations of the asynchronous DWM1001-DEV boards. Hardware clock drift between the TX and RX modules introduces phase noise (Jitter) that entirely masks the sub-millimeter cardiac displacement (0.1 mm). True heart rate extraction requires a hard-wired clock synchronization architecture[cite: 3].

## 📁 Repository Structure
```text
├── Firmware/                 # C++ PlatformIO project for TX/RX modules
│   ├── src/main.cpp          # Core radar logic and SPI extraction
│   └── lib/                  # DW1000 low-level API headers
├── Software/                 # Python real-time acquisition and DSP pipeline
│   ├── serial_reader.py      # Multithreaded acquisition thread
│   ├── dsp_pipeline.py       # VMD, Clutter removal, and Phase extraction
│   └── main_gui.py           # Real-time visualization interface
├── Hardware/                 # 3D models (STL/Fusion 360) for the RF shield
└── Docs/                     # Additional documentation and test data
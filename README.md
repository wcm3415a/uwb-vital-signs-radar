# UWB Radar for Contactless Vital Sign Detection

![Project Status](https://img.shields.io/badge/Status-Completed-success)
![Language](https://img.shields.io/badge/Language-C++%20|%20Python-blue)
![Hardware](https://img.shields.io/badge/Hardware-DWM1001--DEV-orange)

## 📋 Overview
This repository contains the hardware configuration, firmware, and signal processing software for a proof-of-concept UWB radar system. The objective was to design a contactless Ultra-Wideband (UWB) radar system to detect vital signs for future integration into a mobile robot. This research relies on repurposing low-cost DWM1001-DEV modules into a bistatic radar to extract and analyze the Channel Impulse Response (CIR).

This project was conducted as part of a 4th-year engineering internship at the **HCR Laboratory (Robotics and Embedded Systems)** at **Ostfalia University of Applied Sciences, Germany**.

## 📚 Project Documentation (Wiki)
Comprehensive documentation covering the theoretical background, hardware setup, and software architecture is available in the **[Project Wiki](https://github.com/wcm3415a/uwb-vital-signs-radar/wiki)**:
* 📖 **[Hardware Architecture and Test Bench](https://github.com/wcm3415a/uwb-vital-signs-radar/wiki/Hardware-and-Test-Bench):** Details on the bistatic configuration and the 3D-printed electromagnetic shield.
* 📖 **[Firmware and SPI Communication](https://github.com/wcm3415a/uwb-vital-signs-radar/wiki/Firmware-and-SPI):** C++ implementation, register modifications, and SPI throttling.
* 📖 **[DSP Theory and VMD Algorithm](https://github.com/wcm3415a/uwb-vital-signs-radar/wiki/Signal-Processing):** Mathematical foundations of phase extraction and Variational Mode Decomposition.
* 📖 **[Real-Time Python Application](https://github.com/wcm3415a/uwb-vital-signs-radar/wiki/Python-Application):** Multithreaded architecture for high-speed serial parsing and real-time visualization.

## 🛠️ System Architecture
* **Firmware (C++):** A custom low-level firmware developed via PlatformIO bypasses the factory localization features. The transmitter operates at 64 MHz PRF (Channel 5), while the receiver continuously extracts the CIR. SPI communication is throttled to 1 MHz to prevent data corruption.
* **Hardware Mitigation:** A custom 3D-printed shield lined with aluminum prevents receiver saturation caused by direct antenna coupling.
* **Software Pipeline (Python):** An asynchronous multithreaded application parses the 460,800-baud serial stream. It applies an Exponential Moving Average for radar clutter removal, extracts the complex phase (`arctan(Q/I)`), and uses Variational Mode Decomposition (VMD) to isolate physiological frequencies.

## 📊 Experimental Results and Hardware Limitations
The software processing chain was successfully validated on the MobiVital reference dataset, achieving a respiratory rate estimation error of less than 8%. However, real-world testing on the physical prototype revealed strict hardware limitations:

* **Respiration Extraction:** The system successfully extracts the respiratory signal, allowing the distinction between inspiration and expiration cycles. However, the measurement is affected by a permanent residual noise that cannot be fully compensated by software, occasionally falsifying parts of the obtained results.
* **Heart Rate Extraction (Failed):** Heartbeat extraction failed due to absolute physical limitations. The lack of strict hardware clock synchronization between the two asynchronous boards induces a phase noise (Jitter). This electronic drift completely masks the sub-millimeter cardiac displacement, rendering it unexploitable. Future iterations will strictly require hard-wired clock synchronization.

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
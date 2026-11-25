# IMU Attitude Estimation and Sensor Fusion Analysis

**Author:** Choongmin Jeong  
**Affiliation:** School of Electrical and Electronics Engineering, Chung-Ang University  
**Project:** 2026 Capstone Design Project  
**Location:** Seoul, Republic of Korea

---

## 1. Project Overview

This project presents an embedded real-time attitude estimation system using an STM32 microcontroller and a 9-axis IMU sensor.

The primary objective is to implement and compare four sensor fusion algorithms under identical experimental conditions and evaluate their estimation accuracy, disturbance robustness, response characteristics, and computational efficiency.

A custom-built rotational test platform and a Python-based GUI were developed to provide an integrated environment for experimental evaluation and real-time 3D attitude visualization.

## 2. Hardware Configuration

- STM32 NUCLEO-F446RE Microcontroller
- ICM-20948 9-axis IMU
- A4988 Stepper Motor Driver
- Stepper Motor
- Vibration Motor
- Custom 360° Acrylic Rotational Test Platform

**Communication Interfaces:** I²C, USART2, USB Virtual COM Port

**Nominal Sampling Frequency:** 60 Hz

## 3. Sensor Fusion Algorithms

Four attitude estimation algorithms are implemented and executed on the STM32 microcontroller using identical sensor measurements.

- Quaternion-Based Extended Kalman Filter (EKF)
- Complementary Filter
- Mahony Filter
- AQUA Filter

The firmware incorporates sensor calibration, quaternion-based attitude estimation, and disturbance rejection techniques.

## 4. Experimental Evaluation

The algorithms are evaluated under four experimental conditions:

1. Static angular accuracy
2. Rapid rotational motion
3. Vibration disturbance
4. Linear acceleration disturbance

The performance evaluation includes:

- Mean, maximum, and RMS estimation errors
- Attitude stability
- Dynamic response characteristics
- Algorithm execution time
- CPU utilization
- RAM and Flash memory usage

## 5. Python GUI

A custom Python-based graphical user interface provides the following features:

- Real-time 3D attitude visualization
- Interactive sensor calibration
- Hardware and experimental control
- Simultaneous comparison of four sensor fusion algorithms
- Automatic performance evaluation and filter selection
- Experimental data recording and result visualization
- Comparison between the selected filter and gyroscope-only attitude estimation

## 6. Firmware Development History

| Version | Development Stage |
|---|---|
| V1.0 | Initial implementation |
| V1.1 | Attitude estimation refinement |
| V1.4 | Estimation accuracy improvement |
| V1.5 | Algorithm and calibration refinement |
| V1.7_Final | Final optimized implementation |

## 7. Repository Structure

| Directory | Description |
|---|---|
| STM32_Firmware | Firmware source code and development history |
| Python_GUI | GUI source code, visualization assets, and experimental data |
| Hardware | Hardware design, system configuration, and prototype images |
| References | Research papers and official hardware datasheets |

## 8. Development Environment

- STM32CubeIDE
- STM32CubeMX
- C
- Python
- PySide6
- VTK

## 9. Acknowledgments

Special thanks to the following researchers for their valuable technical advice and insights during the development of this project:

- **Jean-Michel Pflimlin** — Mahony Filter and linear acceleration disturbance rejection
- **Guilherme Augusto Silva Pereira** — Quaternion-Based Extended Kalman Filter

---

**Developed by Choongmin Jeong**  
School of Electrical and Electronics Engineering  
Chung-Ang University, Republic of Korea  
November 2026

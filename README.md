# Tercio S1 Firmware

<div align="center">

# TERCIO S1
**High-Performance Closed-Loop Stepper Driver | CAN-FD Ecosystem**

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![Documentation](https://img.shields.io/badge/Docs-terciolabs.com-green)](https://docs.terciolabs.com)
[![Hardware](https://img.shields.io/badge/Hardware-TercioLabs-orange)](https://terciolabs.com)

</div>

---

## Hardware Availability

This repository contains the firmware source code for the Tercio S1 Stepper Driver.
**Official hardware is available for purchase at [terciolabs.com](https://terciolabs.com).**

---

## 🚀 Project Overview

**Tercio S1** is a compact, engineering-grade closed-loop stepper driver designed for robotics and automation enthusiasts who demand precision. It features a robust CAN-FD architecture for low-latency communication and integrates seamlessly with the Tercio ecosystem.

### Key Features
* **Communication:** Low-latency 2.5 Mbps CAN-FD for real-time control.
* **Control:** Closed-loop FOC (Field Oriented Control) for smooth, accurate motion.
* **Usability:** Built-in Web Control GUI for configuration (no coding required for basic setup).
* **Telemetry:** Real-time feedback on motor position, velocity, and torque.

---

## 🛠️ Development & Flashing

### Hardware Requirements
To develop or flash firmware on the S1, you will need:
1.  **Tercio S1 Driver**
2.  **Tercio FD Adapter** (Required for CAN-FD communication)
3.  **ST-Link V2/V3 Programmer** (for flashing via SWD)
4.  **Wiring:** SWD connections (SWDIO, SWCLK, GND, 3V3)

### Development Environment
You can build and flash this project using either **PlatformIO** (VSCode) or **STM32CubeIDE**.

#### Option A: PlatformIO (Recommended)
1.  Install [Visual Studio Code](https://code.visualstudio.com/) and the **PlatformIO** extension.
2.  Clone this repository:
    ```bash
    git clone [https://github.com/Kerol-Dev/Tercio-s1.git](https://github.com/Kerol-Dev/Tercio-s1.git)
    ```
3.  Open the project folder in PlatformIO.
4.  Connect your ST-Link to the S1 board via the SWD headers.
5.  Click the **PlatformIO: Upload** button (arrow icon) to build and flash.

#### Option B: STM32CubeIDE
1.  Open **STM32CubeIDE**.
2.  Select `File > Open Projects from File System...` and navigate to the repository folder.
3.  Connect your ST-Link to the S1 board.
4.  Build the project (Hammer icon).
5.  Run/Debug (Play/Bug icon) to flash the firmware.

---

## 📚 Documentation

For complete wiring diagrams, pinouts, and API references, please consult the official documentation:
**[👉 docs.terciolabs.com](https://docs.terciolabs.com)**

---

## 🤝 Contributing

We welcome contributions from the community!
If you would like to improve the firmware or add new features:
1.  Fork the repository.
2.  Create a feature branch (`git checkout -b feature/AmazingFeature`).
3.  Commit your changes.
4.  Open a Pull Request.

---

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

<div align="center">
  <sub>Copyright © Tercio Labs</sub>
</div>

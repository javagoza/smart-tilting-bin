
# Raspberry Pi CM4 SG90 Servo Library

A lightweight Python module to drive an **SG90 9g micro servo motor** using Pulse Width Modulation (PWM) on a **Raspberry Pi Compute Module 4 (CM4)**.

## Hardware Wiring

Connect your SG90 servo wires to the CM4 GPIO header:
* **Brown / Black Wire (GND)** ──> Physical Pin (e.g., Pin 6 / GND)
* **Red Wire (VCC / 5V)** ──────> Physical Pin 2 or 4 (5V Power)
  * *Note: For heavy loads or multiple servos, power them from an external 5V power supply with a shared common ground.*
* **Orange / Yellow Wire (PWM)** ─> BCM Pin 18 (Physical Pin 12)

---

## Installation & Dependencies

1. Ensure Python 3 and pip are installed on your system.
2. Install the GPIO library compatible with your Raspberry Pi OS version:
   * **For Raspberry Pi OS (Bullseye / Bookworm with `liblgpio`):**
     ```bash
     sudo apt-get install python3-rpi-lgpio
     ```
   * **Or via pip:**
     ```bash
     pip install rpi-lgpio
     ```

---

## Usage Example

Run the provided sweep demo:
```bash
python3 demo.py

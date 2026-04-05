# 🤖 FaceTracker: Standalone ESP32-CAM Pan-Tilt System

**FaceTracker** is an autonomous **Edge Computing** solution for real-time computer vision. By integrating an **ESP32-CAM** with a **PCA9685 PWM controller**, the system performs onboard **facial centroid tracking**—translating visual coordinates directly into physical motion without the need for external processing or Wi-Fi.

---

<table width="100%">
  <tr>
    <td width="70%" align="left" valign="middle">
      <h2>🚀 The Backstory: Edge AI</h2>
    </td>
    <td width="30%" align="center" valign="middle">
      <img src="https://raw.githubusercontent.com/MatsRobot/Face-Tracking-using-ESP32-CAM/main/FaceTracker.gif" alt="ESP32-CAM Face Tracking Hardware Configuration" width="150" style="border-radius: 8px;" />
    </td>
  </tr>
  <tr>
    <td colspan="2">
      <p>
        Most ESP32-CAM implementations are limited to "CameraWebServer" streams. This project moves the intelligence to the <b>Edge</b>, allowing the microcontroller to scan, detect, and act autonomously.
      </p>
      <p>
        By eliminating the latency of a web browser middleman, the system calculates <b>positional error vectors</b> in real-time. The result is a self-contained robotic sensor that centers on human subjects using high-speed <b>I2C communication</b>, making it ideal for mobile robotics and interactive animatronics.
      </p>
    </td>
  </tr>
</table>

---

## ✨ Key Technical Features

* **Autonomous Centroid Tracking:** Onboard processing calculates the **X/Y displacement** required to maintain target alignment at the frame center.
* **Low-Latency Edge AI:** Uses the **MTMN (Multi-Task Cascaded Convolutional Networks)** model for face detection without cloud or app dependency.
* **Hardware-Accelerated PWM:** Offloads servo pulses to a PCA9685 via **400kHz Fast-Mode I2C**, ensuring 12-bit resolution and jitter-free movement.
* **Real-Time Telemetry:** Local **SSD1306 OLED** integration provides instant feedback on coordinate mapping and system health.
* **Optimized Bus Architecture:** Maximizes the ESP32-CAM's limited GPIO by daisy-chaining all peripherals on a single I2C bus.

## 🛠️ Hardware Stack

* **ESP32-CAM (AI-Thinker):**
    * Dual-core LX6 CPU @ 240MHz.
    * Integrated **OV2640** image sensor.
* **PCA9685 16-Channel PWM Driver:**
    * 12-bit resolution for precise angular servo increments.
    * Addresses I2C bus bottlenecking by managing pulse timing independently.
* **Actuators:** Dual SG90/MG90S Micro Servos for Pan and Tilt.
* **Display:** SSD1306 OLED for coordinate debugging.

---

## 🔌 Wiring & Pinout (I2C Configuration)

All external peripherals are synchronized via the **I2C Serial Bus** to conserve GPIO pins for the camera interface.

| Peripheral | ESP32-CAM Pin | Function | Protocol |
| :--- | :--- | :--- | :--- |
| **PCA9685** | GPIO 14 | SDA | I2C Data |
| **PCA9685** | GPIO 15 | SCL | I2C Clock |
| **SSD1306 OLED** | GPIO 14 | SDA | Shared Bus |
| **SSD1306 OLED** | GPIO 15 | SCL | Shared Bus |
| **Logic Power** | 5V / GND | VCC / GND | Logic Supply |

> [!IMPORTANT]
> **Servo Power:** Always use a dedicated 5V power supply for the PCA9685/Servos. Drawing servo current through the ESP32-CAM will cause brownouts and core resets.

---

## 📐 Logic & Control Loop

### Centroid Mapping
The system identifies facial landmarks and determines the **Error Vector** relative to the QVGA resolution center ($160, 120$).

* **The Goal:** Minimize the delta between the detected face centroid and the frame origin.
* **The Action:** The ESP32 calculates a proportional correction and updates the **PCA9685 registers** to adjust the servo duty cycle.

### Data Pipeline
1. **Acquisition:** Frame capture via OV2640 DMA.
2. **Inference:** MTMN model detects faces and produces bounding box coordinates.
3. **Control:** Software calculates $X, Y$ offsets and generates I2C commands.
4. **Feedback:** Coordinates are pushed to the OLED and Serial Monitor.
5. **Actuation:** Servos pivot to center the target within the frame.

---

## ⚡ Quick Start

1. **Hardware:** Assemble the circuit using the shared I2C pinout.
2. **Project Initialization:**
    * Create a new project in **PlatformIO**.
    * Select **AI Thinker ESP32-CAM** as the board.
3. **Critical Configuration Steps:** This project uses a legacy framework (Espressif v1.0.4) that requires manual file handling to resolve missing `fd_forward.h` and `ADS1115-Driver` errors. Complete these **three steps** before attempting to compile:
    * **1. Replace `platformio.ini`:** Overwrite the default file in your project root with the specific configuration provided (ensuring it uses `platform = espressif32@1.12.4`).
    * **2. Replace `main.cpp`:** Copy your code into the `src` folder, replacing the existing boilerplate file.
    * **3. Manual Library Install:** Copy the `ADS1115_Driver-1.0.2` and `esp32cam` folders into the project’s **`lib`** folder. This allows the missing legacy files to be available for the compiler.
4. **Calibration:** Define `SERVO_MIN` and `SERVO_MAX` within the code for physical calibration.
5. **Build & Upload:** Connect your ESP32-CAM via an FTDI adapter and click **Upload** in PlatformIO.

---

### 📂 Required Folder Structure
To ensure the compiler sees your files, your project directory should look like this:

```text
Your_Project_Folder/
├── lib/
│   ├── ADS1115_Driver-1.0.2/
│   └── esp32cam/            <-- Contains legacy fd_forward.h
├── src/
│   └── main.cpp             <-- Your actual code
└── platformio.ini           <-- The specific v1.12.4 config
```
---


## 🔍 Troubleshooting

* **I2C Communication:** If the OLED or PCA9685 is not detected, verify that GPIO 14/15 are not being pulled high/low by other peripherals.
* **FPS Drop:** Ensure "Core Debug Level" is set to "None" in the IDE to maximize CPU cycles for detection logic.
* **Servo Jitter:** Confirm the ground (GND) is common between the ESP32-CAM and the external servo power supply.

## 🗺️ Future Roadmap

* **ESP32-S3 Port:** Leveraging vector instructions for higher frame rates.
* **PID Implementation:** Replacing simple proportional tracking with a full **Proportional-Integral-Derivative** loop for fluid motion.
* **Distance Logic:** Using bounding box area to estimate target distance and adjust tracking speed dynamically.

---

<small>© 2026 MatsRobot | Licensed under the [MIT License](https://github.com/MatsRobot/matsrobot.github.io/blob/main/LICENSE)</small>

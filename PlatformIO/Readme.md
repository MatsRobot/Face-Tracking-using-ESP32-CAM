# 🤖 Comprehensive Setup Guide: FaceTracker for ESP32-CAM

This guide provides a detailed walkthrough for setting up a **Portable VS Code** environment to compile and upload the FaceTracker system. This method keeps your installation self-contained on your local drive.

---

## 📥 1. Install Portable VS Code
1.  **Download:** Visit the VS Code website and download the **Windows 64-bit .zip** version.
2.  **Extract:** Move the `.zip` file to your local drive (e.g., `C:\FaceTracker_IDE`) and extract it there.
3.  **Enable Portable Mode:**
    * Navigate into the extracted folder.
    * Create a new folder named **`data`** inside the main directory. This ensures all settings and extensions stay within this folder rather than spreading to your system.
4.  **Launch:** Run `Code.exe` to open your portable editor.

---

## 🧩 2. Install Required Extensions
You need specific tools to communicate with the ESP32-CAM and other microcontrollers.

1.  **PlatformIO IDE:**
    * Click the **Extensions** icon on the left sidebar (looks like four squares).
    * Search for `PlatformIO IDE` and click **Install**. 
    * Wait for the **Ant Icon** to appear on the left sidebar, signifying the core tools are ready.
2.  **MicroPico:**
    * In the Extensions search bar, type `MicroPico`.
    * Click **Install**. This extension is used for managing Raspberry Pi Pico or general Python-based hardware tasks.

---

## 📂 3. Download Project Files
Before creating a project, obtain the specific legacy files required for this system.

1.  **GitHub Download:** Go to the FaceTracker Repository.
2.  **Get ZIP:** Click the green **Code** button and select **Download ZIP**.
3.  **Unpack:** Extract the ZIP folder to your desktop or a dedicated workspace.

---

## 🚀 4. Create a New PlatformIO Project
This initializes the workspace for the AI Thinker hardware.

1.  Click the **Ant Icon** (PlatformIO) on the left sidebar.
2.  Select **PIO Home > Open**.
3.  Click the **+ New Project** button.
4.  **Configure as follows:**
    * **Name:** `FaceTracker_Project`
    * **Board:** Search for and select `AI Thinker ESP32-CAM`.
    * **Framework:** Select `Arduino`.
    * **Location:** You may use the default or choose a specific folder on your drive.
5.  Click **Finish**. It may take a moment to initialize the project structure.

---

## 📝 5. Manual File Configuration (Crucial)
To avoid errors like "missing `fd_forward.h`," you must manually replace the default files with the downloaded versions.

1.  **Replace `platformio.ini`:** Copy the `platformio.ini` file from your GitHub download and paste it into your new project folder, overwriting the existing one.
2.  **Replace `main.cpp`:** Copy `main.cpp` from the downloaded `src` folder. Paste it into the `src` folder of your new project.
3.  **Install Libraries:**
    * Locate the `lib` folder in your GitHub download.
    * Copy the `ADS1115_Driver-1.0.2` and `esp32cam` folders.
    * Paste both folders into the **`lib`** directory of your new PlatformIO project.

**Your project structure should look like this:**
```text
FaceTracker_Project/
├── lib/
│   ├── ADS1115_Driver-1.0.2/
│   └── esp32cam/            <-- Contains legacy headers
├── src/
│   └── main.cpp             
└── platformio.ini
```
---

## 🔌 6. Compile and Upload

* **Open Project:** In VS Code, go to **File > Open Folder** and select your `FaceTracker_Project` folder.
* **Connect Hardware:** Connect your ESP32-CAM via an FTDI adapter. Ensure **GPIO 0** is connected to **GND** to enable flashing mode.
* **Build:** Click the **Checkmark (✔)** icon in the blue bottom status bar to compile the code.
* **Upload:** Click the **Right Arrow (→)** icon next to it to send the code to the microcontroller.
* **Run:** Once finished, disconnect **GPIO 0** from **GND** and press the **Reset** button on the ESP32-CAM to start the tracking logic.

## 🔍 Quick Troubleshooting

* **Failed to connect:** Check your COM port. You may need to install the CP210x or CH340 drivers for your FTDI adapter.
* **Brownout Error:** Ensure your servos are powered by an external 5V source, not the ESP32-CAM itself.

---

<small>© 2026 MatsRobot | Licensed under the [MIT License](https://github.com/MatsRobot/matsrobot.github.io/blob/main/LICENSE)</small>

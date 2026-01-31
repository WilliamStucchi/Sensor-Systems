# Sensor Systems Projects (Polimi)

This repository contains a collection of **laboratory exercises and homework assignments** for the **Sensor Systems** course at **Politecnico di Milano**.  
The projects are developed for the **STM32 NUCLEO-F401RE** board using the **STM32CubeIDE** ecosystem.

The code explores **embedded systems programming**, sensor interfacing, and the use of microcontroller peripherals such as **GPIO, Timers, ADC, DMA, UART, and I2C**.

A comprehensive set of **Embedded C projects** demonstrating low-level interfacing and data processing on the **STM32F401RE (ARM Cortex-M4)** microcontroller.

---

## 🛠 Hardware & Tools

- **Microcontroller:** STM32F401RE (ARM Cortex-M4)
- **Development Board:** NUCLEO-F401RE
- **IDE:** STM32CubeIDE / STM32CubeMX
- **Language:** Embedded C

---

## 📂 Project Categories

### 1. Basic Interfacing & Interrupts

- **GPIO & Timers**  
  LED control via pushbuttons and non-blocking delays using timer interrupts.

- **External Interrupts**  
  Triggering events (e.g., starting a song) based on sensor inputs or button presses.

---

### 2. Audio & Signals

- **Tone Generation**  
  Playing tones and melodies (e.g., *Io Credo In Me*) using PWM and timers.

- **Signal Handling**  
  Non-blocking audio playback and microphone-triggered events.

---

### 3. Peripherals & Protocols

- **ADC (Analog-to-Digital Converter)**  
  Reading values from potentiometers, LDRs, and internal temperature sensors using:
  - Polling
  - Interrupts
  - DMA

- **I2C**  
  Interfacing with temperature sensors and accelerometers to acquire environmental and motion data.

- **UART**  
  Sending sensor data and receiving commands from a PC terminal via serial communication.

- **DMA (Direct Memory Access)**  
  High-efficiency data transfer from ADC and sensors to memory or UART with minimal CPU overhead.

---

### 4. Displays & Input Devices

- **LCD**  
  Interfacing with character LCDs to display text, sensor values, and custom messages.

- **LED Matrix**  
  Mapping letters and patterns to an LED matrix display.

- **Keyboards & Encoders**  
  Scanning matrix keyboards and reading rotary encoder signals for user input.

- **IR Communication**  
  Receiving and decoding signals from Infrared (IR) remote controllers.

---

## 🚀 Getting Started

### Prerequisites
- Install **STM32CubeIDE**

### Clone the Repository

```bash
git clone https://github.com/WilliamStucchi/Sensor-Systems.git
```

### Importing Projects

1. Open **STM32CubeIDE**
2. Go to **File → Import → Existing Projects into Workspace**
3. Select the specific homework or lab folder you wish to run

### Flash to Board

- Connect your **NUCLEO-F401RE** via USB
- Click **Run** (`Ctrl + F11`)

---

## 👥 Contributors

- **William Stucchi**
- **Course:** Sensor Systems @ Politecnico di Milano

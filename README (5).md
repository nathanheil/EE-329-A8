# EE 329 – Lab A6: ADC Sampling with UART Output (STM32L4)

This project implements an **ADC data acquisition and display system** on the STM32L4A6ZG Nucleo board.  
It samples an analog input on **PA0 (ADC1_IN5)**, computes statistics across multiple samples, and streams results over **LPUART1** to a terminal.  
Relay and LED GPIO control are also included for additional hardware interaction.

---

## 🎯 Project Overview
- **Analog Input:** PA0 configured as ADC1 channel 5  
- **Sampling:** Collects 20 samples using ADC interrupt-driven conversions  
- **Data Processing:** Computes **minimum, maximum, and average** values  
- **Conversions:**  
  - Raw ADC counts → calibrated millivolts  
  - Raw ADC counts → current (µA/mA) using a shunt resistor  
- **UART Output:** Results are formatted and printed to a terminal over LPUART1  
  - MIN / MAX / AVG voltages  
  - Calculated **coil current**  
- **GPIO Control:**  
  - **Relay** output on PB1  
  - **LED** output on PC0  
  - **Button input** on PC13  

---

## 🧩 Source Files
- `main.c / main.h` – System setup, StageOne screen init, ADC statistics, GPIO config【159†source】【160†source】  
- `ADC.c / ADC.h` – ADC1 initialization, ISR handler, conversions, and data formatting【153†source】【154†source】  
- `lpuart.c / lpuart.h` – LPUART1 initialization, print helpers, and escape-sequence output【157†source】【158†source】  
- `delay.c / delay.h` – SysTick microsecond delay implementation【155†source】【156†source】  

---

## ⚡ Hardware Setup
- **Board:** STM32L4A6ZG Nucleo  
- **Analog Input:** PA0 → voltage source or sensor  
- **Relay Output:** PB1  
- **LED Output:** PC0  
- **Button Input:** PC13 (user button)  
- **UART:** LPUART1 on PG7 (TX) / PG8 (RX), connect via ST-LINK VCP or external USB-UART  

---

## 🚀 How to Run
1. Import the source files into **STM32CubeIDE**.  
2. Build and flash to the **NUCLEO-L4A6ZG** board.  
3. Open a terminal (PuTTY, Tera Term) at the correct baud rate.  
4. Press the button to toggle the relay/LED.  
5. Observe **ADC statistics** (MIN, MAX, AVG) and **coil current** printed to the terminal.  

---

## 📂 Repository Structure
```
.
├── main.c / main.h          # System init, GPIO config, statistics display
├── ADC.c / ADC.h            # ADC driver, ISR, conversion utilities
├── lpuart.c / lpuart.h      # UART driver for formatted terminal output
├── delay.c / delay.h        # SysTick microsecond delay
├── .gitignore               # Ignore build artifacts
└── README.md                # Project documentation
```

---

## ✅ Learning Outcomes
- Configuring **ADC1** on STM32 for single-channel sampling  
- Using **interrupt-driven ADC** for batch data collection  
- Implementing **calibrated voltage/current conversions**  
- Driving a **UART terminal interface** with formatted measurement data  
- Combining **GPIO control, ADC sampling, and UART communication** in a modular project  

---

## 📜 License
This project is licensed under the MIT License – see [LICENSE](LICENSE).

---

👤 **Author:** Nathan Heil  
📅 **Course:** EE‑329 (Embedded Systems)  

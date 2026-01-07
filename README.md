#  Kuruma - Autonomous Line Tracking Vehicle

###  University Semester Project

**Kuruma** is an autonomous model vehicle developed as the final project for the module **"Mikrocontroller und Sensorik"** (Winter Semester 2025/26) at **Technische Hochschule Deggendorf**.

While this project utilizes the **NXP Cup** hardware platform and follows its track specifications, it is an **academic implementation** focused on developing a robust **bare-metal software architecture** from scratch. The primary goal was to demonstrate proficiency in embedded systems, real-time control, and sensor fusion without the use of an operating system.

---

##  Project Scope & Goals

Unlike competitive racing bots focused solely on speed, the main objective of this academic work was **System Robustness** and **Architectural Stability**.

* **Course:** Mikrocontroller und Sensorik (Bachelor Angewandte Informatik) 
* **Semester:** Winter Semester 25/26 
* **Platform:** NXP Freedom KL25Z / MK66F18 
* **Focus:** Bare-metal programming, Interrupt handling, and Control Theory.

---

## Key Features

* **Bare-Metal Architecture:** No OS (Super-loop + Interrupts) ensures deterministic timing for motor commutation and sensor reading.
* **Sensorless BLDC Control:** Custom software-based block commutation using Back-EMF detection via ADC (Zero-crossing).
* **Virtual Gearbox:** Dynamic speed adjustment based on steering angle (High speed on straights, high torque/low speed in curves).
* **Advanced Steering Logic:**
    * **Look-Ahead Delay Compensation:** A ring-buffer system to sync the camera's "future" sight with the physical position of the wheels.
    * **Kick-Start Mechanism:** Overcomes servo mechanical friction (stiction) by sending impulse commands before settling on the target angle.
    * **Virtual Walls:** Algorithmic extrapolation of missing lane markers during tight curves.
* **Live Tuning (HMI):** Real-time adjustment of parameters (Steering Delay & Black Threshold) using potentiometers and LCD feedback without recompiling.

---

## Hardware Architecture

* **Microcontroller:** NXP Kinetis MK66F18 (ARM Cortex-M4F) 
* **Vision:** 128-pixel Linear Camera (Analog Output) 
    * *Interface:* ADC1 (Data), FTM1 (Clock/SI Timing) 
* **Propulsion:** Brushless DC Motor (Sensorless)
    * *Interface:* FTM0 (PWM), ADC0 (Back-EMF) 
* **Steering:** Standard Analog Servo
    * *Interface:* FTM3 (PWM) 
* **Display:** 20x4 Character LCD (I2C via `I2C0`) 
* **Visuals:** WS2812B LED Strip (DMA + SPI driven) 

---

## Software Architecture

The software follows a strict separation between **Hard Real-Time** (Motor/Sensors) and **Soft Real-Time** (Logic/UI).

### 1. The Interrupt Layer (High Priority)
* **SysTick (10µs / 100kHz):** Handles the BLDC motor commutation state machine. This ensures the motor never loses sync ("stuttering") regardless of the main loop load.
* **FTM1 & ADC1 IRQ:** Handles camera timing and captures pixel data automatically.
* **GPIO IRQ:** Handles user inputs (Start/Stop, Locking settings).

### 2. The Main Loop (Logic Layer)
* **Lane Detection (`find_line`):**
    * Downsamples 128 pixels into 16 "blocks" for noise reduction.
    * Applies a dynamic threshold (tunable via Potentiometer).
    * Calculates `road_center` using "Virtual Wall" logic if one line is lost.
* **Steering Control (`logic_task` & `lenkung`):**
    * Determines steering direction (Left/Right/Center).
    * Pushes command into a **Ring Buffer** to delay execution (compensating for camera look-ahead distance).
    * Applies "Kick-Start" PWM pulse if the servo is stuck in a dead-band.
* **HMI (`refresh_lcd` & `led_siren_task`):**
    * Updates LCD only when values change ("dirty flag" pattern) to save CPU cycles.
    * Controls LED siren effects via DMA to offload CPU.

# How to Run

## 1. Steering & Servo Control (Lenkung)

These parameters control the physical steering servo and the software compensation for mechanical delay.

| Parameter | Default | Description | Tuning Tip |
| :--- | :--- | :--- | :--- |
| `LEFT` | `120.0f` | PWM Duty cycle for max **Left** turn. | Decrease if wheels hit the chassis. |
| `RIGHT` | `30.0f` | PWM Duty cycle for max **Right** turn. | Increase if wheels hit the chassis. |
| `CENTER` | `70.0f` | PWM Duty cycle for **Straight** driving. | Adjust if the car drifts L/R when it should go straight. |
| `CENTER_L` / `CENTER_R` | `50`/`100` | Kick-start pulse values. | Used to break static friction. Must be stronger than normal steering. |

### Advanced Steering Logic
**`STEERING_DELAY_STEPS`** (Default: `30`)
* **What it does:** The camera sees the track *ahead* of the car. This buffer delays the steering command so the car turns exactly when the wheels reach the curve.
* **How to Tune:**
    * **Car cuts corner (turns too early):** *Increase* this value (e.g., 35, 40).
    * **Car overshoots (turns too late):** *Decrease* this value (e.g., 20, 25).
    * **Formula:** `Delay Distance = Speed * Loop_Time * Buffer_Size`

---

## 2. Motor & Speed (Antrieb)

The BLDC motor is controlled via a virtual gearbox system. Note that for the Commutation Period, **Lower Value = Higher Speed**.

### Speed Settings
| Gear | Parameter | Value (Ticks) | Description |
| :--- | :--- | :--- | :--- |
| **1** | `SPEED_FOR_GEAR_1` | `5000` | **Slowest.** Used for sharp corners to maintain traction. |
| **2** | `SPEED_FOR_GEAR_2` | `4000` | **Medium.** Used for mild curves or transitions. |
| **3** | `SPEED_FOR_GEAR_3` | `3000` | **Fast.** Acceleration phase. |
| **4** | `SPEED_FOR_GEAR_4` | `2000` | **Max Speed.** Used for long straights. |

### Power Settings
* **`START_DUTY_CYCLE_BLDC`** (Default: `28`): Initial power kick to start the rotor. If the motor hums but doesn't spin at start, increase this slightly.
* **`RUN_DUTY_CYCLE_BLDC`** (Default: `22`): Constant power during running. Increase this if the car slows down too much on inclines or friction.

### Acceleration (Ramp-Up)
* **`BLDC_RUNNING_ACCEL_STEP`** (Default: `2`): Determines how "smoothly" the car changes gears.
    * *Lower (1):* Very smooth, slow acceleration.
    * *Higher (5+):* Aggressive acceleration, may cause wheel slip.

---

## 3. Vision & Camera (Kamera)

Parameters for the Linear Camera (128 pixels) and line detection algorithm.

* **`BLACK_THRESHOLD`** (Default: `600`):
    * ADC value (0-4096). Pixels below this are considered **Black**.
    * *Bright Environment:* Increase value (e.g., 800).
    * *Dark Environment:* Decrease value (e.g., 400).
    * *Note:* This is overridden by the Potentiometer if `ADC_LOCK` is off.

* **`IDEAL_CENTER_DIFF`** (Default: `5` blocks):
    * Used for the **"Virtual Wall"** algorithm.
    * If the camera sees only the *Left* line, it calculates the *Right* line position as `Left_Index - IDEAL_CENTER_DIFF`.
    * **Tune this:** Place car on track. If the calculated center is wrong when one line is covered, adjust this value.

---

## 4. Hardware Mappings

Physical connections on the NXP board. Do not change unless wiring changes.

| Macro | Pin / Value | Function |
| :--- | :--- | :--- |
| `LCD_ADDR` | `0x27` | I2C Address for the 20x4 Display. |
| `SIREN_SPEED` | `150` ms | Blinking speed for the LED strip. |
| `LOOP_DELAY_MS` | `5` ms | Main loop refresh rate. Lower = more responsive but higher CPU load. |

---

## ⚠️ Troubleshooting Guide

**Problem: "Motor makes a high pitched noise but doesn't move."**
> **Fix:** Increase `START_DUTY_CYCLE_BLDC` or check `BLDC_START_DELAY_TICKS`.

**Problem: "Car jitters/shakes on straight lines."**
> **Fix:** The `KICK_DURATION_MS` might be too long, or `STEERING_DELAY_STEPS` is too small causing immediate over-reaction.

**Problem: "LCD is flickering."**
> **Fix:** Increase `LCD_REFRESH_RATE` (e.g., to 600ms). The I2C bus might be too busy.
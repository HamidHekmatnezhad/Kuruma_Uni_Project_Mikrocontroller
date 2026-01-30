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
    * **Low-Latency Reaction:** Minimized look-ahead delay to suit the front-mounted camera configuration for immediate obstacle avoidance.
    * **Density-Based Line Repulsion:** Replaces the old center-calculation logic. The system splits the vision data into Left/Right sectors and counts active black blocks. If the block count exceeds the noise threshold (`MIN_BLOCKS_FOR_DETECTION`), the vehicle steers away from the detected boundary to stay in the "White Zone".
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
    * **High-Resolution Processing:** Downsamples 128 raw pixels into **32 blocks** (4 pixels per block) for precise edge detection.
    * **Hysteresis Filter:** Implements a state-retention logic for "Gray" pixels (edges) to prevent flickering and ensure stable line recognition.
    * **Dynamic Thresholding:** Classifies blocks based on a real-time threshold tunable via Potentiometer.
* **Steering Control (`logic_task` & `lenkung`):**
    * **Density-Based Decision:** Instead of calculating a geometric center, the logic compares the density of black blocks in the Left vs. Right sectors (`blk_counter`).
    * **Repulsion Logic:** If the block count on one side exceeds the noise margin (`MIN_BLOCKS_FOR_DETECTION`), the system triggers a steering command to "push" the car away from that boundary.
    * **Low-Latency Buffer:** Uses a minimized Ring Buffer (`STEERING_DELAY_STEPS = 5`) to ensure immediate reaction to the track directly in front of the wheels, while maintaining signal stability.
* **HMI (`refresh_lcd` & `led_siren_task`):**
    * Updates LCD only when values change ("dirty flag" pattern) to save CPU cycles.
    * Controls LED siren effects via DMA to offload CPU.

# How to Run

## 1. Steering & Servo Control (Lenkung)

These parameters control the physical steering servo and the software compensation for mechanical delay.

| Parameter | Default | Description | Tuning Tip |
| :--- | :--- | :--- | :--- |
| `LEFT` | `100.0f` | PWM Duty cycle for max **Left** turn. | Decrease if wheels hit the chassis. |
| `RIGHT` | `50.0f` | PWM Duty cycle for max **Right** turn. | Increase if wheels hit the chassis. |
| `CENTER` | `75.0f` | PWM Duty cycle for **Straight** driving. | Adjust if the car drifts L/R when it should go straight. |


### Advanced Steering Logic
**`STEERING_DELAY_STEPS`** (Default: `5`)
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
| **1** | `SPEED_FOR_GEAR_1` | `4000` | **Slowest.** Used for sharp corners to maintain traction. |
| **2** | `SPEED_FOR_GEAR_2` | `3500` | **Medium.** Used for mild curves or transitions. |
| **3** | `SPEED_FOR_GEAR_3` | `3000` | **Fast.** Acceleration phase. |
| **4** | `SPEED_FOR_GEAR_4` | `2000` | **Max Speed.** Used for long straights. |

### Power Settings
* **`START_DUTY_CYCLE_BLDC`** (Default: `30`): Initial power kick to start the rotor. If the motor hums but doesn't spin at start, increase this slightly.
* **`RUN_DUTY_CYCLE_BLDC`** (Default: `25`): Constant power during running. Increase this if the car slows down too much on inclines or friction.

### Acceleration (Ramp-Up)
* **`BLDC_RUNNING_ACCEL_STEP`** (Default: `2`): Determines how "smoothly" the car changes gears.
    * *Lower (1):* Very smooth, slow acceleration.
    * *Higher (5+):* Aggressive acceleration, may cause wheel slip.

---

## 3. Vision & Camera (Kamera)

Parameters for the Linear Camera (128 pixels) and line detection algorithm.

* **`BLACK_THRESHOLD`** (Default: `1000`):
    * ADC value (0-4096). Pixels below this are considered **Black**.
    * *Bright Environment:* Increase value (e.g., 800).
    * *Dark Environment:* Decrease value (e.g., 400).
    * *Note:* This is overridden by the Potentiometer if `ADC_LOCK` is off.

* **`MIN_BLOCKS_FOR_DETECTION`** (Default: `5` blocks):
    * **Function:** Acts as a **Noise Filter** and **Confidence Threshold** for the repulsion logic.
    * **Logic:** The system counts black blocks on each side. If the count is below this value, the signal is ignored (treated as shadows or sensor noise). Steering only triggers when the line density exceeds this threshold.
    * **Tune this:** Increase this value if the car jitters due to dirt/shadows on the track. Decrease it if the car fails to react to thin lines.

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
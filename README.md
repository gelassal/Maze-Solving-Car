# Autonomous Track Explorer

An autonomous wall-following robot built on the TM4C123 (ARM Cortex-M4) microcontroller. The car uses three Sharp IR distance sensors and hardware PWM to navigate a walled track — centering itself between two walls during normal travel, arcing through curves, and pivoting in place to find a new path when cornered.

---

## Hardware

| Component | Description |
|---|---|
| Microcontroller | TI TM4C123GH6PM (ARM Cortex-M4, 80 MHz) |
| Distance Sensors | 3x Sharp GP2Y0A21YK0F analog IR sensors |
| Motors | 2x DC motors controlled via hardware PWM |
| Motor Driver | H-bridge interfaced through PWM0 |
| Indicators | Onboard RGB LED (Port F) |
| Controls | Onboard push buttons SW1 / SW2 (Port F) |

### Sensor Placement

```
  --------- wall ---------
        \    |    /
    [L]  \  [F]  /  [R]
          [Robot]
             |
           
  --------- wall ---------
```

- **PE1** — Forward-facing sensor (collision detection)
- **PE4** — Right diagonal sensor (right wall distance)
- **PE5** — Left diagonal sensor (left wall distance)

---

## How It Works

### Startup
- SW1 starts the car, SW2 stops it
- On startup, the median filter is primed with 9 sensor reads before motion begins to ensure stable initial readings

### Steering Logic

The `steering()` function runs every main loop iteration and evaluates the following cases in priority order:

| Priority | Condition | Behavior |
|---|---|---|
| 1 | Ahead sensor reads > 20cm (open space) | Pivot in place toward closer wall to reorient |
| 2 | All three sensors < 80cm (boxed in) | Stop both motors, LED blue |
| 3 | Ahead + right blocked, left open | Arc right (left wheel only) |
| 4 | Ahead + left blocked, right open | Arc left (right wheel only) |
| 5 | Normal driving | Proportional centering (see below) |

### Proportional Centering

During normal driving, the car computes the difference between left and right sensor readings and applies a three-tier correction:

```
|diff| < 500        → Go straight (equal power to both wheels)
|diff| < 1200       → Gentle correction (60% duty on the wider side)
|diff| >= 1200      → Aggressive correction (90% duty on the wider side)
```

A higher ADC value means a closer object, so if `left_dist > right_dist`, the left wall is closer and the car steers right by increasing power to the right wheel.

### Sensor Filtering

Raw ADC readings from the GP2Y0A21YK0F sensors contain occasional erroneous voltage spikes. A **median filter** (`y(n) = median(x(n), x(n-1), x(n-2))`) is applied to all three sensor channels every loop iteration to reject outliers before they affect steering.

### LED Status

| LED Color | Meaning |
|---|---|
| Red | At least one sensor reads beyond 10cm (open space nearby) |
| Green | All sensors within 10cm (tightly enclosed, well centered) |
| Blue | All sensors within 80cm (normal driving range) |
| Off | Car is stopped (SW2 pressed) |

---

## ADC Distance Calibration

The GP2Y0A21YK0F outputs an analog voltage inversely proportional to distance. ADC values were calculated from datasheet voltages and tuned experimentally:

| Distance | Voltage | ADC Value |
|---|---|---|
| 10 cm | ~2.6 V | 3200 |
| 15 cm | 1.8 V | 2400 |
| 20 cm | 1.39 V | 1780 |
| 30 cm | 0.9 V | 1190 |
| 40 cm | 0.74 V | 860 |
| 80 cm | 0.4 V | 700 |

---

## Project Structure

```
TrackExplorer/
├── TrackExplorer.c     # Main file: system init, main loop, steering logic
├── Motors.c / .h       # Hardware PWM setup and duty cycle control
├── Sensors.c / .h      # ADC init and FIR/IIR/median filter implementations
├── LEDSW.c / .h        # RGB LED and push button GPIO configuration
├── PLL.c / .h          # Phase-locked loop: sets bus clock to 80 MHz
└── tm4c123gh6pm.h      # TM4C123 register definitions
```

---

## Build & Flash

1. Open `TrackExplorer.uvprojx` in **Keil µVision 5**
2. Build the project (`F7`)
3. Connect the TM4C123 LaunchPad via USB
4. Flash using the onboard ICDI debugger (`F8`)
5. Press **SW1** on the board to start the car

---

## Course Context

Developed for **CECS 347 – Microprocessor-Based System Design** at California State University, Long Beach. Based on starter code originally authored by Dr. Daniel Valvano and Jonathan Valvano (UT Austin), with steering logic and control algorithm implemented independently.

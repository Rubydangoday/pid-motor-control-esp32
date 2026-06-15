# Dual-Loop PID Motor Control (Position & Velocity)

Cascaded PID controller for DC motor using ESP32 and IBT-2 driver.  
Outer loop: position control | Inner loop: velocity control.

## Hardware

| Component | Detail |
|-----------|--------|
| MCU | ESP32 |
| Motor Driver | IBT-2 (BTS7960) |
| Encoder |  
| Motor | 

## Wiring

| ESP32 Pin | IBT-2 Pin |
|-----------|-----------|
| [pin] | RPWM |
| [pin] | LPWM |
| [pin] | R_EN / L_EN |
| [pin] | Encoder A |
| [pin] | Encoder B |

## How It Works
## PID Parameters

| Loop | Kp | Ki | Kd |
|------|----|----|----|
| Position | - | - | - |
| Velocity | - | - | - |

## Usage

1. Install [Arduino IDE](https://www.arduino.cc/en/software)
2. Install ESP32 board: File → Preferences → add URL:  
   `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
3. Clone repo:
```bash
   git clone https://github.com/Rubydangoday/pid-motor-control-esp32.git
```
4. Open `src/main.cpp` in Arduino IDE
5. Select board: **ESP32 Dev Module**
6. Upload

## Results

- Stable position tracking with overshoot 
- Steady-state error 

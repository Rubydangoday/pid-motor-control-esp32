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

# Sending PWM Signals from Raspberry Pi to ESC / Steering Servo

## 1. Introduction

This document explains how the `rc_driver` ROS 2 package drives an RC car's ESC (motor controller) and steering servo from a Raspberry Pi.

## 2. System Overview and Block Diagram (signal flow from cmd_vel input to ESC/servo output)
- Input: cmd_vel from outside the Raspberry Pi (e.g. teleop)
- Blocks: Raspberry Pi (rc_driver) → ESC / Steering Servo

## 3. Overview of the rc_driver Package (node graph)

## 4. Hardware Specifications
### 4.1 ESC Input Signal Specification
### 4.2 Steering Servo Motor Input Signal Specification
### 4.3 gpiozero Servo Class Specification

## 5. Mapping Table: Implementation Parameters vs. Hardware Specifications (why this configuration works)

## 6. GPIO Pin Wiring Diagram / Wiring Table

## 7. Summary and References

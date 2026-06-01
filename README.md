# Embedded Playground
> A practical embedded systems sandbox for experimenting with **AVR** and **STM32** microcontrollers using real-world peripherals, low-level programming, hardware abstraction layers, and debugging workflows.

![License](https://img.shields.io/badge/license-MIT-green)
![Platform](https://img.shields.io/badge/platform-AVR%20%7C%20STM32-blue)
![Language](https://img.shields.io/badge/language-C%20%7C%20C%2B%2B-orange)
![Status](https://img.shields.io/badge/status-active-success)
![Embedded](https://img.shields.io/badge/focus-embedded%20systems-red)

---

## Overview

**Embedded Playground** is a hands-on repository dedicated to learning, prototyping, and experimenting with embedded systems development across **AVR** and **STM32** microcontroller platforms.

The goal of this repository is to provide a structured environment for exploring:

- Bare-metal programming
- Peripheral interfacing
- Hardware abstraction
- Register-level control
- Interrupt-driven systems
- Real-time embedded concepts
- Communication protocols
- Low-power techniques
- Debugging and profiling

Rather than being a single firmware project, this repository acts as a **playground of independent embedded experiments**, reusable drivers, and platform-specific examples.

Whether you're learning embedded systems fundamentals or testing new ideas before integrating them into larger systems, this repo provides a clean, modular environment for experimentation.

---

## Supported Platforms

### AVR Family

Supported AVR microcontrollers include:

- **ATmega328P** (Arduino Uno)
- **ATmega2560**
- **ATtiny85**
- **ATmega32**
- **ATmega16**

Typical focus areas:

- Direct register programming
- Timer configuration
- PWM generation
- UART communication
- ADC interfacing
- SPI/I2C peripherals
- Low-power sleep modes

### STM32 Family

Supported STM32 targets include:

- **STM32F103C8T6** (Blue Pill)
- **STM32F401RE**
- **STM32F407VG**
- **STM32L4 Series**
- **STM32 Nucleo Boards**

Development approaches:

- STM32 HAL
- CMSIS
- Register-level programming
- FreeRTOS experimentation
- DMA usage
- Interrupt systems
- Peripheral abstraction

---

## Repository Goals

This project aims to:

- Provide reusable embedded code examples
- Demonstrate hardware-software interaction
- Compare architectures between AVR and STM32
- Explore embedded optimization techniques
- Build reusable peripheral drivers
- Serve as a personal experimentation lab

---

## Features

### GPIO Experiments

- LED blinking
- Button debouncing
- Interrupt-driven GPIO
- GPIO abstraction layer

### Timers & PWM

- Timer interrupts
- Frequency generation
- Servo motor control
- PWM dimming
- Pulse measurement

### Communication Protocols

#### UART

- Serial debugging
- Command parsing
- Communication interfaces

#### SPI

- Sensor interfacing
- Display communication
- EEPROM examples

#### I2C

- RTC modules
- OLED displays
- Sensor communication

### ADC & DAC

- Analog sensor reading
- Voltage measurement
- Signal acquisition
- PWM-based DAC simulation

### RTOS Experiments

- FreeRTOS task scheduling
- Task synchronization
- Mutexes & semaphores
- Queue systems

### Sensors & Modules

- DHT11 / DHT22
- MPU6050
- HC-SR04
- SSD1306 OLED
- Temperature sensors
- Rotary encoders

---

## Project Structure

```text
embedded-playground/
│
├── avr/
│   ├── gpio/
│   ├── timers/
│   ├── uart/
│   ├── spi/
│   ├── i2c/
│   ├── adc/
│   ├── interrupts/
│   └── examples/
│
├── stm32/
│   ├── hal/
│   ├── baremetal/
│   ├── freertos/
│   ├── drivers/
│   ├── communication/
│   ├── sensors/
│   └── examples/
│
├── shared/
│   ├── utilities/
│   ├── common_drivers/
│   └── abstractions/
│
├── docs/
│   ├── setup/
│   ├── hardware/
│   └── architecture/
│
├── scripts/
│   ├── flash/
│   └── build/
│
├── tools/
│
├── .github/
│   └── workflows/
│
└── README.md

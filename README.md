# RTScheduler-Discovery

A realtime preemptive scheduler for the STM Discovery boards.

## Table of Contents

- [Overview](#overview)
- [Repository Structure](#repository-structure)
- [Prerequisites](#prerequisites)
- [Getting Started](#getting-started)
- [Project Structure](#project-structure)
- [Building](#building)

## Overview

RTScheduler-Discovery is a real-time preemptive scheduler implementation for ARM-based STM Discovery development boards. This project provides the core OS and a foundation for building embedded real-time applications.

## Repository Structure

The repository is organized as follows:

- **`arch/`** - Architecture-specific code and implementations
- **`boards/`** - Board definitions and hardware configurations
- **`drivers/`** - Device drivers for hardware peripherals
- **`incl/`** - Header files and public interfaces
- **`os/`** - Core operating system implementation
- **`stddrv/`** - Standard drivers
- **`stdlib/`** - Standard library utilities
- **`usr/`** - User applications and examples

## Prerequisites

To build RTScheduler-Discovery, you will need:

- **GCC ARM Compiler** - GNU Arm Embedded Toolchain or equivalent
- **Make** - Build system

## Getting Started

1. **Create a project directory** for your application
2. **Copy an existing application** as a template (e.g., OBDI for Chevy TPI analyzer, or cec_gw)
3. **Update your Makefile** to point to the base release:
   ```makefile
   KREL=../RTScheduler-Discovery/boards/MB997C
   ```
4. **Copy and configure** the board-specific `config.h`:
   - Copy `config.h` from your board type
   - Update driver configuration
   - Update I/O pin assignments

## Project Structure

Each board configuration in the `boards/` directory contains:
- Hardware definitions
- Pin mappings
- Device configurations
- Board-specific settings

Update the `config.h` with respect to your drivers, I/O pins, and hardware setup.

## Building

With the GCC ARM compiler installed and prerequisites met, build your project using:

```bash
make
```

For more information on configuration and building, refer to the board-specific documentation in the `boards/` directory.
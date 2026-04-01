# Pololu 3pi+ 2040 Robot - Monorepo

Welcome to the unified repository for the Pololu 3pi+ 2040 Robot project!

This repository contains both the Rust firmware for the robot and the ROS workspace for the workstation controller.

## Repository Structure

- **`firmware/`**: Contains the Rust-based firmware for the Raspberry Pi RP2040 microcontroller. Make sure to `cd firmware` before compiling with Cargo.
- **`ROS/`**: Contains the `workstation_ros` code, including the ROS 2 workspace, MQTT tracking bridge, and simulation tools. Make sure to run `colcon build` inside this directory.

## Getting Started

Please refer to the detailed READMEs inside each directory for build instructions and setup:
- [Firmware README](./firmware/README.md)
- [ROS README](./ROS/README.md)

## Submodules

This repository uses git submodules for external dependencies.
After cloning, make sure to pull the recursive submodules:
```bash
git submodule update --init --recursive
```

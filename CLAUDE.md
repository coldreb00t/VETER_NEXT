# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a robotics project for NVIDIA Jetson platforms. The project is in early stages with the following directory structure:

- `src/` - Source code (application logic, robot control, sensors, vision, etc.)
- `hardware/` - Hardware-related configurations, schematics, or interface definitions
- `scripts/` - Build scripts, deployment scripts, and utility scripts
- `docs/` - Documentation

## Architecture Notes

### Platform Considerations

This project runs on NVIDIA Jetson hardware (Linux 5.15.148-tegra). When working with code:

- Be aware of ARM64 architecture - some packages may require platform-specific builds
- Jetson devices have GPU acceleration (CUDA) available for computer vision and ML tasks
- Common frameworks: JetPack SDK, OpenCV with CUDA support, TensorRT, PyTorch

### Typical Development Stack

Since this is a Jetson robotics project, expect potential use of:

- **Python**: For rapid prototyping, ML inference, and high-level control
- **C++**: For performance-critical robotics code, camera interfaces, real-time control
- **ROS/ROS2**: May be used for robot middleware (check for package.xml or CMakeLists.txt)

## Development Workflow

As the project evolves, common commands will likely include:

- Building: Look for Makefile, CMakeLists.txt, setup.py, or package.json in future commits
- Testing: Check scripts/ directory for test runners
- Deployment: Scripts for deploying to Jetson hardware will likely appear in scripts/

## Future Development Guidelines

When implementing features in this robotics codebase:

- Place hardware interface code (GPIO, I2C, cameras, sensors) in appropriate modules within src/
- Keep hardware-specific configurations in hardware/
- Document camera calibration parameters, sensor specifications, and pin mappings in docs/
- Use scripts/ for automation (build, deploy, hardware setup)

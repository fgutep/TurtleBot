# TurtleBot Burger - Open Source SLAM Platform

<p align="center">
  <img src="img/hero.jpeg" alt="TurtleBot Burger" width="800"/>
</p>

> **This is an active development project** - We're continuously improving and refining the design. Contributions and feedback are welcome!

An open-source, low-cost differential drive robot platform designed for robotics enthusiasts who want to learn SLAM (Simultaneous Localization and Mapping) without breaking the bank. Built from scratch with 3D-printable parts and custom electronics, this TurtleBot Burger-style robot is perfect for hands-on learning and experimentation.

## Project Goals

- **Accessible**: Fully open-source with affordable, readily available components
- **Educational**: Learn SLAM, ROS2, embedded systems, and robotics fundamentals
- **Customizable**: Modify the chassis, firmware, or electronics to suit your needs
- **Community-Driven**: Built by enthusiasts, for enthusiasts

## Bill of Materials

### Electronics
- **Main Computer**: Raspberry Pi 5 (8GB) - Runs ROS2 and SLAM algorithms
- **Motion Controller**: Custom ESP32-S3 PCB with DRV8833 motor drivers
- **LiDAR**: RPLIDAR C1 - For mapping and localization
- **Motors**: 2x Namiki 2CL-3501PG with integrated encoders
- **Power**: Custom 2S 18650 battery pack (5000mAh, ~7.4V nominal)
- **Voltage Regulation**: 5V, 3.3V, and 11V regulators

### Mechanical
- **Chassis**: 3D-printed PLA body (files included)
- **Wheels**: Custom-designed with TPU tread for traction
- **All structural components**: Fully 3D-printable

<p align="center">
  <img src="img/pcb.jpg" alt="Custom ESP32-S3 PCB" width="600"/>
</p>

## Key Features

### Hardware
- **Custom PCB (V1)**: ESP32-S3-based controller with:
  - Independent DRV8833 driver per motor
  - Encoder reading capabilities for precise odometry
  - PID control support
  - Battery voltage and current monitoring
  - I2C expansion header for additional sensors
  - Functions as a ROS2 motion/motors node via UART/Serial

- **Differential Drive Platform**: Simple, reliable two-wheel design
- **High-Quality Encoders**: Namiki motors provide excellent feedback for odometry and control
- **Modular Design**: Easy to modify and expand

### Software
- **ROS2 Integration**: Full ROS2 support running on Raspberry Pi 5
- **UART/Serial Communication**: Motion controller interfaces with ROS2 stack
- **SLAM Ready**: Compatible with standard SLAM packages using RPLIDAR C1
- **Custom Firmware**: ESP32-S3 code for kinematics and motor control (in development)

<p align="center">
  <img src="img/3d_model.jpg" alt="3D Model" width="600"/>
</p>

## Repository Structure

```
.
├── 3D_files/              # 3D printable chassis and wheel designs
│   ├── solidworks/        # Native SolidWorks files
│   └── step/              # Universal STEP format files
├── pcb/                   # PCB design files and schematics
├── firmware/              # ESP32-S3 firmware for motion control
├── ros2_packages/         # ROS2 integration packages
├── docs/                  # Documentation and assembly guides
└── img/                   # Project images
```

## Known Issues

### PCB V1 - DRV8833 Footprint Mismatch
**Status**: Active fix in progress (V2 PCB)

The current V1 PCB has a footprint mismatch between the DRV8833 module and the board design. This causes an unintended short circuit instead of the intended parallel input configuration for the motor drivers.

**Workaround**: We are currently developing V2 of the PCB to resolve this issue. If you're building from V1 files, please wait for the V2 release or contact us for manual modification instructions.

## Getting Started

### Prerequisites
- 3D printer (PLA for chassis, TPU for wheels)
- Basic soldering skills
- ROS2 installation (Humble or later recommended)
- Arduino/PlatformIO for ESP32-S3 firmware

### Build Steps
1. **Print the Chassis**: Use files from `/3D_files/` to print all mechanical components
2. **Assemble Electronics**: Solder and assemble the custom PCB (wait for V2 for best results)
3. **Flash Firmware**: Upload ESP32-S3 code to the motion controller
4. **Install ROS2**: Set up ROS2 on the Raspberry Pi 5
5. **Configure and Calibrate**: Follow documentation in `/docs/` for setup

*Detailed assembly instructions coming soon!*

## Contributing

We welcome contributions! This is an active project and we're looking for:
- Hardware improvements and optimizations
- Firmware enhancements
- ROS2 package development
- Documentation and tutorials
- Testing and bug reports

Please open an issue or pull request if you'd like to contribute.

## License

This project is open-source and available under the [MIT License](LICENSE).

## Resources

- [ROS2 Documentation](https://docs.ros.org/)
- [RPLIDAR C1 Documentation](https://www.slamtec.com/en/Lidar/C1)
- [ESP32-S3 Datasheet](https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_en.pdf)

## Contact

Have questions or want to collaborate? Open an issue or reach out to the maintainers!

---

**Disclaimer**: This is an educational project under active development. Use at your own risk and always follow proper safety procedures when working with electronics and batteries.

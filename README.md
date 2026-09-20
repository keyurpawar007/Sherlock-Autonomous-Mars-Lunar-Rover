# ROS 2 Autonomous Rover Workspace

This repository contains the complete ROS 2 workspace for the **Autonomous Rover System** designed for the **Indian / International Rover Challenge (IRC)**.

## Architecture Blueprint

The complete architecture specification, state machine diagrams, ROS graph topology, sensor fusion equations, TF tree layout, safety watchdogs, and 10-phase implementation roadmap are documented in detail in:

🔗 **[FINAL_AUTONOMOUS_ROVER_ARCHITECTURE.md](file:///d:/12%20sep%20kinematics%20test/rover_ws/FINAL_AUTONOMOUS_ROVER_ARCHITECTURE.md)**

---

## Workspace Structure

- **`rover_bringup/`**: System bringup launch files & master YAML configs.
- **`rover_description/`**: URDF/Xacro models, mesh definitions & TF calibration configs.
- **`rover_interfaces/`**: Custom ROS 2 messages (`WheelRPM`, `MissionStatus`, `TerrainState`) and services.
- **`rover_sensors/`**: Drivers and publishers for GPS, IMU, Encoders, and D435i RealSense depth camera.
- **`rover_localization/`**: EKF state estimation (`robot_localization`) fusing GPS, IMU, and Wheel Odometry.
- **`rover_perception/`**: Modular 3D perception engine (Obstacles, Ground plane filtering, Cliffs, Red zones, Object pose).
- **`rover_mapping/`**: Map generation and costmap layers (`nav2_costmap_2d`).
- **`rover_navigation/`**: Global planning, local dynamic planning, and path following.
- **`rover_control/`**: Differential drive kinematics, velocity control, and closed-loop PID RPM feedback.
- **`rover_stm_bridge/`**: Real-time UART/CAN communication interface with STM32 motor controller.
- **`rover_mission/`**: Top-level mission state machine (RADO autonomous delivery, ABEx science, IDMO assistance).
- **`rover_safety/`**: Autonomous mode lock, cliff/collision safety monitor, and hardware watchdog.
- **`rover_tools/`**: System diagnostics, sensor calibration utilities, and ROS bag loggers.

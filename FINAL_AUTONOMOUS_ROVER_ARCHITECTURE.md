# FINAL AUTONOMOUS ROVER ARCHITECTURE & IMPLEMENTATION SPECIFICATION
**Target Competition:** Indian / International Rover Challenge (IRC)  
**Hardware Platform:** NVIDIA Jetson (Brain) + STM32 (Real-Time Motor Controller) + Intel RealSense D435i + GPS + IMU + Encoders  
**Software Stack:** ROS 2 (Humble/Iron) + Nav2 + EKF (`robot_localization`) + OpenCV / Point Cloud Library  

---

## 1. System Architecture Overview & High-Level Block Diagram

The system operates as a hierarchical, modular ROS 2 architecture where high-level decision-making and perception run on the NVIDIA Jetson, while real-time low-level motor actuation and encoder feedback are handled by the STM32 controller.

```
                         ┌─────────────────────┐
                         │       MISSION       │
                         │      MANAGER        │
                         └──────────┬──────────┘
                                    │
                    Goal / GPS / Task / Object
                                    │
                                    ▼
                         ┌─────────────────────┐
                         │     PERCEPTION      │
                         │                     │
                         │ D435i RGB           │
                         │ D435i Depth         │
                         │ Object Detection    │
                         │ Obstacle Detection  │
                         │ Terrain Detection   │
                         │ Red Zone Detection  │
                         │ Drop/Cliff Detection│
                         └──────────┬──────────┘
                                    │
                                    ▼
                         ┌─────────────────────┐
                         │   SENSOR FUSION     │
                         │                     │
                         │ GPS + IMU + Encoder │
                         └──────────┬──────────┘
                                    │
                                    ▼
                         ┌─────────────────────┐
                         │    LOCALIZATION     │
                         │                     │
                         │ map → robot pose    │
                         │ x, y, yaw           │
                         │ covariance           │
                         └──────────┬──────────┘
                                    │
                                    ▼
                         ┌─────────────────────┐
                         │    WORLD MODEL      │
                         │                     │
                         │ Obstacles           │
                         │ Terrain             │
                         │ No-Go zones         │
                         │ Goal                │
                         └──────────┬──────────┘
                                    │
                    ┌───────────────┴──────────────┐
                    ▼                              ▼
          ┌──────────────────┐          ┌──────────────────┐
          │ GLOBAL PLANNER   │          │ LOCAL PLANNER    │
          │                  │          │                  │
          │ GPS → route      │          │ Depth → avoid    │
          └────────┬─────────┘          └────────┬─────────┘
                   └────────────┬────────────────┘
                                ▼
                     ┌─────────────────────┐
                     │  MOTION CONTROLLER  │
                     │                     │
                     │ linear velocity     │
                     │ angular velocity    │
                     └──────────┬──────────┘
                                ▼
                     ┌─────────────────────┐
                     │     KINEMATICS      │
                     │                     │
                     │ v, ω → wheel RPM    │
                     └──────────┬──────────┘
                                ▼
                     ┌─────────────────────┐
                     │        STM          │
                     │ Motor Controller    │
                     └──────────┬──────────┘
                                ▼
                         LEFT/RIGHT MOTORS
                                │
                                ▼
                           ENCODERS
                                │
                                └──────► FEEDBACK
```

### Parallel Safety Watchdog Engine
Running asynchronously alongside the main control flow is a dedicated Safety Manager node:

```
             SAFETY MANAGER
                  │
     ┌────────────┼─────────────┐
     ↓            ↓             ↓
  Cliff       Collision      Sensor fault
     ↓            ↓             ↓
     └────────── STOP ──────────┘
```

---

## 2. 9-Stage Autonomy Development Roadmap

| Phase | Milestone | Core Objective | Key Deliverables |
|---|---|---|---|
| **Phase 0** | Hardware & ROS 2 Foundation | System bringup & communication | URDF, STM bridge (UART/CAN), sensor drivers |
| **Phase 1** | Wheel Odometry | Kinematics & relative dead-reckoning | `/odom` publisher, differential drive kinematics |
| **Phase 2** | Sensor Fusion & Localization | Global & local state estimation | EKF (`robot_localization`), TF tree (`map->odom->base_link`) |
| **Phase 3** | D435i Perception Engine | 3D visual perception & obstacle detection | 5 independent detectors (Obstacle, Cliff, Red Zone, etc.) |
| **Phase 4** | World Model & Costmaps | Spatial environment modeling | Nav2 costmaps (Global & Local), obstacle inflation |
| **Phase 5** | Autonomous Navigation | Path planning & control | Nav2 / Custom Global Planner + Local Reactive Planner |
| **Phase 6** | Object Detection & 3D Pose | Target perception for RADO | YOLO/OpenCV + Depth projection to 3D `base_link` coordinates |
| **Phase 7** | Autonomous Delivery (RADO) | End-to-end task execution | Mission state machine, autonomous mode lock |
| **Phase 8** | Safety, Watchdogs & Recovery | System resilience & fail-safes | Cliff detector, comm loss watchdog, auto-recovery routines |
| **Phase 9** | Mission Simulation & Field Testing | Competition readiness | 20-min & 30-min field simulations under IRC rules |

---

## 3. Phase 0 — Hardware Architecture & Responsibility Split

### Jetson (System Brain)
- **Role:** High-level processing, AI, state estimation, mapping, planning, state machine.
- **Components:** ROS 2 core, Sensor Drivers (GPS, IMU, D435i), `robot_localization`, Costmaps, Mission Manager, Safety Manager.

### STM32 (Real-Time Motor Controller)
- **Role:** Low-level motor hardware control & sensor acquisition.
- **Responsibilities:**
  - Reads hardware quadrature encoders at high frequency (>100 Hz).
  - Executes closed-loop PID velocity control per wheel.
  - Receives desired target RPM commands (`/drive/rpm_command`) from Jetson.
  - Returns measured RPM feedback (`/drive/rpm_feedback`) to Jetson.
  - Implements hardware safety timeout (stops motors if Jetson heartbeats cease for >200 ms).
  - **CRITICAL RULE:** STM32 **never** makes high-level autonomy or navigation decisions.

```
Jetson (Brain) ──[ /drive/rpm_command ]──► STM32 ──► Motor Drivers ──► Motors
     ▲                                      │
     └───────────[ /drive/rpm_feedback ]────┴── Encoders
```

---

## 4. Standard ROS 2 Topic Architecture

```
/sensors/gps                     [sensor_msgs/msg/NavSatFix]
/sensors/imu                     [sensor_msgs/msg/Imu]
/sensors/imu_d435                [sensor_msgs/msg/Imu]
/sensors/encoders                [rover_interfaces/msg/WheelRPM]

/camera/color/image_raw          [sensor_msgs/msg/Image]
/camera/depth/image_raw          [sensor_msgs/msg/Image]
/camera/depth/points             [sensor_msgs/msg/PointCloud2]

/odom                            [nav_msgs/msg/Odometry] (Wheel encoder only)
/odometry/gps                    [nav_msgs/msg/Odometry] (UTM transformed GPS)
/odometry/fused                  [nav_msgs/msg/Odometry] (EKF output)

/tf                              [tf2_msgs/msg/TFMessage]
/tf_static                       [tf2_msgs/msg/TFMessage]

/perception/objects              [geometry_msgs/msg/PoseArray]
/perception/obstacles            [sensor_msgs/msg/PointCloud2]
/perception/terrain              [rover_interfaces/msg/TerrainState]
/perception/no_go                [geometry_msgs/msg/PolygonStamped]
/perception/cliffs               [geometry_msgs/msg/PoseStamped]

/map                             [nav_msgs/msg/OccupancyGrid]
/costmap                         [nav_msgs/msg/OccupancyGrid]

/navigation/goal                 [geometry_msgs/msg/PoseStamped]
/navigation/path                 [nav_msgs/msg/Path]
/cmd_vel                         [geometry_msgs/msg/Twist]

/drive/rpm_command               [rover_interfaces/msg/WheelRPM]
/drive/rpm_feedback              [rover_interfaces/msg/WheelRPM]

/mission/state                   [std_msgs/msg/String]
/mission/status                  [rover_interfaces/msg/MissionStatus]

/safety/status                   [std_msgs/msg/String]
/safety/stop                     [std_msgs/msg/Bool]
```

---

## 5. Phase 1 — Differential Drive Wheel Kinematics & Odometry Equations

### Differential Drive Kinematics
For a rover with track width $L$ and wheel radius $r$:

1. **Linear Velocity ($v$) & Angular Velocity ($\omega$):**
   $$v = \frac{v_{\text{right}} + v_{\text{left}}}{2}$$
   $$\omega = \frac{v_{\text{right}} - v_{\text{left}}}{L}$$

2. **State Propagation (Dead Reckoning):**
   $$\dot{x} = v \cos(\text{yaw})$$
   $$\dot{y} = v \sin(\text{yaw})$$
   $$\dot{\text{yaw}} = \omega$$

3. **Wheel RPM to Linear Velocity Conversion:**
   $$v_{\text{wheel}} = \frac{\text{RPM} \times 2\pi \times r}{60}$$

### Mandatory Field Test Suite (Phase 1 Validation)
- **Test A (1 Meter Straight):** Command rover straight for 1.0 m.  
  *Expected Output:* $x \approx 1.0\text{ m}, y \approx 0.0\text{ m}, \text{yaw} \approx 0.0^\circ$.
- **Test B (90° In-Place Left Turn):** Command $90^\circ$ turn.  
  *Expected Output:* $\text{yaw} \approx +90.0^\circ (+1.57\text{ rad})$.
- **Test C (10 Meter Calibration Run):** Drive 10.0 m straight on flat tarmac.  
  *Validation:* Compare physical tape-measure distance vs reported encoder distance to compute scale factor correction.

---

## 6. Wheel Slip Analysis & Mitigation

In soft sand and loose rock terrains common in IRC scenarios:
- Measured Wheel Odometry Distance: $10.0\text{ m}$
- Actual Ground Distance Travelled: $8.5\text{ m}$
- **Calculated Wheel Slip Ratio:**
  $$\text{Slip Ratio} = \frac{D_{\text{encoder}} - D_{\text{actual}}}{D_{\text{encoder}}} = \frac{10.0 - 8.5}{10.0} = 15\%$$

**Mitigation Strategy:** Blind trust in wheel encoders causes massive drift over time. Encoder odometry MUST be fused with high-rate IMU angular rates and global GPS coordinates via an Extended Kalman Filter (EKF).

---

## 7. Phase 2 — Sensor Fusion Architecture (EKF)

Using `robot_localization` with dual EKF instances:

```
             GPS (Lat/Lon) ──► navsat_transform_node ──► /odometry/gps ──┐
                                                                         │
IMU (Yaw, Angular Velocity, Acceleration) ───────────────────────────────┼──► EKF Global Node ──► map → odom
                                                                         │
Wheel Encoders (Linear Vel, Angular Vel) ────────────────────────────────┴──► EKF Local Node  ──► odom → base_link
```

---

## 8. Sensor Responsibility Matrix

| Sensor | Measurable Quantity | Primary Purpose | Failure Mode / Limitation |
|---|---|---|---|
| **GPS** | Latitude, Longitude, Altitude | Global reference coordinate anchoring | Drift near cliffs/structures, ~1-3m accuracy |
| **IMU** | Pitch, Roll, Yaw Rate, Accel | Orientation & high-rate heading tracking | Cumulative yaw drift over extended time |
| **Encoders** | Left/Right wheel displacement & velocity | Short-term smooth relative velocity estimation | Wheel slip in soft sand/gravel |
| **D435i Depth** | 3D Point Cloud, Depth Maps | Obstacle detection, terrain geometry, cliff scan | Direct blinding sunlight, reflective surfaces |

---

## 9. Coordinate Frames & TF Tree Topology

Standard ROS REP-105 compliant static & dynamic TF hierarchy:

```
map
 └── odom
      └── base_link
           ├── base_footprint (z = 0 ground level)
           ├── camera_link
           │    └── camera_depth_frame
           ├── imu_link
           └── gps_link
```

### Extrinsic Sensor Calibration Protocol
Every sensor frame MUST have measured static transforms ($x, y, z, \text{roll}, \text{pitch}, \text{yaw}$) relative to `base_link` defined in the robot's URDF (`rover_description/urdf/rover.urdf.xacro`).

---

## 10. Architectural Guardrail: GPS Isolation Rule

> [!CAUTION]
> **NEVER** pipe raw GPS coordinates directly to `/cmd_vel` or motor RPM nodes. Doing so bypasses obstacle avoidance, local costmaps, and orientation control, creating severe crash risks.

**Correct Pipeline Flow:**
$$\text{GPS Coordinate} \longrightarrow \text{Localization (EKF)} \longrightarrow \text{Robot Pose} \longrightarrow \text{Global Planner} \longrightarrow \text{Local Planner} \longrightarrow \text{Kinematics} \longrightarrow \text{RPM Command}$$

---

## 11. Phase 3 — D435i Perception Pipeline

The Intel RealSense D435i camera provides RGB images ($1920\times1080$), Depth maps ($1280\times720$), and dense 3D Point Clouds (`/camera/depth/points`).

```
                              D435i RealSense Camera
                                        │
             ┌──────────────────────────┼──────────────────────────┐
             ▼                          ▼                          ▼
      RGB Image Raw             Depth Image Raw               Point Cloud 3D
             │                          │                          │
             ▼                          ▼                          ▼
     Object Detection           Obstacle Detection         Ground & Terrain Model
```

---

## 12. 5 Independent Perception Modules

To prevent single-point software failures, perception is divided into five isolated ROS 2 nodes:

```
                            D435i Data Streams
                                     │
       ┌──────────────┬──────────────┼──────────────┬──────────────┐
       ▼              ▼              ▼              ▼              ▼
  Object          Obstacle        Terrain         No-Go          Cliff / Drop
  Detector        Detector        Detector       Detector        Detector
(/perception/   (/perception/   (/perception/  (/perception/   (/perception/
  objects)        obstacles)       terrain)        no_go)         cliffs)
```

---

## 13. 3D Obstacle Detection & Ground Removal Engine

Processing raw point cloud data to filter out flat ground and extract physical obstacles:

```
Raw Point Cloud ──► Passthrough Filter ──► RANSAC Ground Removal ──► Non-Ground Points ──► Clustered Obstacles ──► Costmap Layer
```

1. **Passthrough Filtering:** Crop ROI to $[x: -2.0\text{m to } +2.0\text{m}, y: -1.0\text{m to } +5.0\text{m}, z: -0.5\text{m to } +2.0\text{m}]$.
2. **Plane Segmentation:** Use RANSAC plane fitting to identify the dominant horizontal ground plane.
3. **Outlier Extraction:** Extract points residing $>15\text{ cm}$ above the fitted ground plane as true obstacles.

---

## 14. Ground Plane Estimation Algorithm

Points satisfying the plane equation $|ax + by + cz + d| < \epsilon$ (where $\epsilon = 0.05\text{ m}$) are classified as traversable ground plane and stripped from the obstacle point cloud.

---

## 15. Cliff / Vertical Drop Safety Detector

Vertical drops, trenches, and steep cliffs pose immediate structural failure risks to the rover.

```
       ROVER
     ████████
    ───────────┐
               │  ◄── DETECTED DISCONTINUITY (CLIFF)
               │
```

### Detection Logic:
1. Scan depth sensor rays projected at a fixed downward angle $\alpha$.
2. Measure expected range $R_{\text{expected}} = \frac{h_{\text{camera}}}{\sin(\alpha + \text{pitch})}$.
3. If measured depth exceeds $R_{\text{expected}} + \Delta_{\text{threshold}}$ (where $\Delta = 0.35\text{ m}$), trigger an immediate **CLIFF ALERT**.
4. Publish `/perception/cliffs` and signal `/safety/stop` to override drive controls.

---

## 16. Terrain & Slope Angle Detection

Local point cloud normal estimation determines terrain inclination angle $\theta$:
$$\theta = \arccos(\mathbf{n}_{\text{local}} \cdot \mathbf{n}_{\text{up}})$$

- If $\theta \le \theta_{\text{safe}}$ (e.g., $20^\circ$): **TRAVERSABLE (FREE)**
- If $\theta > \theta_{\text{safe}}$: **NON-TRAVERSABLE (COST = HIGH / OBSTACLE)**

---

## 17. Red / Contaminated Zone Perception (IRC Rule Compliance)

IRC rules penalize entry into red-marked contaminated regions by **10% of total score**.

```
RGB Image ──► HSV Conversion ──► Red Hue Mask ──► Depth Projection ──► Costmap Inflation (Cost = 255)
```

- **HSV Thresholds:** $H_1 \in [0, 10], H_2 \in [170, 180], S \in [100, 255], V \in [100, 255]$.
- **Costmap Encoding:** Extracted red contours are projected into 3D world space and inscribed as forbidden costmap polygons with infinite traversal cost ($255$).

---

## 18. Red Object vs Red Zone Spatial Mapping

Red objects (e.g., red cones, red markers) are distinct from red floor zones:
- **Red Cones/Markers:** Processed as 3D point obstacles with inflation radius.
- **Red Surface Regions:** Processed as planar no-go polygons on the ground map.

---

## 19. Phase 4 — World Model & Multi-Layer Costmap

The Nav2 Costmap stack aggregates multiple spatial layers into a unified grid:

```
             WORLD MODEL
                  │
       ┌──────────┼───────────┐
       ▼          ▼           ▼
   Obstacles   Terrain     No-Go
       │          │           │
       └──────────┼───────────┘
                  ▼
               COSTMAP
```

### Standard Cost Values:
- `0`: Free Space
- `1 - 9`: Clear Flat Surface
- `10 - 49`: Rough Terrain / Soft Sand
- `50 - 98`: Steep Inclines / Minor Rocks
- `99 - 254`: Obstacle Inflation Safety Buffer
- `255`: Forbidden (Lethal Obstacle / Cliff / Red Zone)

---

## 20. Costmap Layer Configuration Matrix

```yaml
costmap:
  plugins: ["static_layer", "obstacle_layer", "red_zone_layer", "cliff_layer", "inflation_layer"]
  obstacle_layer:
    plugin: "nav2_costmap_2d::ObstacleLayer"
    observation_sources: pointcloud
    pointcloud:
      topic: /perception/obstacles
      clearing: true
      marking: true
  inflation_layer:
    plugin: "nav2_costmap_2d::InflationLayer"
    inflation_radius: 0.75 # meters
    cost_scaling_factor: 3.0
```

---

## 21. Phase 5 — Hierarchical Navigation Architecture

Navigation operates across two complementary planning layers:

```
GLOBAL PLANNER (A* / Dijkstra) ──► Computes coarse path from current pose to GPS Goal
                                          │
LOCAL PLANNER (TEB / DWA / RPP) ◄─────────┴──► Computes dynamic collision-free velocity commands
```

```
Global Planner Example:
START ──────────────────► [ROCK (Avoid)] ──────────────────► GOAL

Local Planner Dynamic Reaction:
           ─────────╮
                    │  [Dynamic Rock]
           ─────────╯ ──────────────► Path Restored
```

---

## 22. Continuous High-Frequency Control Loop

Executed continuously at $20\text{ Hz}$ ($50\text{ ms}$ interval):

$$\text{Sensors} \rightarrow \text{Pose (EKF)} \rightarrow \text{Perception} \rightarrow \text{Costmap} \rightarrow \text{Local Planner} \rightarrow /cmd\_vel \rightarrow \text{Kinematics} \rightarrow \text{RPM Cmd} \rightarrow \text{STM32}$$

---

## 23. Motion Controller Logic

Generates target velocities $(v, \omega)$ based on path curvature and proximity to obstacles:
- **Max Linear Speed ($v_{\text{max}}$):** $1.2\text{ m/s}$
- **Max Angular Speed ($\omega_{\text{max}}$):** $1.0\text{ rad/s}$

---

## 24. Differential Kinematics Node Engine

Translates ROS `/cmd_vel` ($v, \omega$) into left/right motor RPM commands:

$$\text{RPM}_{\text{left}} = \frac{v - \frac{\omega L}{2}}{2\pi r} \times 60$$
$$\text{RPM}_{\text{right}} = \frac{v + \frac{\omega L}{2}}{2\pi r} \times 60$$

---

## 25. Closed-Loop PID RPM Feedback Control

To maintain true straight-line tracking under variable ground friction:

```
Desired RPM ──► [ + ] ──► PID Controller ──► PWM Command ──► STM32 ──► Motor ──► Encoder
                 ▲ -                                                               │
                 └────────────────────── Actual RPM Feedback ──────────────────────┘
```

---

## 26. Phase 6 — Target Object Detection (RADO Task)

For IRC RADO tasks requiring autonomous search, identification, and localization of scattered equipment (Toolbox, Hammer, Wrench, Supply Containers, Rocks up to 5 kg):

```
RGB Camera Stream ──► Object Detection Network (YOLO/OpenCV) ──► Bounding Box (u, v) + Label
```

---

## 27. 3D Object Pose Transformation Pipeline

1. Extract pixel center $(u, v)$ of detected object.
2. Query corresponding depth value $Z = \text{DepthImage}(u, v)$.
3. Project pixel to 3D Camera Frame:
   $$X_c = \frac{(u - c_x) \cdot Z}{f_x}, \quad Y_c = \frac{(v - c_y) \cdot Z}{f_y}, \quad Z_c = Z$$
4. Transform camera coordinate $(X_c, Y_c, Z_c)$ to robot `base_link` frame via static TF.

---

## 28. Speed-Adaptive Goal Approach Protocol

As the rover approaches a targeted delivery item or placement zone:

| Distance to Target ($D$) | Speed Profile | Active Controller Mode |
|---|---|---|
| $D > 2.0\text{ m}$ | High ($1.0\text{ m/s}$) | Global Path Navigation |
| $0.5\text{ m} < D \le 2.0\text{ m}$ | Medium ($0.4\text{ m/s}$) | Fine Vision Alignment |
| $0.2\text{ m} < D \le 0.5\text{ m}$ | Slow ($0.15\text{ m/s}$) | Precise Docking / Creep |
| $D \le 0.2\text{ m}$ | Stop ($0.0\text{ m/s}$) | Manipulation / Release Mode |

---

## 29. RADO Autonomous Mode Software Lock

Per IRC rules during autonomous delivery:
- **OPERATORS PERMITTED:** Video telemetry monitoring & system state display.
- **OPERATORS PROHIBITED:** Steering commands, velocity adjustments, arm control inputs, manual path overrides.

---

## 30. Operation Mode State Gatekeeper

```
                          SYSTEM_MODE
                              │
       ┌──────────────────────┼──────────────────────┐
       ▼                      ▼                      ▼
    TELEOP                AUTONOMOUS           EMERGENCY_STOP
 (Manual Drive)         (Software Lock)      (All Motors Inhibited)
       │                      │                      │
 Joystick Active      Joystick IGNORED       Hard Hardware Brake
```

---

## 31. Autonomous Delivery State Machine Architecture

```
                    IDLE
                     │
                     ▼
               MISSION START
                     │
                     ▼
               INITIAL CHECK
                     │
                     ▼
              RECONNAISSANCE
                     │
                     ▼
              OBJECT DATABASE
                     │
                     ▼
              SELECT OBJECT
                     │
                     ▼
              PLAN DELIVERY
                     │
                     ▼
             ENABLE AUTONOMY
                     │
                     ▼
              NAVIGATE OBJECT
                     │
                     ▼
             OBJECT PICKUP
                     │
                     ▼
              PLAN DELIVERY
                     │
                     ▼
            AUTONOMOUS DRIVE
                     │
              ┌──────┴──────┐
              ▼             ▼
          OBSTACLE       FAILURE
              │             │
              ▼             ▼
           REPLAN       RECOVERY
              │             │
              └──────┬──────┘
                     ▼
             APPROACH GOAL
                     │
                     ▼
               PLACE OBJECT
                     │
                     ▼
              VERIFY PLACEMENT
                     │
                ┌────┴────┐
                ▼         ▼
             SUCCESS     FAIL
                │         │
                ▼         ▼
            NEXT TASK   RECOVER
                │
                ▼
              FINISH
```

---

## 32. Reconnaissance Phase Specification

During the initial 10-minute reconnaissance window allowed in RADO:
1. Rover sweeps designated area.
2. Perception node detects objects, computes estimated GPS coordinates & confidence scores.
3. Automatically writes records to central `Object Database`.

---

## 33. Strict Competition Rule Compliance: No Scouting

Prior to official mission timer start, **no pre-scanning or scouting** of the arena is permitted. Maps generated in practice runs cannot be used as static global maps during official scoring attempts.

---

## 34. Goal Localization & Landmark Verification

Target delivery locations are given as approximate GPS coordinates.
- Rover navigates to within the GPS uncertainty ellipse ($1 - 3\text{ m}$).
- Switches from global GPS guidance to local visual marker detection (ArUco / Color boundary / Geometry detection) for final precision alignment.

---

## 35. Delivery Placement Verification Lifecycle

Before declaring delivery success, the system executes an automated verification check:

```
Target Zone Detected ──► Correct Object Label? ──► Object Released? ──► Object Inside Polygon? ──► MISSION SUCCESS
```

---

## 36. Phase 7 — Parallel Safety Manager Specification

The Safety Manager node operates independently of all planners, continuously monitoring system telemetry:

```
              SAFETY NODE
                   │
       ┌───────────┼────────────┐
       ▼           ▼            ▼
   Obstacle      Cliff     Localization
   Distance     Detector       Lost
       │           │            │
       └───────────┼────────────┘
                   ▼
                STOP
```

---

## 37. Mandatory Safety Interruption Conditions

1. **Proximity Alert:** Obstacle distance $< 0.4\text{ m}$.
2. **Cliff Alert:** Drop detected in front of path.
3. **Localization Loss:** EKF covariance trace $> \text{threshold}$.
4. **Sensor Timeout:** Loss of D435i or IMU data for $> 300\text{ ms}$.
5. **Encoder Fault:** Mismatch between command RPM and encoder feedback under flat drive.
6. **Watchdog Failure:** Base station heartbeat drop $> 2.0\text{ s}$.

---

## 38. Communication Loss & Frequency Management

IRC field environments feature heavy RF congestion.
- System automatically monitors ping / heartbeat.
- **Loss $< 3.0\text{ s}$:** Continue autonomous execution with elevated caution mode ($50\%$ max speed).
- **Loss $> 3.0\text{ s}$:** Bring rover to a safe, controlled stop (`/safety/stop = True`).

---

## 39. Mission Time Awareness Strategy

For 30-minute IRC field missions:
- **10-Minute Checkpoint Rule:** If 0 points scored in first 10 minutes, abort current approach and switch to high-probability fallback task.
- **20-Minute Checkpoint Rule:** If points scored $< 30\%$ of total at 20 minutes, abort complex tasks and focus exclusively on high-value delivery points.

---

## 40. Priority & Risk-Aware Task Scheduler

Task priority evaluation function:

$$\text{Priority Score} = \frac{\text{Expected Points}}{\text{Estimated Time (min)}} \times (1 - \text{Terrain Risk Factor}) \times P_{\text{success}}$$

---

## 41. Intervention Handling Protocol

- **Limits:** Max 4 total interventions in finals, max 2 per mission (each incurs a **20% score penalty**).
- **Policy:** Software recovery routines MUST attempt self-healing before requesting manual human intervention.

---

## 42. Automated Failure Recovery Routines

```
                       RECOVERY MANAGER
                              │
     ┌────────────────────────┼────────────────────────┐
     ▼                        ▼                        ▼
Obstacle Lockout       GPS Signal Degraded      Path Planning Failure
(Clear Costmap &        (Fallback to IMU +       (360° Rotate Scan &
 Re-plan Local)          Encoder + Visual)        Rebuild World Map)
```

---

## 43. Autonomous Bio-Exploration (ABEx) Mission State Machine

```
SITE SELECTION ──► NAVIGATE ──► DOCUMENTATION ──► PANORAMA ──► CLOSE-UP ──► SAMPLE LOCATION ──► ARM DEPLOY ──► 10cm+ SAMPLE ──► SEAL ──► ANALYZE ──► CACHE ──► RECORD GPS
```

---

## 44. Equipment Servicing (IDMO) Assistance Architecture

Visual assistance pipeline for panel servicing tasks (switches, buttons, knobs, drawers up to 1.5m height):

```
Panel Detection ──► Pose Estimation ──► Arm Kinematics ──► End-Effector Align ──► Action ──► Visual Verification
```

---

## 45. Complete System Operation Modes Matrix

```
                 ROVER
                   │
       ┌───────────┼────────────┐
       ▼           ▼            ▼
     TELEOP      AUTONOMY     SAFETY
       │           │            │
       ▼           ▼            ▼
 Manual       Mission        Emergency
 Driving      Manager        Stop
```

### Autonomy Sub-Modes:
- `AUTONOMOUS_NAVIGATION`
- `AUTONOMOUS_RECON`
- `AUTONOMOUS_DELIVERY`
- `AUTONOMOUS_RETURN`
- `AUTONOMOUS_RECOVERY`
- `ABEX_MODE`
- `IDMO_ASSIST`

---

## 46. Complete Jetson ROS 2 Workspace Structure

```
rover_ws/
└── src/
    ├── rover_bringup/
    │   ├── config/
    │   │   └── rover_system.yaml
    │   └── launch/
    │       └── rover_bringup.launch.py
    ├── rover_description/
    │   ├── meshes/
    │   ├── urdf/
    │   │   └── rover.urdf.xacro
    │   └── launch/
    │       └── rsp.launch.py
    ├── rover_interfaces/
    │   ├── msg/
    │   │   ├── WheelRPM.msg
    │   │   ├── MissionStatus.msg
    │   │   └── TerrainState.msg
    │   └── srv/
    │       └── SetMode.srv
    ├── rover_sensors/
    │   ├── gps_node.py
    │   ├── imu_node.py
    │   ├── encoder_node.py
    │   └── d435i_node.py
    ├── rover_localization/
    │   ├── ekf_node.py
    │   ├── gps_fusion.py
    │   └── localization_monitor.py
    ├── rover_perception/
    │   ├── object_detector.py
    │   ├── obstacle_detector.py
    │   ├── terrain_detector.py
    │   ├── no_go_detector.py
    │   └── cliff_detector.py
    ├── rover_mapping/
    │   ├── map_manager.py
    │   └── costmap_node.py
    ├── rover_navigation/
    │   ├── global_planner.py
    │   ├── local_planner.py
    │   ├── path_follower.py
    │   └── goal_manager.py
    ├── rover_control/
    │   ├── velocity_controller.py
    │   ├── kinematics.py
    │   └── rpm_controller.py
    ├── rover_stm_bridge/
    │   ├── uart_bridge.py
    │   ├── can_bridge.py
    │   └── watchdog.py
    ├── rover_mission/
    │   ├── mission_manager.py
    │   ├── rado_manager.py
    │   ├── abex_manager.py
    │   └── idmo_manager.py
    ├── rover_safety/
    │   ├── emergency_stop.py
    │   ├── cliff_safety.py
    │   ├── collision_safety.py
    │   ├── localization_safety.py
    │   └── system_watchdog.py
    └── rover_tools/
        ├── calibration.py
        ├── diagnostics.py
        └── bag_recording.py
```

---

## 47. Complete End-to-End System Computation Graph

```
GPS ───────────────┐
                   │
IMU ───────────────┼──► SENSOR FUSION ──► /odometry/fused ──► LOCALIZATION ──► /tf ──┐
                   │        (EKF)                                                      │
ENCODER ───────────┘                                                                   │
                                                                                       ▼
D435i ──► PERCEPTION ENGINE ──► OBSTACLES / CLIFFS / RED ZONES ──► COSTMAP ──► GLOBAL + LOCAL PLANNER
                                                                                       │
                                                                                       ▼
                                                                                   /cmd_vel
                                                                                       │
                                                                                       ▼
                                                                             KINEMATICS ENGINE
                                                                                       │
                                                                                       ▼
                                                                                 RPM COMMAND
                                                                                       │
                                                                                       ▼
                                                                                     STM32
                                                                                       │
                                                                                       ▼
                                                                                    MOTORS
                                                                                       │
                                                                                       ▼
                                                                                    ENCODERS
                                                                                       │
                                                                                       └─► FEEDBACK
```

---

## 48. Parallel Safety Graph & Emergency Override Flow

```
D435i Camera ────► Obstacle Detector ──┐
                                       │
D435i Depth ─────► Cliff Detector ─────┤
                                       │
GPS / IMU ───────► Localization Monitor├──► SAFETY MANAGER ──► Emergency STOP Override ──► Motor Lock
                                       │
STM32 ───────────► Motor Diagnostics  │
                                       │
Base Comm ───────► Watchdog Heartbeat ─┘
```

---

## 49. Modular Control Isolation Rule

> **Design Principle:** High-level planners MUST NEVER publish direct motor commands or wheel RPMs. Planners publish standard `geometry_msgs/msg/Twist` (`/cmd_vel`). The `rover_control` kinematics package converts `/cmd_vel` to RPMs. This allows changing wheel geometry, motor drivers, or drive types without modifying navigation algorithms.

---

## 50. 19-Step Comprehensive Verification & Testing Protocol

1. **Test 1:** Raw encoder counting & direction verification.
2. **Test 2:** Encoder-only dead reckoning odometry ($1\text{m}$ straight, $90^\circ$ spin).
3. **Test 3:** IMU heading integration & drift rate measurement.
4. **Test 4:** GPS UTM coordinate transformation accuracy.
5. **Test 5:** Dual EKF sensor fusion (`/odometry/fused` stability test).
6. **Test 6:** D435i point cloud alignment & ground plane extraction.
7. **Test 7:** Obstacle detection at static ranges ($0.5\text{m}$, $1.0\text{m}$, $2.0\text{m}$).
8. **Test 8:** Cliff detector activation on simulated drop edge.
9. **Test 9:** Red zone color segmentation under full outdoor sunlight.
10. **Test 10:** Slope angle estimation on $15^\circ$ vs $25^\circ$ ramps.
11. **Test 11:** Nav2 costmap inflation layer generation.
12. **Test 12:** Global path planning around static box obstacle.
13. **Test 13:** Local planner dynamic obstacle avoidance during motion.
14. **Test 14:** Object detection & 3D coordinate estimation.
15. **Test 15:** Speed-adaptive goal approach decelerations.
16. **Test 16:** Base station comm drop simulation (Watchdog auto-stop within $2\text{s}$).
17. **Test 17:** Wheel slip detection on loose sand track.
18. **Test 18:** 20-Minute RADO delivery autonomous field simulation.
19. **Test 19:** 30-Minute Full Competition Mock Run under IRC rules.

---

## 51. Progressive Field Testing Protocol

- **Level 1 (Flat Tarmac Straight):** Validate baseline kinematic velocity tracking & EKF pose stability.
- **Level 2 (Single Box Avoidance):** Validate D435i obstacle detection, costmap marking, and dynamic local path deviation.
- **Level 3 (Multi-Feature Arena):** Validate simultaneous obstacle avoidance, red zone avoidance, and slope negotiation.
- **Level 4 (IRC Benchmark Field):** Execute full autonomous RADO search, pickup, autonomous drive, and verified delivery.

---

## 52. Realistic IRC RADO Execution Lifecycle

1. System Initialization & Self-Diagnostics.
2. EKF Localization Convergence Check.
3. 10-Minute Reconnaissance Sweep & Object Logging.
4. Target Selection from Object Database.
5. Target Approach & Pickup Alignment.
6. **ENGAGE AUTONOMOUS MODE SOFTWARE LOCK**.
7. Global Route Generation to Target GPS.
8. Real-time Local Obstacle Avoidance via D435i.
9. Red Contaminated Zone Bypass via Costmap Constraint.
10. Steep Slope Angle Check & Safe Route Deviation.
11. Approach Target Delivery Zone with Adaptive Deceleration.
12. Fine Landmark/Marker Visual Localization.
13. Precision Object Placement.
14. Automated Placement & Alignment Verification.
15. Success Confirmation Message to Mission Control.
16. Transition to Next Target Task.

---

## 53. IRC Rulebook Constraint Matrix

| Rulebook Requirement | System Implementation | Architectural Solution |
|---|---|---|
| **500m Operation Range** | Global GPS + EKF Fusion | Dual EKF global localization |
| **Approximate Delivery GPS** | Precision Visual Docking | ArUco / Color marker visual fine alignment |
| **Loose Sand Terrain** | Slip-Aware State Estimation | IMU + Encoder velocity variance gating |
| **Boulders & Rough Rocks** | 3D Depth Point Cloud Filtering | RANSAC ground removal + obstacle clustering |
| **Vertical Drop Hazard** | Dedicated Cliff Detector | Ray-cast depth discontinuity safety node |
| **Steep Slope Constraints** | Terrain Slope Angle Analysis | Normal vector angle calculation vs threshold |
| **Red Zone Avoidance (10% Penalty)** | Color Mask Costmap Layer | HSV Red segmentation with infinite cost ($255$) |
| **Autonomous Delivery Task** | Autonomy Lock State Machine | Manual override command lockout gate |
| **30-Minute Mission Limit** | Mission Clock & Priority Engine | Time-aware task scheduling & point optimization |
| **Intervention Rules (20% Penalty)** | Multi-Tiered Recovery Engine | Autonomous self-healing routines before manual pause |
| **Comm Interference** | Comm Watchdog & RF Hopping | Heartbeat loss fail-safe stop trigger |
| **65 kg / 1.5m Mass & Size Bounds** | Mechanical & Envelope Design | Structural compliance and URDF footprint matching |

---

## 54. 10-Phase Phased Implementation Roadmap

1. **Step 1:** Establish workspace packages & URDF transforms.
2. **Step 2:** Write STM32 UART/CAN bridge and wheel velocity RPM publisher.
3. **Step 3:** Implement differential drive kinematics and wheel odometry node.
4. **Step 4:** Configure `robot_localization` dual EKF for sensor fusion.
5. **Step 5:** Integrate D435i driver and write 3D obstacle & ground removal node.
6. **Step 6:** Implement Cliff/Drop safety detector and Red Zone color mask node.
7. **Step 7:** Configure Nav2 global/local costmaps and planners.
8. **Step 8:** Build Object Detector node and 3D camera-to-robot coordinate transformer.
9. **Step 9:** Write Mission Manager & Autonomous Delivery State Machine.
10. **Step 10:** Conduct progressive field testing (Tests 1–19) and competition trials.

---

## 55. Existing System Upgrade Roadmap

To transition your existing repository (`rover_ws`) to this full architecture:

1. **Keep Existing Base Nodes:** Retain core kinematics (`rover_kinematics`) and basic camera interfaces (`rover_camera`).
2. **Integrate Interface Messages:** Expand `rover_interfaces` with `WheelRPM.msg`, `MissionStatus.msg`, and `TerrainState.msg`.
3. **Upgrade Perception:** Expand `rover_perception` into 5 isolated nodes (`obstacle_node.py`, `cliff_node.py`, `red_zone_node.py`, `terrain_node.py`, `object_node.py`).
4. **Deploy Sensor Fusion:** Add `rover_localization` running `robot_localization` EKF with GPS, IMU, and Wheel Encoders.
5. **Add Mission & Safety Controllers:** Create `rover_mission` and `rover_safety` packages to enforce autonomy mode locks, task state machines, and emergency stop watchdogs.

import pyrealsense2 as rs
import numpy as np
import cv2
import math
import time
from collections import deque

# ==========================================
# CALIBRATED VEHICLE & LOGIC PARAMETERS
# ==========================================
CAM_MOUNT_HEIGHT = 0.40   # Height above ground in meters (40 cm)
CAM_PITCH_DEG    = 15.0   # Downward pitch angle in degrees

STOP_SAFETY_DIST = 0.40   # Emergency stop distance buffer (40 cm)
MAX_CLIMB_HEIGHT = 0.50   # Max climbable height threshold (50 cm)
CHASSIS_CLR_HGHT = 0.35   # Safe chassis under-belly clearance (35 cm)
TRACK_SAFE_WIDTH = 0.40   # Safe wheel track width clearance (40 cm)

GROUND_MARGIN    = 0.03   # 3 cm ground noise tolerance
MIN_DIST_M       = 0.30   # Minimum valid perception distance (30 cm)
MAX_DIST_M       = 2.50   # Maximum forward inspection distance (2.5 m)
MIN_CLUSTER_PX   = 800    # Minimum pixel count for valid obstacle

# Confidence & Output Gating Parameters
CONFIDENCE_FRAME_THRESHOLD = 5   # Must be detected across 5 consecutive frames
LOG_TIME_INTERVAL          = 1.0 # Throttle periodic prints to once per 1.0s

PITCH_RAD = math.radians(CAM_PITCH_DEG)
COS_PITCH = math.cos(PITCH_RAD)
SIN_PITCH = math.sin(PITCH_RAD)

# Moving average metric buffers (5 frames)
dist_buffer = deque(maxlen=5)
width_buffer = deque(maxlen=5)
height_buffer = deque(maxlen=5)

def evaluate_decision(dist_m, width_m, height_m):
    """
    Returns: (decision_string, action_code, color_bgr)
    Action Codes:
      0 = PATH CLEAR
      1 = STRADDLE
      2 = CLIMB / CROSS
      3 = DETOUR
      4 = STOP / PATH BLOCKED
    """
    if dist_m < STOP_SAFETY_DIST:
        return "EMERGENCY STOP (TOO CLOSE)", 4, (0, 0, 255)
    
    if height_m <= MAX_CLIMB_HEIGHT:
        if height_m <= CHASSIS_CLR_HGHT and width_m <= 0.30:
            return "STRADDLE (UNDER-BELLY)", 1, (255, 255, 0)
        else:
            return "CLIMB / CROSS", 2, (0, 255, 0)
    else:
        if width_m <= TRACK_SAFE_WIDTH:
            return "DETOUR (NARROW OBSTACLE)", 3, (0, 165, 255)
        else:
            return "PATH BLOCKED (FULL STOP)", 4, (0, 0, 255)

def main():
    pipeline = rs.pipeline()
    config = rs.config()

    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    profile = pipeline.start(config)
    align = rs.align(rs.stream.color)

    # RealSense SDK Filters
    spatial_filter = rs.spatial_filter()
    temporal_filter = rs.temporal_filter()
    hole_filling = rs.hole_filling_filter()

    depth_profile = profile.get_stream(rs.stream.depth).as_video_stream_profile()
    intrinsics = depth_profile.get_intrinsics()

    u_grid, v_grid = np.meshgrid(np.arange(640), np.arange(480))

    # Output gating state tracking
    consecutive_frames = 0
    last_action_code   = None
    last_log_time      = 0.0

    print("[INFO] Rover Perception Active. Confidence Gating Enabled.")
    print("[INFO] Press 'q' to exit.")

    try:
        while True:
            frames = pipeline.wait_for_frames()
            aligned = align.process(frames)

            depth_frame = aligned.get_depth_frame()
            color_frame = aligned.get_color_frame()

            if not depth_frame or not color_frame:
                continue

            filtered_depth = spatial_filter.process(depth_frame)
            filtered_depth = temporal_filter.process(filtered_depth)
            filtered_depth = hole_filling.process(filtered_depth)

            color_img = np.asanyarray(color_frame.get_data())
            depth_img = np.asanyarray(filtered_depth.get_data())
            depth_m = depth_img * depth_frame.get_units()

            roi_x1, roi_x2 = int(640 * 0.20), int(640 * 0.80)
            roi_y1, roi_y2 = int(480 * 0.10), int(480 * 0.95)

            x_cam = (u_grid - intrinsics.ppx) * depth_m / intrinsics.fx
            y_cam = (v_grid - intrinsics.ppy) * depth_m / intrinsics.fy
            z_cam = depth_m

            y_world = y_cam * COS_PITCH + z_cam * SIN_PITCH
            z_world = -y_cam * SIN_PITCH + z_cam * COS_PITCH
            height_above_ground = CAM_MOUNT_HEIGHT - y_world

            roi_mask = (u_grid >= roi_x1) & (u_grid < roi_x2) & (v_grid >= roi_y1) & (v_grid < roi_y2)
            obstacle_mask = (
                roi_mask &
                (depth_m > MIN_DIST_M) & 
                (depth_m < MAX_DIST_M) & 
                (height_above_ground > GROUND_MARGIN)
            )

            binary_obs = (obstacle_mask * 255).astype(np.uint8)
            kernel = np.ones((5, 5), np.uint8)
            cleaned_obs = cv2.morphologyEx(binary_obs, cv2.MORPH_OPEN, kernel)

            num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(cleaned_obs)
            
            raw_detected = False
            label_idx_to_use = -1
            max_area = 0

            for idx in range(1, num_labels):
                area = stats[idx, cv2.CC_STAT_AREA]
                if area >= MIN_CLUSTER_PX and area > max_area:
                    max_area = area
                    label_idx_to_use = idx
                    raw_detected = True

            # Track temporal persistence
            if raw_detected:
                consecutive_frames += 1
            else:
                consecutive_frames = 0
                dist_buffer.clear()
                width_buffer.clear()
                height_buffer.clear()

            confidence_pct = min(100, int((consecutive_frames / CONFIDENCE_FRAME_THRESHOLD) * 100))
            is_sure = consecutive_frames >= CONFIDENCE_FRAME_THRESHOLD

            obstacle_data = {
                "obstacle_detected": False,
                "surety": False,
                "confidence_pct": confidence_pct,
                "distance_m": 0.0,
                "width_m": 0.0,
                "height_m": 0.0,
                "decision": "PATH CLEAR",
                "action_code": 0
            }

            if is_sure and label_idx_to_use != -1:
                component_mask = (labels == label_idx_to_use)

                z_cls = z_world[component_mask]
                x_cls = x_cam[component_mask]
                h_cls = height_above_ground[component_mask]

                inst_dist   = float(np.percentile(z_cls, 5))
                inst_width  = float(np.percentile(x_cls, 95) - np.percentile(x_cls, 5))
                inst_height = float(np.percentile(h_cls, 95))

                dist_buffer.append(inst_dist)
                width_buffer.append(inst_width)
                height_buffer.append(inst_height)

                dist_m   = float(np.mean(dist_buffer))
                width_m  = max(float(np.mean(width_buffer)), 0.05)
                height_m = max(float(np.mean(height_buffer)), 0.02)

                decision, action_code, color_status = evaluate_decision(dist_m, width_m, height_m)

                obstacle_data = {
                    "obstacle_detected": True,
                    "surety": True,
                    "confidence_pct": 100,
                    "distance_m": round(dist_m, 2),
                    "width_m": round(width_m, 2),
                    "height_m": round(height_m, 2),
                    "decision": decision,
                    "action_code": action_code
                }

                x_box, y_box = stats[label_idx_to_use, cv2.CC_STAT_LEFT], stats[label_idx_to_use, cv2.CC_STAT_TOP]
                w_box, h_box = stats[label_idx_to_use, cv2.CC_STAT_WIDTH], stats[label_idx_to_use, cv2.CC_STAT_HEIGHT]
                cv2.rectangle(color_img, (x_box, y_box), (x_box + w_box, y_box + h_box), color_status, 2)

                cv2.rectangle(color_img, (10, 10), (370, 175), (0, 0, 0), -1)
                cv2.putText(color_img, f"Obstacle Detected: YES", (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
                cv2.putText(color_img, f"Surety:            100% CONFIDENT", (20, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 255, 0), 1)
                cv2.putText(color_img, f"Distance (Depth):  {obstacle_data['distance_m']:.2f} m", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
                cv2.putText(color_img, f"Width:             {obstacle_data['width_m']:.2f} m", (20, 103), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
                cv2.putText(color_img, f"Height:            {obstacle_data['height_m']:.2f} m", (20, 126), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
                cv2.putText(color_img, f"Decision:          {obstacle_data['decision']}", (20, 149), cv2.FONT_HERSHEY_SIMPLEX, 0.50, color_status, 2)
                cv2.putText(color_img, f"Action Code:       {obstacle_data['action_code']}", (20, 168), cv2.FONT_HERSHEY_SIMPLEX, 0.50, color_status, 2)

            else:
                cv2.rectangle(color_img, (10, 10), (370, 55), (0, 0, 0), -1)
                if raw_detected:
                    cv2.putText(color_img, f"Evaluating Noise... ({confidence_pct}%)", (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)
                else:
                    cv2.putText(color_img, "Obstacle Detected: NO (CLEAR)", (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

            # ==========================================
            # CONFIDENCE LOGIC: CONTROLLED TERMINAL LOGGING
            # ==========================================
            current_time = time.time()
            state_changed = (obstacle_data['action_code'] != last_action_code)
            timer_elapsed = (current_time - last_log_time) >= LOG_TIME_INTERVAL

            # Only print when HIGH SURETY condition is met AND (State changed OR 1s timer passed)
            if obstacle_data['surety'] and (state_changed or timer_elapsed):
                print(f"[HIGH CONFIDENCE PAYLOAD] {obstacle_data}")
                last_action_code = obstacle_data['action_code']
                last_log_time = current_time
            elif state_changed and not obstacle_data['surety'] and last_action_code != 0 and last_action_code is not None:
                # Clear path state event
                print(f"[HIGH CONFIDENCE PAYLOAD] {obstacle_data}")
                last_action_code = 0
                last_log_time = current_time

            cv2.rectangle(color_img, (roi_x1, roi_y1), (roi_x2, roi_y2), (255, 255, 0), 1)
            cv2.imshow("Rover Obstacle Perception Test", color_img)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
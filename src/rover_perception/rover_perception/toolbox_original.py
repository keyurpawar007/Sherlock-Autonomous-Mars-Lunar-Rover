import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO
from collections import deque

# =========================
# SETTINGS
# =========================
MODEL_PATH = "best.pt"
CONFIDENCE = 0.50

MIN_DEPTH = 0.30
MAX_DEPTH = 5.00

DEPTH_ROI_RATIO = 0.30
SMOOTHING_FRAMES = 5

# Rover considers toolbox reached at this distance
GOAL_REACHED_DISTANCE = 0.35

# =========================
# LOAD YOLO MODEL
# =========================
model = YOLO(MODEL_PATH)

# =========================
# REALSENSE SETUP
# =========================
pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(
    rs.stream.depth,
    640,
    480,
    rs.format.z16,
    30
)

config.enable_stream(
    rs.stream.color,
    640,
    480,
    rs.format.bgr8,
    30
)

profile = pipeline.start(config)

# Get depth scale BEFORE applying filters
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

print("Depth scale:", depth_scale)

# =========================
# ALIGN DEPTH TO COLOR
# =========================
align = rs.align(rs.stream.color)

# Because depth is aligned to color,
# use COLOR camera intrinsics for XYZ calculation.
color_profile = profile.get_stream(rs.stream.color)
color_intrinsics = (
    color_profile
    .as_video_stream_profile()
    .get_intrinsics()
)

# =========================
# DEPTH FILTERS
# =========================
spatial = rs.spatial_filter()
temporal = rs.temporal_filter()
hole_filling = rs.hole_filling_filter()

# =========================
# DEPTH SMOOTHING
# =========================
depth_history = deque(
    maxlen=SMOOTHING_FRAMES
)

print("====================================")
print(" TOOLBOX GOAL DETECTION STARTED")
print("====================================")
print("Rover is continuously searching for toolbox...")
print("Press Q to stop.")

try:

    while True:

        # =========================
        # GET CAMERA FRAMES
        # =========================
        frames = pipeline.wait_for_frames()

        # Align depth with RGB
        aligned_frames = align.process(frames)

        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if not depth_frame or not color_frame:
            continue

        # =========================
        # APPLY DEPTH FILTERS
        # =========================
        depth_frame = spatial.process(depth_frame)
        depth_frame = temporal.process(depth_frame)
        depth_frame = hole_filling.process(depth_frame)

        # =========================
        # CONVERT TO NUMPY
        # =========================
        color_image = np.asanyarray(
            color_frame.get_data()
        )

        depth_image = np.asanyarray(
            depth_frame.get_data()
        )

        # Convert depth values to metres
        depth_m = depth_image * depth_scale

        # =========================
        # YOLO TOOLBOX DETECTION
        # =========================
        results = model(
            color_image,
            conf=CONFIDENCE,
            verbose=False
        )

        best_box = None
        best_confidence = 0

        # =========================
        # FIND TOOLBOX ONLY
        # =========================
        for result in results:

            if result.boxes is None:
                continue

            for box in result.boxes:

                class_id = int(box.cls[0])
                confidence = float(box.conf[0])

                class_name = model.names[class_id]

                # Ignore all objects except toolbox
                if class_name.lower() != "toolbox":
                    continue

                # Select highest-confidence toolbox
                if confidence > best_confidence:

                    best_confidence = confidence

                    coordinates = (
                        box.xyxy[0]
                        .cpu()
                        .numpy()
                    )

                    x1, y1, x2, y2 = map(
                        int,
                        coordinates
                    )

                    best_box = (
                        x1,
                        y1,
                        x2,
                        y2
                    )

        # =========================
        # TOOLBOX FOUND
        # =========================
        if best_box is not None:

            x1, y1, x2, y2 = best_box

            # Keep coordinates inside image
            x1 = max(0, x1)
            y1 = max(0, y1)
            x2 = min(639, x2)
            y2 = min(479, y2)

            box_width = x2 - x1
            box_height = y2 - y1

            # =========================
            # CENTRAL DEPTH ROI
            # =========================
            roi_x1 = int(
                x1 + box_width * DEPTH_ROI_RATIO
            )

            roi_x2 = int(
                x2 - box_width * DEPTH_ROI_RATIO
            )

            roi_y1 = int(
                y1 + box_height * DEPTH_ROI_RATIO
            )

            roi_y2 = int(
                y2 - box_height * DEPTH_ROI_RATIO
            )

            depth_roi = depth_m[
                roi_y1:roi_y2,
                roi_x1:roi_x2
            ]

            # =========================
            # VALID DEPTH
            # =========================
            valid_depth = depth_roi[
                (depth_roi >= MIN_DEPTH) &
                (depth_roi <= MAX_DEPTH)
            ]

            if len(valid_depth) > 20:

                # Median reduces depth noise
                toolbox_depth = float(
                    np.median(valid_depth)
                )

                # Store recent measurements
                depth_history.append(
                    toolbox_depth
                )

                # Smoothed toolbox distance
                smooth_depth = float(
                    np.median(depth_history)
                )

                # =========================
                # TOOLBOX CENTER
                # =========================
                center_x = int(
                    (x1 + x2) / 2
                )

                center_y = int(
                    (y1 + y2) / 2
                )

                # =========================
                # CALCULATE 3D POSITION
                # =========================
                point_3d = (
                    rs.rs2_deproject_pixel_to_point(
                        color_intrinsics,
                        [center_x, center_y],
                        smooth_depth
                    )
                )

                X = point_3d[0]
                Y = point_3d[1]
                Z = point_3d[2]

                # =========================
                # GOAL STATUS
                # =========================
                if smooth_depth <= GOAL_REACHED_DISTANCE:

                    goal_status = "GOAL REACHED"

                else:

                    goal_status = "GOAL AHEAD"

                # =========================
                # DRAW TOOLBOX BOX
                # =========================
                cv2.rectangle(
                    color_image,
                    (x1, y1),
                    (x2, y2),
                    (0, 255, 0),
                    2
                )

                # =========================
                # DRAW DEPTH ROI
                # =========================
                cv2.rectangle(
                    color_image,
                    (roi_x1, roi_y1),
                    (roi_x2, roi_y2),
                    (255, 0, 0),
                    2
                )

                # =========================
                # DRAW CENTER
                # =========================
                cv2.circle(
                    color_image,
                    (center_x, center_y),
                    5,
                    (0, 0, 255),
                    -1
                )

                # =========================
                # DISPLAY INFORMATION
                # =========================
                cv2.putText(
                    color_image,
                    "GOAL: TOOLBOX",
                    (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 0),
                    2
                )

                cv2.putText(
                    color_image,
                    "Confidence: {:.2f}".format(
                        best_confidence
                    ),
                    (20, 60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (0, 255, 0),
                    2
                )

                cv2.putText(
                    color_image,
                    "Distance: {:.2f} m".format(
                        smooth_depth
                    ),
                    (20, 90),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (0, 255, 255),
                    2
                )

                cv2.putText(
                    color_image,
                    "X: {:.2f} m".format(X),
                    (20, 120),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.60,
                    (255, 255, 255),
                    2
                )

                cv2.putText(
                    color_image,
                    "Y: {:.2f} m".format(Y),
                    (20, 150),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.60,
                    (255, 255, 255),
                    2
                )

                cv2.putText(
                    color_image,
                    "Z: {:.2f} m".format(Z),
                    (20, 180),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.60,
                    (255, 255, 255),
                    2
                )

                cv2.putText(
                    color_image,
                    goal_status,
                    (20, 220),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.75,
                    (0, 255, 0),
                    2
                )

                # Toolbox label
                cv2.putText(
                    color_image,
                    "TOOLBOX",
                    (x1, max(25, y1 - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2
                )

                # =========================
                # TERMINAL OUTPUT
                # =========================
                print(
                    "\rTOOLBOX | "
                    "Conf: {:.2f} | "
                    "Distance: {:.2f} m | "
                    "X: {:.2f} | "
                    "Y: {:.2f} | "
                    "Z: {:.2f} | "
                    "{}".format(
                        best_confidence,
                        smooth_depth,
                        X,
                        Y,
                        Z,
                        goal_status
                    ),
                    end=""
                )

            else:

                cv2.putText(
                    color_image,
                    "TOOLBOX DEPTH INVALID",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 0, 255),
                    2
                )

        # =========================
        # TOOLBOX NOT FOUND
        # =========================
        else:

            depth_history.clear()

            cv2.putText(
                color_image,
                "SEARCHING FOR TOOLBOX...",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2
            )

            cv2.putText(
                color_image,
                "GOAL: TOOLBOX",
                (20, 75),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 255),
                2
            )

        # =========================
        # DISPLAY CAMERA
        # =========================
        cv2.imshow(
            "Rover Toolbox Goal Detection",
            color_image
        )

        # =========================
        # EXIT
        # =========================
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

finally:

    pipeline.stop()
    cv2.destroyAllWindows()

    print("\n")
    print("====================================")
    print(" TOOLBOX DETECTION STOPPED")
    print("====================================")
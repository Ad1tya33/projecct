import cv2
import numpy as np
import socket
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import csv
import time
from datetime import datetime

# ============================================================
# CONFIGURATION
# ============================================================

CARMAKER_IP = "172.22.64.1"

CAMERA_PORT = 2210
DVA_PORT = 16660

FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# ============================================================
# ROI SETTINGS
# ============================================================

ROI_START = 0.45

# ============================================================
# SINGLE-LANE TRACKING
# ============================================================

DESIRED_LEFT_X = 120

# ============================================================
# PID SETTINGS
# ============================================================

KP = 0.80
KI = 0.0005
KD = 0.10

MAX_STEER = 1.00

STEER_SMOOTHING = 0.75

STEERING_ENABLE_DELAY = 0.5

# ============================================================
# METRICS
# ============================================================

# increased because tRoad is not true lane-center offset
LANE_HALF_WIDTH = 3.5

RMSE_THRESHOLD = 0.10
DNF_TIME = 5.0

outside_start = None
DNF = False

# ============================================================
# PID VARIABLES
# ============================================================

prev_error = 0.0
integral_error = 0.0
prev_steer = 0.0

# ============================================================
# LANE SMOOTHING
# ============================================================

prev_left_lane = DESIRED_LEFT_X

# ============================================================
# OUTPUT FILES
# ============================================================

timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

CSV_FILE = f"lka_log_{timestamp}.csv"
PLOT_FILE = f"lka_plots_{timestamp}.png"
VIDEO_FILE = f"lka_video_{timestamp}.avi"

# ============================================================
# LOGGING
# ============================================================

times = []
camera_error_history = []
steer_history = []
speed_history = []
troad_history = []

# ============================================================
# DVA HELPERS
# ============================================================

def dva_set(sock, quantity, value):

    try:

        sock.sendall(
            f"set {quantity} {value:.6f}\n".encode()
        )

        sock.recv(64)

    except:
        pass


def dva_get(sock, quantity):

    try:

        sock.sendall(
            f"get {quantity}\n".encode()
        )

        resp = sock.recv(64).decode().strip()

        return float(resp[1:].strip())

    except:
        return 0.0

# ============================================================
# RMSE
# ============================================================

def compute_rmse(values):

    penalised = np.maximum(
        0,
        np.abs(values) - RMSE_THRESHOLD
    )

    return np.sqrt(
        np.mean(penalised ** 2)
    )

# ============================================================
# IMAGE PROCESSING
# ============================================================

def process_frame(frame):

    global prev_left_lane

    debug = frame.copy()

    h, w = frame.shape[:2]

    roi = frame[int(h * ROI_START):, :]

    # ========================================================
    # PREPROCESSING
    # ========================================================

    gray = cv2.cvtColor(
        roi,
        cv2.COLOR_BGR2GRAY
    )

    blur = cv2.GaussianBlur(
        gray,
        (5, 5),
        0
    )

    edges = cv2.Canny(
        blur,
        50,
        150
    )

    # ========================================================
    # HOUGH LINES
    # ========================================================

    lines = cv2.HoughLinesP(
        edges,
        1,
        np.pi / 180,
        threshold=50,
        minLineLength=60,
        maxLineGap=40
    )

    left_candidates = []

    if lines is not None:

        for line in lines:

            x1, y1, x2, y2 = line[0]

            if x2 - x1 == 0:
                continue

            slope = (
                (y2 - y1) /
                (x2 - x1)
            )

            # left lane only
            if slope < -0.4 and slope > -3.0:

                left_candidates.extend([x1, x2])

                cv2.line(
                    roi,
                    (x1, y1),
                    (x2, y2),
                    (0, 255, 0),
                    3
                )

    # ========================================================
    # LEFT LANE ESTIMATION
    # ========================================================

    left_lane_x = prev_left_lane

    if len(left_candidates) > 0:

        raw_left = int(
            np.mean(left_candidates)
        )

        # temporal smoothing
        left_lane_x = int(
            0.80 * prev_left_lane +
            0.20 * raw_left
        )

        prev_left_lane = left_lane_x

    # ========================================================
    # ERROR COMPUTATION
    # ========================================================

    error_px = (
        DESIRED_LEFT_X - left_lane_x
    )

    normalized_error = (
        error_px / 100.0
    )

    normalized_error = np.clip(
        normalized_error,
        -1.5,
        1.5
    )

    # ========================================================
    # VISUALIZATION
    # ========================================================

    roi_y = int(h * ROI_START)

    cv2.line(
        debug,
        (DESIRED_LEFT_X, roi_y),
        (DESIRED_LEFT_X, h),
        (0, 0, 255),
        2
    )

    cv2.line(
        debug,
        (left_lane_x, roi_y),
        (left_lane_x, h),
        (255, 0, 0),
        3
    )

    cv2.putText(
        debug,
        f"Lane Error: {normalized_error:+.3f}",
        (30, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 255),
        2
    )

    return debug, edges, normalized_error

# ============================================================
# PID CONTROLLER
# ============================================================

def pid_control(error, speed):

    global prev_error
    global integral_error
    global prev_steer

    dt = 0.05

    if speed > 5:

        integral_error += (
            error * dt
        )

    else:

        integral_error = 0.0

    derivative = (
        error - prev_error
    ) / dt

    prev_error = error

    steer_raw = -(
        KP * error +
        KI * integral_error +
        KD * derivative
    )

    steer_raw = np.clip(
        steer_raw,
        -MAX_STEER,
        MAX_STEER
    )

    steer = (
        STEER_SMOOTHING * prev_steer +
        (1 - STEER_SMOOTHING) * steer_raw
    )

    prev_steer = steer

    return steer

# ============================================================
# CONNECT CAMERA
# ============================================================

print("\n====================================")
print(" SINGLE-LANE CAMERA LKA STARTED ")
print("====================================")

cam_sock = socket.socket(
    socket.AF_INET,
    socket.SOCK_STREAM
)

cam_sock.connect(
    (CARMAKER_IP, CAMERA_PORT)
)

print("RSDS Camera Connected")

# ============================================================
# CONNECT DVA
# ============================================================

DVA = socket.socket(
    socket.AF_INET,
    socket.SOCK_STREAM
)

DVA.connect(
    (CARMAKER_IP, DVA_PORT)
)

print("DVA Connected")

dva_set(
    DVA,
    "Driver.Lat.passive",
    1
)

print("Python Steering Active")
print("Press CTRL + C to stop")

# ============================================================
# VIDEO WRITER
# ============================================================

video_writer = None

buffer = b""

start_time = time.time()

# ============================================================
# MAIN LOOP
# ============================================================

try:

    while True:

        data = cam_sock.recv(65536)

        if not data:
            break

        buffer += data

        start = buffer.find(b"*RSDS")

        if start == -1:
            continue

        header_end = start + 64

        if len(buffer) < header_end:
            continue

        header = buffer[
            start:header_end
        ].decode(
            "latin-1",
            errors="ignore"
        )

        parts = header.split()

        try:

            w, h = map(
                int,
                parts[4].split('x')
            )

            img_size = int(parts[5])

        except:

            buffer = buffer[start + 1:]

            continue

        if len(buffer) < header_end + img_size:
            continue

        raw = buffer[
            header_end:
            header_end + img_size
        ]

        buffer = buffer[
            header_end + img_size:
        ]

        img = np.frombuffer(
            raw,
            dtype=np.uint8
        ).reshape((h, w, 3))

        frame = cv2.cvtColor(
            img,
            cv2.COLOR_RGB2BGR
        )

        frame = cv2.resize(
            frame,
            (FRAME_WIDTH, FRAME_HEIGHT)
        )

        if video_writer is None:

            fourcc = cv2.VideoWriter_fourcc(*'XVID')

            video_writer = cv2.VideoWriter(
                VIDEO_FILE,
                fourcc,
                20,
                (FRAME_WIDTH, FRAME_HEIGHT)
            )

        # ====================================================
        # PROCESS FRAME
        # ====================================================

        debug, edges, camera_error = process_frame(frame)

        # ====================================================
        # VEHICLE STATES
        # ====================================================

        speed = (
            dva_get(DVA, "Vhcl.v") * 3.6
        )

        troad = dva_get(
            DVA,
            "Vhcl.tRoad"
        )

        # ====================================================
        # PID CONTROL
        # ====================================================

        steer = pid_control(
            camera_error,
            speed
        )

        elapsed = (
            time.time() - start_time
        )

        # ====================================================
        # ENABLE STEERING
        # ====================================================

        if elapsed > STEERING_ENABLE_DELAY:

            dva_set(
                DVA,
                "VC.Steer.Ang",
                steer
            )

        else:

            dva_set(
                DVA,
                "VC.Steer.Ang",
                0.0
            )

        # ====================================================
        # DNF CHECK
        # ====================================================

        if abs(troad) > LANE_HALF_WIDTH:

            if outside_start is None:

                outside_start = time.time()

            elif (
                time.time() - outside_start
            ) > DNF_TIME:

                DNF = True

        else:

            outside_start = None

        # ====================================================
        # LOGGING
        # ====================================================

        times.append(elapsed)

        camera_error_history.append(
            camera_error
        )

        steer_history.append(
            steer
        )

        speed_history.append(
            speed
        )

        troad_history.append(
            troad
        )

        # ====================================================
        # DISPLAY
        # ====================================================

        cv2.putText(
            debug,
            f"tRoad: {troad:+.3f} m",
            (30, 80),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 0),
            2
        )

        cv2.putText(
            debug,
            f"Steer: {steer:+.4f}",
            (30, 120),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2
        )

        cv2.putText(
            debug,
            f"Speed: {speed:5.1f} km/h",
            (30, 160),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2
        )

        cv2.imshow(
            "Lane Detection",
            debug
        )

        cv2.imshow(
            "Edges",
            edges
        )

        video_writer.write(
            debug
        )

        print(
            f"t={elapsed:5.1f}s | "
            f"LaneErr={camera_error:+.3f} | "
            f"Steer={steer:+.4f} | "
            f"tRoad={troad:+.3f} | "
            f"Speed={speed:5.1f}km/h"
        )

        cv2.waitKey(1)

except KeyboardInterrupt:

    print("\n====================================")
    print(" CTRL+C — shutting down cleanly")
    print("====================================")

# ============================================================
# CLEANUP
# ============================================================

cv2.destroyAllWindows()

cam_sock.close()

DVA.close()

if video_writer is not None:
    video_writer.release()

# ============================================================
# SAVE CSV
# ============================================================

with open(CSV_FILE, 'w', newline='') as f:

    writer = csv.writer(f)

    writer.writerow([
        'time_s',
        'camera_error',
        'steer',
        'speed_kmh',
        'tRoad'
    ])

    for i in range(len(times)):

        writer.writerow([
            times[i],
            camera_error_history[i],
            steer_history[i],
            speed_history[i],
            troad_history[i]
        ])

# ============================================================
# METRICS
# ============================================================

TIME = np.array(times)

CAM_ERROR = np.array(
    camera_error_history
)

STEER = np.array(
    steer_history
)

SPEED = np.array(
    speed_history
)

TROAD = np.array(
    troad_history
)

RMSE = compute_rmse(
    TROAD
)

ILC = (
    np.sum(
        np.abs(TROAD) < LANE_HALF_WIDTH
    ) / len(TROAD)
) * 100

MAX_DEV = np.max(
    np.abs(TROAD)
)

# ============================================================
# FINAL RESULTS
# ============================================================

print("\n==================================================")
print("FINAL RESULTS")
print("==================================================")

print(f"RMSE          : {RMSE:.4f} m")
print(f"ILC           : {ILC:.2f} %")
print(f"Max Deviation : {MAX_DEV:.3f} m")
print(f"DNF           : {'YES' if DNF else 'NO'}")

print("==================================================")

# ============================================================
# PLOTS
# ============================================================

plt.style.use(
    'dark_background'
)

fig = plt.figure(
    figsize=(16, 10)
)

ax1 = plt.subplot(2, 2, 1)

ax1.plot(
    TIME,
    CAM_ERROR,
    linewidth=2
)

ax1.set_title(
    "Lane Error vs Time"
)

ax1.grid(alpha=0.3)

ax2 = plt.subplot(2, 2, 2)

ax2.plot(
    TIME,
    STEER,
    linewidth=2
)

ax2.set_title(
    "Steering vs Time"
)

ax2.grid(alpha=0.3)

ax3 = plt.subplot(2, 2, 3)

ax3.plot(
    TIME,
    SPEED,
    linewidth=2
)

ax3.set_title(
    "Speed vs Time"
)

ax3.grid(alpha=0.3)

ax4 = plt.subplot(2, 2, 4)

ax4.axis('off')

metrics = (
    f"RMSE          : {RMSE:.4f} m\n\n"
    f"ILC           : {ILC:.2f} %\n\n"
    f"Max Deviation : {MAX_DEV:.3f} m\n\n"
    f"DNF           : {'YES' if DNF else 'NO'}"
)

ax4.text(
    0.05,
    0.9,
    metrics,
    fontsize=18,
    verticalalignment='top'
)

plt.tight_layout()

plt.savefig(
    PLOT_FILE
)

print(f"\nPlots → {PLOT_FILE}")
print(f"Video → {VIDEO_FILE}")
print(f"CSV   → {CSV_FILE}")
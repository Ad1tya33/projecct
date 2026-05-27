# FINAL ENDURANCE CONTROLLER
# LKA + AEB
# NO CAMERA DISPLAY VERSION

import cv2
import numpy as np
import socket
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import csv
import time
from datetime import datetime
from CarMaker import CarMaker, Quantity

# ============================================================
# CONFIGURATION
# ============================================================

CARMAKER_IP = "172.22.64.1"
CARMAKER_PORT = 16660

CAMERA_PORT = 2210
DVA_PORT = 16660

FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# ============================================================
# ROI SETTINGS
# ============================================================

ROI_START = 0.45

# ============================================================
# LANE TARGET
# ============================================================

DESIRED_LEFT_X = 90

# ============================================================
# PID SETTINGS (LKA)
# ============================================================

KP_LKA = 0.65
KI_LKA = 0.0003
KD_LKA = 0.02

MAX_STEER = 0.05

STEER_SMOOTHING = 0.45

STEERING_ENABLE_DELAY = 0.5

# ============================================================
# SPEED CONTROLLER
# ============================================================

TARGET_SPEED = 30 / 3.6

KP_SPEED = 0.5
KI_SPEED = 0.08

speed_integral = 0.0

# ============================================================
# AEB SETTINGS
# ============================================================

LANE_THRESHOLD = 2.0

TARGET_STOP_DIST = 12.0

TTC_THRESHOLD = 3.0
CLOSING_THRESHOLD = -0.3

BRAKE_FORCE = 0.35

# ============================================================
# OUTPUT FILES
# ============================================================

timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

CSV_FILE = f"endurance_log_{timestamp}.csv"
PLOT_FILE = f"endurance_plots_{timestamp}.png"

# ============================================================
# LOGGING
# ============================================================

times = []

speed_history = []
troad_history = []
distance_history = []

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
# AEB HOLD STATE
# ============================================================

aeb_hold = False

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
# IMAGE PROCESSING
# ============================================================

def process_frame(frame):

    global prev_left_lane

    h, w = frame.shape[:2]

    roi = frame[int(h * ROI_START):, :]

    gray = cv2.cvtColor(
        roi,
        cv2.COLOR_BGR2GRAY
    )

    blur = cv2.GaussianBlur(
        gray,
        (9, 9),
        0
    )

    edges = cv2.Canny(
        blur,
        80,
        200
    )

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

            if slope < -0.4 and slope > -3.0:

                left_candidates.extend(
                    [x1, x2]
                )

    if len(left_candidates) > 0:

        left_lane_x = int(
            np.mean(left_candidates)
        )

        left_lane_x = int(
            0.93 * prev_left_lane +
            0.07 * left_lane_x
        )

        prev_left_lane = left_lane_x

    else:

        left_lane_x = prev_left_lane

    error = (
        DESIRED_LEFT_X -
        left_lane_x
    ) / 100.0

    # DEADZONE

    if abs(error) < 0.03:

        error = 0.0

    return error

# ============================================================
# PID CONTROLLER
# ============================================================

def pid_control(error):

    global prev_error
    global integral_error
    global prev_steer

    integral_error += error

    derivative = (
        error - prev_error
    )

    raw = (
        KP_LKA * error +
        KI_LKA * integral_error +
        KD_LKA * derivative
    )

    target_steer = (
        STEER_SMOOTHING * prev_steer +
        (1 - STEER_SMOOTHING) * raw
    )

    max_delta = 0.03

    delta = target_steer - prev_steer

    delta = max(
        -max_delta,
        min(max_delta, delta)
    )

    steer = prev_steer + delta

    steer = max(
        -MAX_STEER,
        min(MAX_STEER, steer)
    )

    prev_error = error
    prev_steer = steer

    return steer

# ============================================================
# CONNECT CAMERA
# ============================================================

camera_socket = socket.socket(
    socket.AF_INET,
    socket.SOCK_STREAM
)

camera_socket.connect(
    (CARMAKER_IP, CAMERA_PORT)
)

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

# ============================================================
# CONNECT CARMAKER
# ============================================================

cm = CarMaker(
    CARMAKER_IP,
    CARMAKER_PORT
)

cm.connect()

# ============================================================
# RADAR SUBSCRIPTIONS
# ============================================================

speed_q = Quantity(
    "Car.v",
    Quantity.FLOAT
)

dist_q = Quantity(
    "Sensor.Radar.Vhcl.RAD00.Obj0.Dist",
    Quantity.FLOAT
)

vrel_q = Quantity(
    "Sensor.Radar.Vhcl.RAD00.Obj0.Vrel",
    Quantity.FLOAT
)

y_q = Quantity(
    "Sensor.Radar.Vhcl.RAD00.Obj0.DistY",
    Quantity.FLOAT
)

for q in (
    speed_q,
    dist_q,
    vrel_q,
    y_q
):
    cm.subscribe(q)

# ============================================================
# CONTROL QUANTITIES
# ============================================================

gas_q = Quantity(
    "VC.Gas",
    Quantity.FLOAT
)

brake_q = Quantity(
    "VC.Brake",
    Quantity.FLOAT
)

long_q = Quantity(
    "Driver.Long.passive",
    Quantity.INT
)

cm.DVA_write(long_q, 1)

# ============================================================
# CSV LOGGER
# ============================================================

csv_file = open(
    CSV_FILE,
    'w',
    newline=''
)

csv_writer = csv.writer(csv_file)

csv_writer.writerow([
    'Time',
    'Speed',
    'tRoad',
    'Distance',
    'Steer'
])

# ============================================================
# MAIN LOOP
# ============================================================

start_time = time.time()

prev_control_time = time.time()

try:

    while True:

        now_abs = time.time()

        elapsed = (
            now_abs -
            start_time
        )

        # ====================================================
        # RECEIVE CAMERA FRAME
        # ====================================================

        data = b''

        expected = (
            FRAME_WIDTH *
            FRAME_HEIGHT *
            3
        )

        while len(data) < expected:

            packet = camera_socket.recv(
                expected - len(data)
            )

            if not packet:
                break

            data += packet

        if len(data) != expected:
            continue

        frame = np.frombuffer(
            data,
            dtype=np.uint8
        ).reshape(
            (
                FRAME_HEIGHT,
                FRAME_WIDTH,
                3
            )
        )

        # ====================================================
        # LKA
        # ====================================================

        error = process_frame(frame)

        steer = pid_control(error)

        # ====================================================
        # APPLY STEERING
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
        # RADAR UPDATE
        # ====================================================

        cm.read()

        speed = speed_q.data or 0.0

        distance = dist_q.data or 0.0

        vrel = vrel_q.data or 0.0

        y_dist = y_q.data or 0.0

        # ====================================================
        # AEB LOGIC
        # ====================================================

        valid_target = (
            distance > 0 and
            abs(y_dist) < LANE_THRESHOLD
        )

        ttc = 999

        if vrel < CLOSING_THRESHOLD:

            ttc = (
                distance /
                abs(vrel)
            )

        if valid_target:

            if (
                distance < TARGET_STOP_DIST or
                ttc < TTC_THRESHOLD
            ):

                aeb_hold = True

            elif distance > 18.0:

                aeb_hold = False

        else:

            aeb_hold = False

        aeb_active = aeb_hold

        # ====================================================
        # LONGITUDINAL CONTROL
        # ====================================================

        now_control = time.time()

        dt = now_control - prev_control_time

        prev_control_time = now_control

        if dt <= 0:
            dt = 0.01

        if aeb_active:

            gas = 0.0
            brake = BRAKE_FORCE

        else:

            speed_error = TARGET_SPEED - speed

            speed_integral += speed_error * dt

            gas = (
                KP_SPEED * speed_error +
                KI_SPEED * speed_integral
            )

            gas = max(
                0.0,
                min(0.08, gas)
            )

            brake = 0.0

        cm.DVA_write(
            gas_q,
            gas
        )

        cm.DVA_write(
            brake_q,
            brake
        )

        # ====================================================
        # METRICS
        # ====================================================

        troad = dva_get(
            DVA,
            "Vhcl.tRoad"
        )

        times.append(elapsed)

        speed_history.append(
            speed * 3.6
        )

        troad_history.append(
            troad
        )

        distance_history.append(
            distance
        )

        csv_writer.writerow([
            elapsed,
            speed * 3.6,
            troad,
            distance,
            steer
        ])

        print(
            f"t={elapsed:5.1f}s | "
            f"Spd={speed*3.6:5.1f}km/h | "
            f"Dist={distance:5.2f}m | "
            f"Steer={steer:+.3f} | "
            f"AEB={'ON' if aeb_active else 'OFF'}"
        )

except KeyboardInterrupt:
    pass

# ============================================================
# CLEANUP
# ============================================================

csv_file.close()

# ============================================================
# PLOTS
# ============================================================

plt.figure(figsize=(14, 10))

plt.subplot(3, 1, 1)
plt.plot(times, speed_history)
plt.title("Speed vs Time")
plt.xlabel("Time (s)")
plt.ylabel("Speed (km/h)")
plt.grid(True)

plt.subplot(3, 1, 2)
plt.plot(times, troad_history)
plt.title("tRoad vs Time")
plt.xlabel("Time (s)")
plt.ylabel("tRoad (m)")
plt.grid(True)

plt.subplot(3, 1, 3)
plt.plot(times, distance_history)
plt.title("Distance vs Time")
plt.xlabel("Time (s)")
plt.ylabel("Distance (m)")
plt.grid(True)

plt.tight_layout()

plt.savefig(PLOT_FILE)

print("==================================================")
print("ENDURANCE SIMULATION COMPLETE")
print("==================================================")
print(f"CSV   : {CSV_FILE}")
print(f"Plots : {PLOT_FILE}")
print("==================================================")
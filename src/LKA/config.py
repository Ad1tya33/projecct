"""
config.py
Shared configuration and calibration parameters for the LKA pipeline.

ROLE IN ARCHITECTURE:
This file acts as the central configuration hub for the Lane Keeping Assist (LKA) system. 
It defines all the tunable parameters, thresholds, and mappings used by the other modules.
By centralizing these parameters, the systme behaviour can easily be tweaked (e.g., 
adjusting target speed, PID gains, camera settings) without digging into the control logic.

KEY SECTIONS:
- Camera Settings: TCP connection parameters to receive frames from MovieNX.
- Control & Speed: Loop frequency and longitudinal target speed.
- LKA Physics & PID: Pure pursuit lookahead, PID gains, and physical vehicle properties.
- CarMaker DVA Quantities: Exact names for CarMaker Data Variable Access (DVA) to read 
  sensors and write control commands via the APO interface.
"""

# ── CAMERA SETTINGS (TCP Client) ──────────────────────────────────────────
# Standard MovieNX Camera RSI export settings
CAMERA_HOST = "localhost"
CAMERA_PORT = 2210
CAMERA_TIMEOUT = 5.0
CAMERA_RECONNECT_DELAY = 1.0
IMG_W = 640
IMG_H = 480
IMG_CHANNELS = 3

# ── CONTROL LOOP EXECUTION ────────────────────────────────────────────────
# Target execution rate for the main lateral control loop.
# 30 Hz matches a typical vision processing pipeline and CarMaker frame rates.
CONTROL_HZ = 30.0

# ── SPEED CONTROL ─────────────────────────────────────────────────────────
TARGET_SPEED_KMH = 30.0          # Constant cruise speed [km/h]
GAS_QTY   = "DM.Gas"            # Throttle DVA quantity [0.0 – 1.0]
BRAKE_QTY = "DM.Brake"          # Brake DVA quantity   [0.0 – 1.0]

# ── LKA PHYSICS & KINEMATICS ──────────────────────────────────────────────
# Vehicle: Demo_IPG_CompanyCar (RWD, 1391 kg, Izz=1894 kg·m²)
# Front axle: 3.517m, Rear axle: 0.895m from ref point
WHEELBASE_M = 2.622              # Actual wheelbase (3.517 - 0.895) [m]
LOOKAHEAD_M = 5.0                # Pure pursuit lookahead — shorter at 30 km/h for responsive steering [m]
PIXEL_TO_METER = 0.006           # Calibration: right solid at x≈615 when centred at 70% strip → half-width=295px → 1.75/295≈0.006

# ── PID TRIM GAINS (small corrections on top of Pure Pursuit) ──────────────
# All gains produce ROAD-WHEEL angle [rad]. Steering ratio applied at output.
# At 30 km/h, 0.6m error should give ~0.06 rad (3.4°) road-wheel correction.
KP = 0.10                       # Proportional gain [rad_roadwheel/m]
KI = 0.005                      # Integral gain — corrects steady-state offset
KD = 0.08                       # Derivative gain — damping (prevents overshoot)
INTEGRAL_CLIP = 0.10            # Anti-windup clip [rad·s]

# ── FEEDFORWARD & DAMPING ──────────────────────────────────────────────────
# Curvature FF: steering_wheel_angle = L * curvature * steering_ratio
# At curvature=0.01 (R=100m): 2.622 * 0.01 * 16 = 0.42 rad (24° steering wheel)
CURV_FF_GAIN = 16.0              # steering_ratio: road_wheel_to_steering_wheel
YAW_DAMP_GAIN = 0.02             # Yaw rate damping [rad_roadwheel/(rad/s)]

# ── STEERING CONSTRAINTS (road-wheel angle) ──────────────────────────────
# At 30 km/h: safe lateral accel ~0.3g → min radius ~20m → max road-wheel ~0.13 rad
MAX_STEER_RAD = 0.20             # Max road-wheel angle [rad] (~11.5 deg → ~180° steer wheel)
RATE_LIMIT_RAD = 0.03            # Max road-wheel change per cycle [rad] (~1.7 deg/frame)

# ── PERFORMANCE SCORING ──────────────────────────────────────────────────
RMSE_WINDOW = 100                # Samples for rolling RMSE calculation
LANE_WIDTH_M = 3.5               # Lane width for ILC / DNF calculations

# ── DNF (DID NOT FINISH) WATCHDOG ──────────────────────────────────────────
DNF_THRESHOLD = 1.5              # Lateral error threshold (m)
DNF_TIME_S = 2.0                 # Time out-of-lane triggers DNF (seconds)

# ── CARMAKER SENSOR NAMES ───────────────────────────────────────────────────
# These are DVA (Data Variable Access) quantity names in CarMaker
CAMERA_RSI_SENSOR = "CARS00"
GROUND_TRUTH_SENSOR = "GT00"
ROAD_SENSOR = "RD00"
INERTIAL_SENSOR = "IN_Param"
SLIP_ANGLE_SENSOR = "SL_Param"

# ── DVA QUANTITY NAMES (From Vehicle Data Set) ───────────────────────────
# Camera frame (not used directly in Python, handled by TCP)
CAM_QUANTITY = "Camera.Frame"
VHCL_SPEED_QTY = "Vhcl.v"        # Vehicle velocity [km/h]

# Road/Lane information — Road Sensor "RD00" mounted as "VehSensor_1"
# Pattern: Sensor.Road.Vhcl.<MountingName>.<Route|Path>.<field>
LINE_LAT_DEV = "Sensor.Road.Vhcl.VehSensor_1.Route.Deviation.Dist"  # Lateral deviation [m]
ROAD_CURVATURE = "Sensor.Road.Vhcl.VehSensor_1.Route.CurveXY"      # Curvature [1/m]

# Alternative DVA quantity names to probe (in priority order)
# The correct name depends on sensor mount name and CarMaker version
LAT_DEV_CANDIDATES = [
    "Car.Road.Route.DevDist",                                  # Global: always available with route
    "Car.Road.Route.LaneDevDist",                              # Lane-level deviation
    "Sensor.Road.Vhcl.VehSensor_1.Route.Deviation.Dist",      # Sensor-based
    "Sensor.Road.Vhcl.VehSensor_1.rr.tx",                     # Road-relative lateral pos
    "Car.Road.Path.DevDist",                                   # Path deviation
    "Sensor.Road.Vhcl.RoadSensor.Route.Deviation.Dist",
    "Sensor.Road.Vhcl.FR1.Route.Deviation.Dist",
]
CURVATURE_CANDIDATES = [
    "Car.Road.Route.CurveXY",                                  # Global: always available
    "Car.Road.CurveXY",                                        # Simple global curvature
    "Sensor.Road.Vhcl.VehSensor_1.Route.CurveXY",             # Sensor-based
    "Sensor.Road.Vhcl.VehSensor_1.rr.kappa",                  # Road-relative curvature
    "Sensor.Road.Vhcl.RoadSensor.Route.CurveXY",
    "Sensor.Road.Vhcl.FR1.Route.CurveXY",
]

# Inertial measurements
INERTIAL_YAW_RATE = "Car.YawRate"                     # Yaw rate [rad/s]
INERTIAL_SLIP_ANGLE = "Car.SideSlipAngle"             # Slip angle [rad]
INERTIAL_ACCEL_LAT = "Car.ay"                         # Lateral accel [m/s^2]

# Steering control outputs
STEER_SOURCE_QTY = "DM.Steer.Source"                  # 0=Driver, 1=External
STEER_ANGLE_QTY = "DM.Steer.Ang"                      # Command angle [rad]
STEER_ANGLE_DVA_QTY = "DM.Steer.Ang.DVA"              # DVA override enable
STEER_MODE_QTY = "DM.Steer.Mode"                      # 0=Manual, 1=Auto

# Logging/monitoring
RMSE_PUBLISH_QTY = "LKA.RMSE"                         # Published RMSE
SYNC_MODE = 1                                         # Enable sync with CarMaker

# ── TERMINAL TELEMETRY ─────────────────────────────────────────────────────
PRINT_TELEMETRY = True                                # Print live LKA telemetry in terminal
PRINT_EVERY_N_TICKS = 10                              # Print every N control ticks
"""
LKA_Main.py
Entry point and Main Control Loop for the Lane Keeping Assist (LKA) system.

ROLE IN ARCHITECTURE:
This is the "Brain" and "Actuation" hub of the LKA system. It connects the Perception 
(sensor_fusion.py) with the simulation environment (CarMaker) and calculates the necessary 
steering and speed commands to keep the car in the lane.

KEY RESPONSIBILITIES:
1. CarMaker Connection: Establishes an async APO connection to read/write DVA quantities.
2. Sensor Polling: Fetches camera frames (via TCP) and telemetry (speed, yaw rate) from CarMaker.
3. Control Logic: Combines Pure Pursuit (geometric path following) with a PID controller 
   (to trim steady-state error) to compute the target steering wheel angle.
4. Actuation: Writes the calculated steering, gas, and brake commands back to CarMaker.
5. Telemetry & Logging: Tracks system performance (RMSE) and logs data for debugging.
"""

import sys
import os
import time
import math
import asyncio
import numpy as np

# --- FORCE THE REAL CARMAKER API PATH ---
_carMakerApiPaths = [
    r"C:\CarMakerOffice-win-15.0\IPGCarMakerPythonApiAddOn-15.0-win64\Python",
    r"C:\IPG\carmaker\win64-15.0\Python",
    r"C:\Program Files\IPG\CarMaker-15.0\python"
]
for _path in _carMakerApiPaths:
    if os.path.isdir(_path) and _path not in sys.path:
        sys.path.insert(0, _path)

_parent = os.path.dirname(r"C:\IPG\carmaker\win64-15.0\Python")
if _parent not in sys.path:
    sys.path.insert(0, _parent)


# --- IMPORT CARMAKER API ---
try:
    import cmapi
    from cmapi import SimControlInteractive, DVADuration, ApoServer
    print("[API] CONNECTED TO CARMAKER API")
except ImportError as e:
    print(f"[ERROR] cmapi not available: {e}")
    sys.exit(1)

import config as cfg
from sensor_fusion import SensorFusion
from lka_utils import RMSETracker, DNFWatchdog, DataLogger
from camera_tcp_client import CameraClient

# ── PID STATE (SINGLE, UNIFIED CONTROLLER) ────────────────────────────────
_integral = 0.0
_prev_error = 0.0
_prev_steer = 0.0

def pid_step(error: float, dt: float) -> float:
    """
    PID trim controller. Provides small corrections on top of pure pursuit.
    
    Args:
        error: Lateral error [m]
        dt: Time step [s]
        
    Returns:
        Steering wheel angle correction [rad]
    """
    global _integral, _prev_error
    
    # Accumulate integral with anti-windup
    _integral += error * dt
    _integral = np.clip(_integral, -cfg.INTEGRAL_CLIP, cfg.INTEGRAL_CLIP)
    
    # Calculate derivative
    derivative = (error - _prev_error) / dt if dt > 1e-6 else 0.0
    _prev_error = error
    
    # PID output
    pid_output = cfg.KP * error + cfg.KI * _integral + cfg.KD * derivative
    
    return pid_output

def pure_pursuit_angle(lookahead_pt: tuple, img_h: int) -> float:
    """
    Calculate steering wheel angle from pure pursuit geometry.
    
    Uses LOOKAHEAD_M for forward distance (perspective-correct),
    and PIXEL_TO_METER only for lateral offset.
    
    Args:
        lookahead_pt: (x, y) pixel coordinates from vision
        img_h: Image height [pixels]
        
    Returns:
        Steering wheel angle [rad]
    """
    lx, ly = lookahead_pt
    dx_px = lx - cfg.IMG_W / 2.0
    
    # Lateral offset in meters (PIXEL_TO_METER is calibrated for x-direction)
    dx_m = dx_px * cfg.PIXEL_TO_METER
    
    # Forward distance: use calibrated lookahead, NOT pixel conversion
    # In a perspective camera, y-pixels don't map linearly to meters
    dy_m = cfg.LOOKAHEAD_M
    
    # Lookahead distance
    ld = math.hypot(dx_m, dy_m)
    if ld < 0.1:
        return 0.0
    
    # Pure pursuit: heading error to lookahead point
    alpha = math.atan2(dx_m, dy_m)
    
    # Bicycle model: road wheel angle
    steer_rad = math.atan2(2.0 * cfg.WHEELBASE_M * math.sin(alpha), ld)
    
    # Return road-wheel angle; steering ratio applied at final output stage
    return steer_rad

def rate_limit(new_val: float, prev_val: float) -> float:
    """
    Rate limit steering angle change.
    
    Args:
        new_val: New steering command [rad]
        prev_val: Previous steering command [rad]
        
    Returns:
        Rate-limited steering command [rad]
    """
    delta = np.clip(new_val - prev_val, -cfg.RATE_LIMIT_RAD, cfg.RATE_LIMIT_RAD)
    return prev_val + delta

# ── MAIN CONTROL LOOP ──────────────────────────────────────────────────────

async def run():
    """
    Main 30 Hz control loop (async).
    Connects to running CarMaker GUI via APO for DVA access.
    """
    global _prev_steer

    # 1. Load project and connect to the already-running CarMaker GUI via APO
    sim_control = None
    simio = None
    try:
        import pathlib
        cmapi.Project.load(pathlib.Path(r"C:\IPG2026\CM_Projects\CM_15"))

        # Check that a CarMaker server is running on localhost
        servers = cmapi.query_aposerverinfos("localhost")
        if not servers:
            print("[ERROR] No running CarMaker instance found on localhost.")
            print("[ERROR] Please start CarMaker GUI first, then re-run this script.")
            return
        print(f"[API] Found {len(servers)} CarMaker server(s) on localhost")

        # Connect using empty ApoServerInfo (matches any server)
        master = ApoServer()
        master.set_sinfo(cmapi.ApoServerInfo())
        master.set_host("localhost")

        sim_control = await SimControlInteractive.create_with_master(master)
        await sim_control.connect()
        simio = sim_control.get_simio()
        print("[API] Connected to CarMaker via APO. DVA access ready.")
    except Exception as e:
        print(f"[ERROR] Failed to connect to CarMaker: {e}")
        return

    print("[API] Please start the simulation from CarMaker GUI if not already running.")

    # 3. Configure Control Authority via DVA writes
    try:
        simio.dva_write_absolute_value(cfg.STEER_SOURCE_QTY, 1, DVADuration.permanent)
        print("[CONTROL] Steering Source set to External (1).")
    except Exception as e:
        print(f"[ERROR] Failed to set steering source: {e}")

    try:
        simio.dva_write_absolute_value(cfg.STEER_ANGLE_DVA_QTY, 1, DVADuration.permanent)
        print("[CONTROL] DVA override enabled (1).")
    except Exception as e:
        print(f"[ERROR] Failed to enable DVA: {e}")

    try:
        simio.dva_write_absolute_value(cfg.STEER_MODE_QTY, 0, DVADuration.permanent)
        print("[CONTROL] Steering Mode set to Manual (0).")
    except Exception as e:
        print(f"[ERROR] Failed to set steering mode: {e}")

    print(f"[CONTROL] Speed controller armed. Target: {cfg.TARGET_SPEED_KMH:.0f} km/h (cycle-level writes in main loop)")

    print("\n" + "="*60)
    print("[LKA] CONTROL AUTHORITY ESTABLISHED")
    print("="*60 + "\n")

    # DVA quantity probing is deferred until the simulation is running
    # (quantities return 0 while idle). We'll probe on tick 5.
    active_lat_dev_qty = cfg.LINE_LAT_DEV  # default until probed
    active_curvature_qty = cfg.ROAD_CURVATURE
    dva_probed = False

    # 4. Setup TCP Camera Client
    cam_client = CameraClient(
        host=cfg.CAMERA_HOST,
        port=cfg.CAMERA_PORT,
        timeout=cfg.CAMERA_TIMEOUT
    )
    print("[CAMERA] Initializing TCP camera client...")
    await asyncio.sleep(1)  # Give C++ server time to open port
    if not cam_client.connect():
        print("[WARNING] Camera connection failed. Will retry during run.")

    # 5. Initialize controllers and logging
    fusion = SensorFusion()
    rmse_tk = RMSETracker()
    dnf_wd = DNFWatchdog()
    logger = DataLogger()

    t_prev = time.time()
    tick = 0

    try:
        while True:
            try:
                # ── RE-ASSERT DVA EVERY FRAME ──────────────────────────────────
                simio.dva_write_absolute_value(cfg.STEER_ANGLE_DVA_QTY, 1, DVADuration.permanent)

                # ── 1. READ SENSORS ────────────────────────────────────────────
                frame = cam_client.get_frame()
                
                # Read vehicle speed for telemetry
                vhcl_speed = 0.0
                try:
                    result = await simio.dva_read_async(cfg.VHCL_SPEED_QTY)
                    vhcl_speed_raw = result[0] if isinstance(result, tuple) else result
                    vhcl_speed = vhcl_speed_raw * 3.6  # Convert m/s → km/h
                except:
                    pass

                # ── DEFERRED DVA PROBING (once, after sim is running) ──────
                if not dva_probed and tick == 5 and vhcl_speed > 0.1:
                    dva_probed = True
                    print("[DVA] Probing lateral deviation quantities (sim running)...")
                    for qty in cfg.LAT_DEV_CANDIDATES:
                        try:
                            result = await simio.dva_read_async(qty)
                            val = result[0] if isinstance(result, tuple) else result
                            print(f"  {qty} = {val}")
                            if val != 0.0 and active_lat_dev_qty == cfg.LINE_LAT_DEV:
                                active_lat_dev_qty = qty
                        except Exception as e:
                            print(f"  {qty} = ERROR ({e})")
                    print(f"[DVA] Selected lat_dev: {active_lat_dev_qty}")
                    
                    print("[DVA] Probing curvature quantities...")
                    for qty in cfg.CURVATURE_CANDIDATES:
                        try:
                            result = await simio.dva_read_async(qty)
                            val = result[0] if isinstance(result, tuple) else result
                            print(f"  {qty} = {val}")
                            # Curvature can be 0 on straight road — accept first non-error
                            if active_curvature_qty == cfg.ROAD_CURVATURE:
                                active_curvature_qty = qty
                        except Exception as e:
                            print(f"  {qty} = ERROR ({e})")
                    print(f"[DVA] Selected curvature: {active_curvature_qty}")

                # Read Ground Truth Lateral Deviation (DVA)
                # Uses the probed quantity name that works in this session
                lat_dev = 0.0
                yaw_rate = 0.0
                curvature = 0.0
                try:
                    result = await simio.dva_read_async(active_lat_dev_qty)
                    lat_dev = result[0] if isinstance(result, tuple) else result
                except:
                    pass
                try:
                    result = await simio.dva_read_async(cfg.INERTIAL_YAW_RATE)
                    yaw_rate = result[0] if isinstance(result, tuple) else result
                except:
                    pass
                try:
                    result = await simio.dva_read_async(active_curvature_qty)
                    curvature = result[0] if isinstance(result, tuple) else result
                except:
                    pass

                # Periodic Debug Print
                if tick % 15 == 0:
                    print(f"[TICK {tick}] Frame: {frame is not None} | v={vhcl_speed:.1f} | lat_dev={lat_dev:+.3f}m | curv={curvature:.5f}")

                # --- Frame wait logic ---
                if frame is None:
                    if not cam_client.connected:
                        if tick % 30 == 0:
                            print("[CAMERA] Disconnected. Attempting reconnect...")
                        cam_client.reconnect()

                    if tick % 30 == 0:
                        print("[CAMERA] Waiting for valid frame from Movie NX...")
                    
                    # Hold last steering command instead of writing nothing
                    try:
                        simio.dva_write_absolute_value(cfg.STEER_ANGLE_QTY, _prev_steer * 16.0, DVADuration.cycle)
                    except:
                        pass
                    await asyncio.sleep(0.005)
                    tick += 1
                    continue

            except Exception as e:
                print(f"[ERROR] Exception in sensor reading: {e}")
                await asyncio.sleep(0.01)
                tick += 1
                continue

            # ── 2. SENSOR FUSION (VISION + DEAD-RECKONING) ─────────────────
            lateral_error = 0.0
            lane_detected = False
            lookahead_pt = (cfg.IMG_W // 2, cfg.IMG_H // 2)
            try:
                fusion.set_telemetry(steer_cmd=_prev_steer, speed_kmh=vhcl_speed)
                lateral_error, lane_detected, lookahead_pt = fusion.update(frame, yaw_rate=yaw_rate)
                
                # DVA lat_dev is logged for evaluation only (Rule R1: no ground-truth inputs)
                if not lane_detected and tick % 30 == 0:
                    print(f"[VISION LOST] dead-reckoning err={lateral_error:+.3f}m | GT lat_dev={lat_dev:+.3f}m (log only)")
            except Exception as e:
                print(f"[ERROR] Sensor fusion failed: {e}")
                await asyncio.sleep(1.0 / cfg.CONTROL_HZ)
                tick += 1
                continue

            t_now = time.time()
            dt = max(t_now - t_prev, 0.001)
            t_prev = t_now

            # ── 3. UNIFIED PID CONTROL LOOP ────────────────────────────────
            # All computations in ROAD-WHEEL angle [rad], converted to
            # steering-wheel angle at the final write stage.
            try:
                pid_cmd = pid_step(lateral_error, dt)
                pp_cmd = pure_pursuit_angle(lookahead_pt, cfg.IMG_H)
                yaw_damp = yaw_rate * cfg.YAW_DAMP_GAIN  # Oppose current yaw rotation (IMU = allowed)
                raw_steer = pp_cmd + pid_cmd + yaw_damp
            except Exception as e:
                print(f"[ERROR] Control calculation failed: {e}")
                raw_steer = 0.0

            # ── 4. RATE LIMITING & SATURATION ─────────────────────────────
            try:
                steer_cmd = rate_limit(raw_steer, _prev_steer)
                steer_cmd = float(np.clip(steer_cmd, -cfg.MAX_STEER_RAD, cfg.MAX_STEER_RAD))
                _prev_steer = steer_cmd
            except Exception as e:
                print(f"[ERROR] Steering command calculation failed: {e}")
                steer_cmd = 0.0

            # ── 5. WRITE STEERING TO CARMAKER ──────────────────────────────
            # DM.Steer.Ang: steering WHEEL angle [rad]
            # Empirically: positive = RIGHT turn in this setup.
            # Our control law: positive error → steer right → positive DM.Steer.Ang
            STEERING_RATIO = 16.0
            steer_wheel_rad = steer_cmd * STEERING_RATIO
            try:
                simio.dva_write_absolute_value(cfg.STEER_ANGLE_QTY, steer_wheel_rad, DVADuration.cycle)
            except Exception as e:
                print(f"[ERROR] Failed to write steering command: {e}")

            # ── 5b. SPEED CONTROL (proportional gas/brake) ─────────────────
            try:
                speed_err = cfg.TARGET_SPEED_KMH - vhcl_speed
                if speed_err > 0:
                    gas_cmd = float(np.clip(speed_err / 10.0, 0.0, 1.0))
                    simio.dva_write_absolute_value(cfg.GAS_QTY,   gas_cmd, DVADuration.cycle)
                    simio.dva_write_absolute_value(cfg.BRAKE_QTY, 0.0,     DVADuration.cycle)
                else:
                    brake_cmd = float(np.clip(-speed_err / 5.0, 0.0, 1.0))
                    simio.dva_write_absolute_value(cfg.GAS_QTY,   0.0,       DVADuration.cycle)
                    simio.dva_write_absolute_value(cfg.BRAKE_QTY, brake_cmd, DVADuration.cycle)
            except Exception as e:
                pass  # Speed control failure is non-fatal

            # ── 6. BOOKKEEPING ────────────────────────────────────────────
            try:
                rmse_tk.update(lateral_error)
                dnf_wd.update(lateral_error)
                simio.dva_write_absolute_value(cfg.RMSE_PUBLISH_QTY, rmse_tk.rmse, DVADuration.cycle)
            except Exception as e:
                print(f"[ERROR] Bookkeeping failed: {e}")

            # ── 7. LOGGING ────────────────────────────────────────────────
            try:
                logger.log(
                    t_now, lateral_error, steer_cmd,
                    rmse_tk.rmse, rmse_tk.ilc,
                    lane_detected
                )

                tick += 1
                if tick % 30 == 0:
                    print(f"[LKA t={t_now:.2f}s] "
                          f"err={lateral_error:+.3f}m | "
                          f"steer={steer_cmd:+.1f}\u00b0 | "
                          f"v={vhcl_speed:.1f}km/h | "
                          f"RMSE={rmse_tk.rmse:.4f}m | "
                          f"ILC={rmse_tk.ilc:.2%}")

                if cfg.PRINT_TELEMETRY and (tick % max(1, cfg.PRINT_EVERY_N_TICKS) == 0):
                    print(
                        f"[TELEMETRY] "
                        f"err={lateral_error:+.3f}m | "
                        f"steer={steer_cmd:+.2f}deg | "
                        f"v={vhcl_speed:.1f}km/h"
                    )
            except Exception as e:
                print(f"[ERROR] Logging failed: {e}")

            await asyncio.sleep(1.0 / cfg.CONTROL_HZ)

    except KeyboardInterrupt:
        print("\n[LKA] Stopped by User.")
    finally:
        cam_client.close()
        logger.close()
        try:
            simio.dva_write_absolute_value(cfg.STEER_SOURCE_QTY, 0, DVADuration.permanent)
        except:
            pass
        try:
            await sim_control.disconnect()
        except:
            pass
        print(f"\n[LKA] Simulation Finished. Final RMSE: {rmse_tk.rmse:.4f}m")


if __name__ == "__main__":
    cmapi.Task.run_main_task(run())

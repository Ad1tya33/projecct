import os
import csv
import time
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.lines import Line2D
from matplotlib.collections import LineCollection
from scipy.signal import medfilt
from CarMaker import CarMaker, Quantity

# -----------------------------------------------
# TEAM CONFIG
# -----------------------------------------------
TEAM_ID      = "264022"
TEAM_NAME    = "The Elecruisers"
SENSOR_OFFSET = 6.0        # m — radar face to target rear bumper (calibrate once)

def _fname(tag, ext):
    return f"{TEAM_ID}_{TEAM_NAME}_{tag}.{ext}"

# -----------------------------------------------
# CONNECTION
# -----------------------------------------------
IP   = "172.22.64.1"
PORT = 16660

cm = CarMaker(IP, PORT)
cm.connect()

# -----------------------------------------------
# SUBSCRIBE — SENSORS
# -----------------------------------------------
speed_q = Quantity("Car.v",                               Quantity.FLOAT)
dist_q  = Quantity("Sensor.Radar.Vhcl.RAD00.Obj0.Dist",  Quantity.FLOAT)
vrel_q  = Quantity("Sensor.Radar.Vhcl.RAD00.Obj0.Vrel",  Quantity.FLOAT)
y_q     = Quantity("Sensor.Radar.Vhcl.RAD00.Obj0.DistY", Quantity.FLOAT)
pos_x_q = Quantity("Car.tx",                              Quantity.FLOAT)
pos_y_q = Quantity("Car.ty",                              Quantity.FLOAT)

for q in (speed_q, dist_q, vrel_q, y_q, pos_x_q, pos_y_q):
    cm.subscribe(q)

# -----------------------------------------------
# SUBSCRIBE — CONTROL
# -----------------------------------------------
gas_q   = Quantity("VC.Gas",              Quantity.FLOAT)
brake_q = Quantity("VC.Brake",            Quantity.FLOAT)
long_q  = Quantity("Driver.Long.passive", Quantity.INT)
lat_q   = Quantity("Driver.Lat.passive",  Quantity.INT)

# -----------------------------------------------
# SPEED CONTROLLER PARAMETERS
# -----------------------------------------------
TARGET_SPEED  = 30 / 3.6   # 8.333 m/s
KP            = 0.5
KI            = 0.08
integral      = 0.0
prev_time     = time.time()

# -----------------------------------------------
# AEB PARAMETERS
# -----------------------------------------------
LANE_THRESHOLD    = 2.0    # m
TARGET_STOP_DIST  = 6.0    # m
TTC_THRESHOLD     = 3.0    # s
CLOSING_THRESHOLD = -0.3   # m/s
LOOKAHEAD_FACTOR  = 1.8    # s

# -----------------------------------------------
# STATE
# -----------------------------------------------
aeb_active     = False
aeb_brake      = 0.0
prev_speed     = None
prev_dist      = None
prev_dist_time = None

# -----------------------------------------------
# TERMINAL DISPLAY
# -----------------------------------------------
_last_print_t = -999.0
_last_phase   = None
PRINT_INTERVAL = 0.5       # s — how often to print status lines

def _terminal(speed, dist, ttc, brake, vrel, aeb, t):
    global _last_print_t, _last_phase

    kmh   = (speed or 0) * 3.6
    phase = ("STOPPED"  if aeb  and kmh < 0.5 else
             "BRAKING"  if aeb               else
             "LAUNCH"   if kmh < 5.0          else
             "CRUISE")
    cols  = {"LAUNCH":"\033[93m","CRUISE":"\033[96m",
             "BRAKING":"\033[91m","STOPPED":"\033[92m"}
    col   = cols[phase]
    rst   = "\033[0m"

    if phase != _last_phase:
        _last_phase = phase
        sep = "─" * 64
        print(f"\n{col}{sep}")
        if   phase == "LAUNCH":  print("  >> ZONE 1 — ACCELERATION     target: 30 km/h")
        elif phase == "CRUISE":  print("  >> ZONE 2 — CRUISE & MONITORING     radar active")
        elif phase == "BRAKING": print("  >> ZONE 3 — AEB ENGAGED     BRAKING")
        elif phase == "STOPPED":
            phys = (dist - SENSOR_OFFSET) if dist else 0
            ok   = "[PASS]" if 3.0 <= phys <= 9.0 else "[FAIL]"
            print(f"  >> VEHICLE STOPPED     gap = {phys:.2f} m     {ok}")
        print(f"{sep}{rst}")

    if (t - _last_print_t) < PRINT_INTERVAL:
        return
    _last_print_t = t

    ttc_s  = f"{min(ttc,99.9):5.1f} s" if (ttc and ttc < 900) else "   N/A "
    vrel_s = f"{vrel:+.2f}"             if vrel                else "   N/A"
    print(f"{col}[{t:6.2f}s] {phase:<8}  "
          f"Spd:{kmh:5.1f} km/h  Dist:{dist:6.2f} m  "
          f"TTC:{ttc_s}  Brake:{brake:.3f}  Vrel:{vrel_s} m/s{rst}")

# -----------------------------------------------
# LOG
# -----------------------------------------------
log        = []   # [t, speed, dist, ttc, brake, decel, aeb, vrel, ego_x, ego_y]
start_time = time.time()

# -----------------------------------------------
# MAIN LOOP
# -----------------------------------------------
while True:
    try:
        cm.read()

        speed = speed_q.data
        dist  = dist_q.data
        vrel  = vrel_q.data
        y     = y_q.data
        ego_x = pos_x_q.data
        ego_y = pos_y_q.data

        gas            = 0.0
        brake          = 0.0
        ttc            = None
        effective_vrel = None

        current_time = time.time() - start_time
        now_abs      = time.time()

        # ── Invalid distance guard ──
        if dist is not None and dist <= 0.5:
            if speed is not None and speed < 1.0:
                dist = TARGET_STOP_DIST
            else:
                continue

        # ── Effective Vrel ──
        if (dist is not None and prev_dist is not None
                and prev_dist_time is not None
                and speed is not None and speed > 4.0
                and dist < 60.0):
            dt_dist = now_abs - prev_dist_time
            if dt_dist > 0.01:
                dist_derived_vrel = (dist - prev_dist) / dt_dist
                if vrel is not None and vrel < CLOSING_THRESHOLD:
                    effective_vrel = vrel
                elif dist_derived_vrel < CLOSING_THRESHOLD:
                    effective_vrel = dist_derived_vrel

        if dist is not None:
            prev_dist      = dist
            prev_dist_time = now_abs

        # ── AEB trigger ──
        if not aeb_active and dist is not None and y is not None and speed is not None:
            if abs(y) < LANE_THRESHOLD:

                # Trigger A — TTC
                if effective_vrel is not None and effective_vrel < CLOSING_THRESHOLD:
                    ttc = dist / abs(effective_vrel)
                    if ttc < TTC_THRESHOLD:
                        aeb_active = True

                # Trigger B — Proximity (catches cut-in when Vrel ~ 0)
                dynamic_warn_dist = max(speed * LOOKAHEAD_FACTOR + SENSOR_OFFSET,
                                        SENSOR_OFFSET + 14.0)
                if not aeb_active and dist < dynamic_warn_dist:
                    aeb_active = True

        # ── AEB braking ──
        if aeb_active:
            gas = 0.0

            ttc = (dist / abs(effective_vrel)
                   if effective_vrel is not None and effective_vrel < -0.1
                   else 999.0)

            if dist is not None:
                if   dist > 26: target_brake = 0.10
                elif dist > 22: target_brake = 0.14
                elif dist > 19: target_brake = 0.17
                elif dist > 16: target_brake = 0.19
                elif dist > 13: target_brake = 0.21
                else:           target_brake = 0.23
            else:
                target_brake = 0.23

            aeb_brake = min(aeb_brake + 0.010, target_brake)
            brake     = aeb_brake

            if speed is not None and speed < 0.2:
                brake = 0.20

            if dist is not None and dist < (SENSOR_OFFSET + 4.0):
                aeb_brake = min(aeb_brake + 0.03, 0.30)
                brake     = aeb_brake

        # ── Speed control ──
        if not aeb_active and speed is not None:
            now = time.time()
            dt  = max(now - prev_time, 0.01)
            error = TARGET_SPEED - speed
            if speed > 5:
                integral += error * dt
            u         = KP * error + KI * integral
            prev_time = now

            if u > 0:
                gas   = min(u, 0.42)
                brake = 0.0
            else:
                gas   = 0.0
                if speed > TARGET_SPEED:
                    brake = min(-u, 0.10)

            if speed < 10:            gas = min(gas, 0.25)
            if speed > TARGET_SPEED - 0.5: gas *= 0.45
            if brake > 0:             integral = 0.0

        # ── Hard speed cap ──
        if speed is not None and speed > TARGET_SPEED + 0.2:
            gas = 0.0

        brake = min(max(brake, 0.0), 1.0)

        # ── Apply control ──
        cm.DVA_write(long_q,  1)
        cm.DVA_write(lat_q,   1)
        cm.DVA_write(gas_q,   gas)
        cm.DVA_write(brake_q, brake)

        # ── Deceleration ──
        decel = 0.0
        if prev_speed is not None and speed is not None:
            decel = max(-1.0, min(8.0, (prev_speed - speed) / 0.02))
        prev_speed = speed

        # ── Log + terminal ──
        log.append([current_time, speed, dist, ttc, brake, decel,
                    aeb_active, effective_vrel, ego_x, ego_y])
        _terminal(speed, dist, ttc, brake, effective_vrel, aeb_active, current_time)

        time.sleep(0.02)

    except KeyboardInterrupt:
        break

# -----------------------------------------------
# DEAD-RECKON ego X/Y if Car.tx/ty unavailable
# -----------------------------------------------
if log and log[0][8] is None:
    x = 0.0
    for i, row in enumerate(log):
        dt = (log[i][0] - log[i-1][0]) if i > 0 else 0.02
        x += (row[1] or 0.0) * dt
        log[i][8] = x
        log[i][9] = 0.0

# -----------------------------------------------
# UNPACK LOG
# -----------------------------------------------
t       = np.array([r[0] for r in log])
spd_ms  = np.array([r[1] or 0.0 for r in log])
dist_a  = np.array([r[2] or 0.0 for r in log])
ttc_a   = np.array([r[3] if r[3] is not None else np.nan for r in log])
brake_a = np.array([r[4] for r in log])
decel_a = np.array([r[5] for r in log])
aeb_a   = np.array([int(bool(r[6])) for r in log])
ego_x   = np.array([r[8] or 0.0 for r in log])
ego_y   = np.array([r[9] or 0.0 for r in log])
spd_kmh = spd_ms * 3.6
ttc_p   = np.where(ttc_a > 20, np.nan, ttc_a)
ks      = 9 if len(decel_a) >= 9 else (len(decel_a) | 1)
decel_s = medfilt(decel_a, kernel_size=max(ks, 3))

aeb_idx  = next((i for i,v in enumerate(aeb_a) if v), None)
stop_idx = next((i for i in range(len(spd_ms)-1,-1,-1) if spd_ms[i]>0.05), None)
t_aeb    = t[aeb_idx]  if aeb_idx  is not None else None
t_stop   = t[stop_idx] if stop_idx is not None else None
stop_dist= None
if aeb_idx is not None:
    for i in range(aeb_idx, len(spd_ms)):
        if spd_ms[i] < 0.10:
            stop_dist = dist_a[i] - SENSOR_OFFSET
            break

peak_decel = float(np.max(decel_s))
max_spd    = float(np.max(spd_kmh))
tmax       = float(t[-1])

# ── Pass / Fail ──
p_spd   = 27 <= max_spd    <= 33
p_decel = 5  <= peak_decel <= 8
p_stop  = stop_dist is not None and 3 <= stop_dist <= 9
def tk(b): return "[PASS]" if b else "[FAIL]"
sd = f"{stop_dist:.2f} m" if stop_dist is not None else "N/A"

print(f"\n{'='*60}")
print("  AEB SIMULATION — RESULTS SUMMARY")
print('='*60)
print(f"  {tk(p_spd):<8} Zone 1 speed    {max_spd:.1f} km/h   (27–33 km/h)")
print(f"  {tk(p_decel):<8} Peak decel      {peak_decel:.2f} m/s²  (5–8 m/s²)")
print(f"  {tk(p_stop):<8} Final stop gap  {sd}     (3–9 m)")
print(f"\n  OVERALL: {'PASS' if (p_spd and p_decel and p_stop) else 'REVIEW REQUIRED'}")
print('='*60 + "\n")

# -----------------------------------------------
# PLOT HELPERS
# -----------------------------------------------
OUT_DIR = "results/AEB/"
os.makedirs(OUT_DIR, exist_ok=True)

BG    = "#0f1117"
PANEL = "#1a1d27"
GRIDC = "#252535"
WHITE = "#e8e8f0"
MUTED = "#888899"

def _style(ax, title, xlabel, ylabel, xlim=None, ylim=None):
    ax.set_facecolor(PANEL)
    ax.set_title(title,   color=WHITE, fontsize=10, fontweight="bold", pad=6)
    ax.set_xlabel(xlabel, color=MUTED, fontsize=8)
    ax.set_ylabel(ylabel, color=MUTED, fontsize=8)
    ax.tick_params(colors=MUTED, labelsize=7)
    for sp in ax.spines.values(): sp.set_edgecolor("#333344")
    ax.grid(True, color=GRIDC, linewidth=0.5, linestyle="--")
    if xlim: ax.set_xlim(*xlim)
    if ylim: ax.set_ylim(*ylim)

def _zones(ax):
    if t_aeb:
        ax.axvspan(0, t_aeb,  alpha=0.07, color="#4fc3f7")
        ax.axvspan(t_aeb, tmax, alpha=0.07, color="#ef5350")
        ax.axvline(t_aeb,  color="#ef5350", lw=1.2, ls="--", alpha=0.75)
    if t_stop:
        ax.axvline(t_stop, color="#66bb6a", lw=1.2, ls="--", alpha=0.75)

def _leg(ax):
    ax.legend(fontsize=7, facecolor="#111", edgecolor="#444",
              labelcolor=WHITE, framealpha=0.85)

def _save(fig, name):
    path = os.path.join(OUT_DIR, _fname(name, "png"))
    plt.savefig(path, dpi=180, bbox_inches="tight",
                facecolor=fig.get_facecolor())
    plt.close(fig)
    print(f"  Saved → {path}")

# -----------------------------------------------
# PLOT 1 — BEV TRAJECTORY
# -----------------------------------------------
fig, ax = plt.subplots(figsize=(12, 5))
fig.patch.set_facecolor(BG)
tx_tgt = ego_x + dist_a
pts  = np.array([ego_x, ego_y]).T.reshape(-1,1,2)
segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
lc   = LineCollection(segs, cmap="plasma",
                      norm=plt.Normalize(0,33), linewidth=2.5, zorder=3)
lc.set_array(spd_kmh[:-1])
ax.add_collection(lc)
cb = fig.colorbar(lc, ax=ax, pad=0.01)
cb.set_label("Ego speed (km/h)", color=WHITE, fontsize=8)
cb.ax.yaxis.set_tick_params(color=MUTED)
plt.setp(cb.ax.yaxis.get_ticklabels(), color=MUTED, fontsize=7)
ax.plot(tx_tgt, ego_y, color="#ffa726", lw=1.2, ls="--",
        alpha=0.7, label="Target path")
if aeb_idx is not None:
    ax.scatter(ego_x[aeb_idx], ego_y[aeb_idx], s=80, color="#ef5350",
               zorder=5, marker="^", label=f"AEB trigger ({t_aeb:.1f} s)")
if stop_idx is not None:
    ax.scatter(ego_x[stop_idx], ego_y[stop_idx], s=80, color="#66bb6a",
               zorder=5, marker="s", label=f"Stopped (gap {sd})")
lane_w = 3.75
x_min, x_max = float(ego_x.min())-5, float(tx_tgt.max())+10
for off in [-lane_w, 0, lane_w]:
    ax.axhline(off, color="#555566", lw=0.8,
               ls="--" if off==0 else "-", alpha=0.6)
ax.axhspan(-lane_w, lane_w, alpha=0.04, color="white")
_style(ax, "Bird's Eye View — AEB Trajectory (colour = ego speed)",
       "Longitudinal position (m)", "Lateral position (m)")
ax.set_xlim(x_min, x_max)
ax.set_ylim(-lane_w*2, lane_w*2)
_leg(ax)
fig.tight_layout()
_save(fig, "AEB_BEV_Trajectory")

# -----------------------------------------------
# PLOT 2 — DISTANCE vs TIME
# -----------------------------------------------
fig, ax = plt.subplots(figsize=(10, 5))
fig.patch.set_facecolor(BG)
_style(ax, "Ego–Target Distance vs Time",
       "Time (s)", "Distance (m)", xlim=(0,tmax))
_zones(ax)
ax.plot(t, dist_a, color="#ffb74d", lw=2, label="Sensor distance")
ax.axhline(SENSOR_OFFSET+9, color="#ef5350", ls="--", lw=1.1,
           label=f"Max stop {SENSOR_OFFSET+9:.0f} m  (physical 9 m)")
ax.axhline(SENSOR_OFFSET+3, color="#ef5350", ls=":", lw=0.9,
           label=f"Min stop {SENSOR_OFFSET+3:.0f} m  (physical 3 m)")
ax.axhline(SENSOR_OFFSET+6, color="#66bb6a", ls="--", lw=0.9,
           label=f"Target {SENSOR_OFFSET+6:.0f} m  (physical 6 m)")
if t_stop and stop_dist is not None:
    ax.annotate(f"Stop gap\n{stop_dist:.2f} m",
                xy=(t_stop, dist_a[stop_idx]),
                xytext=(t_stop-3.5, dist_a[stop_idx]+12),
                color="#66bb6a", fontsize=8,
                arrowprops=dict(arrowstyle="->", color="#66bb6a", lw=1.1))
_leg(ax)
fig.tight_layout()
_save(fig, "AEB_Distance_vs_Time")

# -----------------------------------------------
# PLOT 3 — DECELERATION + BRAKE
# -----------------------------------------------
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
fig.patch.set_facecolor(BG)
for ax in (ax1, ax2): _zones(ax)

_style(ax1, "Deceleration Profile vs Time",
       "", "Deceleration (m/s²)", xlim=(0,tmax), ylim=(0,10.5))
ax1.plot(t, decel_s, color="#a5d6a7", lw=1.8, label="Deceleration (smoothed)")
ax1.fill_between(t, decel_s, alpha=0.15, color="#a5d6a7")
ax1.axhline(8, color="#ef5350", ls="--", lw=1.1, label="Max 8 m/s²")
ax1.axhline(5, color="#ffa726", ls="--", lw=1.0, label="Min 5 m/s²")
ax1.fill_between(t, 5, np.clip(decel_s,5,8),
                 where=(decel_s>=5)&(decel_s<=8),
                 alpha=0.18, color="#66bb6a", label="Compliant (5–8 m/s²)")
pi = int(np.argmax(decel_s))
ax1.annotate(f"Peak {decel_s[pi]:.1f} m/s²",
             xy=(t[pi], decel_s[pi]), xytext=(t[pi]+0.6, decel_s[pi]+0.6),
             color="#a5d6a7", fontsize=8,
             arrowprops=dict(arrowstyle="->", color="#a5d6a7", lw=1))
_leg(ax1)

_style(ax2, "Brake Command vs Time",
       "Time (s)", "Brake (0–1 normalised)", xlim=(0,tmax), ylim=(0,0.38))
ax2.fill_between(t, brake_a, alpha=0.25, color="#ef5350")
ax2.plot(t, brake_a, color="#ef5350", lw=1.8, label="Brake command")
_leg(ax2)
fig.suptitle("AEB — Deceleration & Brake Force Profile",
             color=WHITE, fontsize=12, fontweight="bold")
fig.tight_layout()
_save(fig, "AEB_Decel_Brake_Profile")

# -----------------------------------------------
# PLOT 4 — FULL 6-PANEL SUMMARY
# -----------------------------------------------
fig = plt.figure(figsize=(17, 11))
fig.patch.set_facecolor(BG)
gs  = fig.add_gridspec(3, 2, hspace=0.48, wspace=0.30,
                        left=0.07, right=0.97, top=0.87, bottom=0.06)
ax  = [[fig.add_subplot(gs[r,c]) for c in range(2)] for r in range(3)]

# Speed
_style(ax[0][0], "Speed vs Time", "Time (s)", "Speed (km/h)",
       xlim=(0,tmax), ylim=(0,38))
_zones(ax[0][0])
ax[0][0].plot(t, spd_kmh, color="#4fc3f7", lw=1.8, label="Ego speed")
ax[0][0].axhline(30, color="#66bb6a", ls="--", lw=1.0, label="Target 30 km/h")
ax[0][0].axhline(27, color="#ffa726", ls=":", lw=0.8)
ax[0][0].axhline(33, color="#ffa726", ls=":", lw=0.8, label="27–33 band")
_leg(ax[0][0])

# Distance
_style(ax[0][1], "Distance vs Time", "Time (s)", "Distance (m)",
       xlim=(0,tmax))
_zones(ax[0][1])
ax[0][1].plot(t, dist_a, color="#ffb74d", lw=1.8, label="Sensor dist")
ax[0][1].axhline(SENSOR_OFFSET+9, color="#ef5350", ls="--", lw=0.9, label="Max 9 m")
ax[0][1].axhline(SENSOR_OFFSET+3, color="#ef5350", ls=":", lw=0.8, label="Min 3 m")
ax[0][1].axhline(SENSOR_OFFSET+6, color="#66bb6a", ls="--", lw=0.8, label="Target 6 m")
_leg(ax[0][1])

# TTC
_style(ax[1][0], "Time-to-Collision vs Time", "Time (s)", "TTC (s)",
       xlim=(0,tmax), ylim=(0,22))
_zones(ax[1][0])
ax[1][0].plot(t, ttc_p, color="#ce93d8", lw=1.6, label="TTC (cap 20 s)")
ax[1][0].axhline(3.0, color="#ef5350", ls="--", lw=1.0, label="Threshold 3 s")
_leg(ax[1][0])

# Brake
_style(ax[1][1], "Brake Command vs Time", "Time (s)", "Brake (0–1)",
       xlim=(0,tmax), ylim=(0,0.36))
_zones(ax[1][1])
ax[1][1].fill_between(t, brake_a, alpha=0.22, color="#ef5350")
ax[1][1].plot(t, brake_a, color="#ef5350", lw=1.6, label="Brake command")
_leg(ax[1][1])

# Decel
_style(ax[2][0], "Deceleration vs Time", "Time (s)", "Deceleration (m/s²)",
       xlim=(0,tmax), ylim=(0,10.5))
_zones(ax[2][0])
ax[2][0].plot(t, decel_s, color="#a5d6a7", lw=1.6, label="Decel (smoothed)")
ax[2][0].fill_between(t, 5, np.clip(decel_s,5,8),
                      where=(decel_s>=5)&(decel_s<=8),
                      alpha=0.18, color="#66bb6a", label="Compliant 5–8")
ax[2][0].axhline(8, color="#ef5350", ls="--", lw=1.0, label="Max 8 m/s²")
ax[2][0].axhline(5, color="#ffa726", ls="--", lw=0.9, label="Min 5 m/s²")
ax[2][0].annotate(f"Peak {decel_s[pi]:.1f} m/s²",
                  xy=(t[pi], decel_s[pi]), xytext=(t[pi]+0.5, decel_s[pi]+0.6),
                  color="#a5d6a7", fontsize=7,
                  arrowprops=dict(arrowstyle="->", color="#a5d6a7", lw=0.8))
_leg(ax[2][0])

# AEB state
_style(ax[2][1], "AEB System State vs Time", "Time (s)", "",
       xlim=(0,tmax), ylim=(-0.05,1.2))
_zones(ax[2][1])
ax[2][1].fill_between(t, aeb_a, alpha=0.30, color="#ef5350")
ax[2][1].plot(t, aeb_a, color="#ef5350", lw=1.4, label="AEB Active")
ax[2][1].set_yticks([0,1])
ax[2][1].set_yticklabels(["INACTIVE","ACTIVE"], color=MUTED, fontsize=7)
if t_aeb:
    ax[2][1].annotate(f"Trigger\n{t_aeb:.1f} s",
                      xy=(t_aeb, 0.5), xytext=(t_aeb+0.8, 0.78),
                      color="#ef5350", fontsize=7,
                      arrowprops=dict(arrowstyle="->", color="#ef5350", lw=0.8))
_leg(ax[2][1])

# Zone legend strip
elems = [
    mpatches.Patch(color="#4fc3f7", alpha=0.35, label="Zone 1 — Acceleration"),
    mpatches.Patch(color="#4fc3f7", alpha=0.35, label="Zone 2 — Threat detection"),
    mpatches.Patch(color="#ef5350", alpha=0.35, label="Zone 3 — AEB braking"),
    Line2D([0],[0], color="#ef5350", lw=1.2, ls="--", label="AEB trigger"),
    Line2D([0],[0], color="#66bb6a", lw=1.2, ls="--", label="Vehicle stopped"),
]
fig.legend(handles=elems, loc="upper center", ncol=5, fontsize=8,
           facecolor="#111", edgecolor="#444", labelcolor=WHITE,
           framealpha=0.9, bbox_to_anchor=(0.5, 0.965))

# Summary bar
bar = (f"{tk(p_spd)} Zone 1: {max_spd:.1f} km/h (27–33)   "
       f"{tk(p_decel)} Peak decel: {peak_decel:.1f} m/s² (5–8)   "
       f"{tk(p_stop)} Stop gap: {sd} (3–9 m)")
overall = "PASS — ALL CRITERIA MET" if (p_spd and p_decel and p_stop) \
          else "REVIEW — CHECK CRITERIA"
fig.text(0.5, 0.923, f"{overall}   |   {bar}",
         ha="center", va="center", fontsize=8.5, color=WHITE, style="italic",
         bbox=dict(boxstyle="round,pad=0.4", fc=PANEL, ec="#445566", lw=1))
fig.suptitle("Automatic Emergency Braking — Simulation Results Summary",
             color=WHITE, fontsize=13, fontweight="bold", y=0.997)
_save(fig, "AEB_Results_Summary")

# -----------------------------------------------
# SAVE CSV
# -----------------------------------------------
csv_path = os.path.join(OUT_DIR, _fname("AEB_DataLog", "csv"))
with open(csv_path, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["Time_s","Speed_ms","Speed_kmh","Distance_m",
                "TTC_s","Brake","Decel_ms2","AEB_Active","Vrel_ms",
                "Ego_X_m","Ego_Y_m"])
    for r in log:
        row = list(r)
        row.insert(2, round((r[1] or 0)*3.6, 3))
        w.writerow(row)
print(f"  Saved → {csv_path}")
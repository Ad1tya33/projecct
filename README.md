# 🚗 Team Elecruisers — ADAS Simulation Repository
### aBAJA SAEINDIA 2026 | IPG CarMaker 14.0.1 | SRM Institute of Science and Technology

[![Team](https://img.shields.io/badge/Team-Elecruisers-blue?style=flat-square)](https://github.com/Ad1tya33)
[![Competition](https://img.shields.io/badge/Competition-aBAJA%20SAEINDIA%202026-orange?style=flat-square)]()
[![Simulator](https://img.shields.io/badge/Simulator-IPG%20CarMaker%2014.0.1-green?style=flat-square)]()
[![Language](https://img.shields.io/badge/Language-Python-yellow?style=flat-square)]()

---

## 📖 Overview

This repository contains the complete ADAS simulation stack developed by **Team Elecruisers** for the **aBAJA SAEINDIA 2026** competition. All three mandatory ADAS challenges — **Automatic Emergency Braking (AEB)**, **Lane Keeping Assist (LKA)**, and **Endurance** — are implemented and tested inside IPG CarMaker 14.0.1 using Python-based controllers without ROS2 or MATLAB.

The repository is organized into five branches, each corresponding to a distinct category of deliverables.

---

## 🌿 Branch Structure

```
main
├── src          ← Python source code for AEB, LKA, Endurance controllers
├── config       ← Sensor configuration screenshots + sensor config report
├── scenarios    ← CarMaker scenario files for all challenges
├── results      ← Simulation output: videos, plots, data logs, analysis
└── docs         ← Architecture doc, hardware report, sensor config document
```

---

## 🌿 Branch Details

### `src` — Source Code

Python-based ADAS controllers for all three challenges. Control commands are sent to CarMaker via `pycarmaker DVA_write()` and vehicle state signals are read using raw TCP sockets.

| Module | Description |
|--------|-------------|
| `aeb/` | Automatic Emergency Braking — detects obstacles via Radar (RAD00), computes TTC, applies brake via `DM.Brake` |
| `lka/` | Lane Keeping Assist — processes Camera RSI frames (640×480 RGB over TCP port 2210), BEV transform + sliding window lane detection, Pure Pursuit steering |
| `endurance/` | Endurance run controller — combined longitudinal and lateral control for sustained circuit laps |

**Key DVA signals used:**

| Signal | Unit | Use |
|--------|------|-----|
| `Vhcl.v` | m/s | Vehicle speed — throttle/brake scaling, Pure Pursuit lookahead |
| `DM.Brake` | — | Brake command output (AEB) |
| `DM.Gas` | — | Throttle command output |
| `DM.Steer.Ang` | rad | Steering angle output (LKA) |
| `Car.YawRate` | rad/s | Logged for post-run analysis |
| `Car.ay` | m/s² | Logged for post-run analysis |

---

### `config` — Sensor Configuration

Screenshots of all sensor configurations set up in **CarMaker Office → Vehicle Data Set → Sensors** tab, plus a detailed sensor configuration report.

| Sensor | Name | Key Parameters |
|--------|------|---------------|
| Inertial | `IN00` | Mount: x=1.2, y=0.0, z=0.6 m \| Calc class: Global |
| Road | `RD00` | Mount: x=1.5, y=0.0, z=0.3 m \| Preview: 10 m along route |
| Radar | `RAD00` | Mount: x=1.8, y=0.0, z=0.5 m \| FoV: 120°×30° \| Range: 1–160 m \| Freq: 76 GHz \| Cycle: 50 ms |
| Camera RSI | `CAM00` | Mount: x=1.6, y=0.0, z=1.5 m \| 640×480px \| FoV: 75° \| TCP port: 2210 \| Cycle: 33 ms |

> Full sensor parameter tables, RF settings, cluster configuration, and compliance notes are in the sensor config report inside this branch.

---

### `scenarios` — Scenario Files

All CarMaker scenario files used for testing and submission, organized by challenge.

```
scenarios/
├── aeb/
│   └── aeb_scenario.zip / .TestRun      ← Single file; copy directly to CarMaker project Data/TestRun/
├── lka/
│   ├── lka_track.xodr                   ← Road network (OpenDRIVE)
│   └── lka_scenario.xosc                ← Scenario definition (OpenSCENARIO)
├── endurance/
│   ├── endurance_track.xodr             ← Road network (OpenDRIVE)
│   └── endurance_scenario.xosc          ← Scenario definition (OpenSCENARIO)
└── self_created/
    ├── self_aeb_scenario                ← Custom AEB test scene
    ├── self_lka_scene_1.xodr/.xosc      ← Custom LKA scene 1
    └── self_lka_scene_2.xodr/.xosc      ← Custom LKA scene 2
```

**How to use scenario files in CarMaker:**

- **AEB:** Copy the scenario file directly into your CarMaker project's `Data/TestRun/` directory and load it from CarMaker Office → TestRun.
- **LKA / Endurance:** Import the `.xodr` road file via CarMaker's Road Editor, then load the `.xosc` scenario via the TestRun manager.

---

### `results` — Simulation Results

All simulation output evidence organized by challenge.

```
results/
├── AEB/
│   ├── aeb_plot.png              ← Brake force / TTC / velocity over time
│   ├── aeb_bev_plot.png          ← Bird's-eye view trajectory plot
│   ├── aeb_video.mp4             ← Successful AEB activation recording
│   ├── aeb_failure_video.mp4     ← Edge case / failure mode recording
│   ├── aeb_data_log.csv          ← Raw DVA signal log
│   └── aeb_analysis.pdf          ← Performance analysis report
├── LKA/
│   ├── lka_video.mp4             ← Lane keeping run recording
│   ├── lka_plot.png              ← Lateral deviation / steering angle over time
│   └── lka_analysis.pdf          ← RMSE / ILC performance report
├── Endurance/
│   ├── endurance_video.mp4       ← Full endurance run recording
│   ├── endurance_plot.png        ← Combined speed / deviation plots
│   └── endurance_analysis.pdf    ← Lap analysis report
└── Self_Created/
    ├── self_aeb_result/          ← Results for custom AEB scenario
    ├── self_lka_scene1_result/   ← Results for custom LKA scene 1
    └── self_lka_scene2_result/   ← Results for custom LKA scene 2
```

---

### `docs` — Documentation

| Document | Description |
|----------|-------------|
| `software_architecture.pdf` | Full ADAS pipeline architecture — perception, planning, control flow |
| `hardware_report.pdf` | Hardware spec and physical vehicle sensor placement |
| `sensor_config_screenshots/` | CarMaker Office sensor config screenshots (also in `config` branch) |
| `sensor_config_doc.pdf` | Detailed sensor parameter document with compliance table (R1–R5, B.1, B.5) |

---

## 🛠️ Setup & Running

### Prerequisites

- IPG CarMaker 14.0.1 (Windows)
- Python 3.9+
- Virtual environment at `aeb_env` (or create your own)

```bash
# Create and activate venv
python -m venv aeb_env
aeb_env\Scripts\activate        # Windows

# Install dependencies
pip install pycarmaker numpy opencv-python
```

### Running a Controller

1. Open CarMaker Office and load the desired scenario from `Data/TestRun/`.
2. Start the CarMaker simulation server.
3. In a separate terminal (with venv active):

```bash
# AEB
python src/aeb/aeb_controller.py

# LKA
python src/lka/lka_controller.py

# Endurance
python src/endurance/endurance_controller.py
```

4. Press **Start** in CarMaker. The Python controller will connect automatically.

> **Note:** The Camera RSI stream (LKA) must be received on `localhost:2210`. Ensure the Sensor Cluster in CarMaker Office is configured with Client Type = User Defined, Socket = 2210.

---

## 🏁 Competition Challenges

| Challenge | Approach | Sensor Used | Status |
|-----------|----------|-------------|--------|
| AEB | TTC-based threshold braking | Radar RAD00 | ✅ Implemented |
| LKA | BEV transform + Sliding Window + Pure Pursuit | Camera RSI CAM00 | ✅ Implemented |
| Endurance | Combined longitudinal + lateral control | Radar + Camera + Inertial | ✅ Implemented |

---

## 📋 Rule Compliance

| Rule | Requirement | Status |
|------|-------------|--------|
| R1 | No ground-truth lane/object state as algorithm input | ✅ Compliant |
| R2 | No simulator-internal privileged APIs | ✅ Compliant |
| R3 | Inputs from configured sensor models only | ✅ Compliant |
| R4 | Approved sensor types only (Camera, Radar, IMU) | ✅ Compliant |
| R5 | Sensor config validated by Technical Committee | ⏳ Pending |
| B.1 | Real-world reproducibility | ✅ Compliant |
| B.5 | Min rates: Perception ≥ 10 Hz, Control ≥ 20 Hz | ✅ Compliant (30 Hz) |

---

## 👥 Team

**Team Elecruisers** — SRM Institute of Science and Technology  
Autonomous Vehicle Division | aBAJA SAEINDIA 2026

---

## 📄 License

This repository is submitted as part of the aBAJA SAEINDIA 2026 competition. All rights reserved by Team Elecruisers, SRM IST.

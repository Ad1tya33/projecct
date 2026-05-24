# Submission Guidelines — aBAJA SAEINDIA 2026 | Software Virtual World Simulation

> **This is your team's official submission repository. Keep it private until the deadline. Maintain a meaningful commit history throughout the development season.**

---

## Part A — Overview

The Software Virtual World Simulation event assesses your autonomous vehicle capabilities in realistic, physics-accurate virtual environments. The primary objective is **not** to pass a test — it is to use simulation as a rigorous engineering tool to analyse, validate, and critically understand your ADAS system's behaviour.

**All points are awarded during the Presentation. Simulation outputs (videos, graphs, plots) are the technical evidence underpinning your presentation.**

### Three Mandatory Events — All Must Be Completed

| # | Event | Control Authority |
|---|-------|------------------|
| 1 | Automatic Emergency Braking (AEB) | Team controls **throttle & brake** only |
| 2 | Lane Keep Assist (LKA) | Team controls **steering** only |
| 3 | Endurance (AEB + LKA Combined) | Team controls **throttle, brake & steering** (full autonomous) |

> ❗ Failure to complete any mandatory event = **zero marks** for that event.

---

## Part B — System Constraints

### B.1 Real-World Reproducibility
Every design decision must be physically implementable on a real aBAJA vehicle. Any shortcut that cannot be reproduced on a real vehicle is **prohibited** and will be penalised.

### B.2 Permitted Sensors

| Sensor | Primary Use | Notes |
|--------|-------------|-------|
| Camera (Monocular / Stereo) | Lane detection, object detection | Configurable FOV, update rate, noise, mount |
| LiDAR | 3D obstacle detection & mapping | Configurable point density, range, noise |
| RADAR | Object detection, range & velocity | Configurable range, resolution, angular FOV |
| IMU | Vehicle dynamics (acceleration, angular rate) | Configurable mounting position, refresh rate |
| GNSS (GPS) | Global localisation | Real-world accuracy modelling required |
| Custom sensors | Any additional type | **Must be approved by Technical Review before use** |

**All sensors must:**
- Output raw or minimally processed data (images, point clouds, range data)
- Use physically plausible FOV, update rate, and noise settings
- Be mounted at physically plausible positions on the vehicle
- Be fully documented and submitted for Technical Review

### B.3 Programming Language
Any language/framework that interfaces with your simulator: Python, C++, MATLAB/Simulink, ROS 2.

### B.4 Strictly Prohibited
- Using simulator ground-truth object state APIs as algorithm inputs
- Accessing the simulator's internal scene graph or physics engine for perception
- Hard-coding scenario-specific object positions or trajectories
- Using pre-recorded sensor data from the same scenario as a replay feed
- Any technique that bypasses genuine perception, planning, or control

### B.5 Compute Budget Constraint
Design your stack as if it will run on your declared embedded compute platform (e.g., NVIDIA Jetson, Raspberry Pi CM4). Total cost of sensors + compute + electronics must comply with the rulebook budget cap.

**Minimum real-time rates:** Perception ≥ 10 Hz | Control ≥ 20 Hz

Your **Hardware-Algorithm Compatibility Report** (deliverable #12) must include:
1. Proposed compute platform + itemised cost breakdown showing rulebook compliance
2. Published or measured benchmarks showing your model runs at required Hz on target hardware
3. If simulation used a heavier model: explicit comparison with the deployment model and trade-off analysis
4. Justification that total sensor + compute cost is within the budget cap

---

## Part C — Mandatory Scenarios

### C.1 — Automatic Emergency Braking (AEB)

**Scenario M1 (Mandatory):** A target vehicle cuts into the ego lane from an adjacent lane and decelerates to a stop.

| Zone | Description | Requirement |
|------|-------------|-------------|
| Zone 1 — Acceleration | Ego accelerates from rest | Must reach **30 km/h ±3 km/h** |
| Zone 2 — Threat Detection | Target cuts in from adjacent lane | Detect and brake at **5–8 m/s²** |
| Zone 3 — Stopping | Ego comes to a complete stop | Stop **3–9 m** from rear of target |

> ⚠️ Cut-in parameters (relative speed, lateral entry speed, distance) **must vary across runs**. Hard-coded responses will be penalised.

**Pass / Fail:**

| Status | Condition |
|--------|-----------|
| ✅ PASS | 30 km/h in Zone 1 · detects cut-in · 5–8 m/s² deceleration · stops 3–9 m from target |
| DNS | Fails to initiate movement in Zone 1 |
| DQ | Fails to brake / wrong speed / wrong deceleration / wrong stop distance / collision |

**Deliverables → `results/AEB/`**
- [ ] Ego-vehicle POV video — cut-in, braking, final stop (`.mp4`)
- [ ] Bird's Eye View (BEV) trajectory plot, colour-coded by velocity (`.png`)
- [ ] Distance vs. Time plot — ego to cut-in target (`.png`)
- [ ] Deceleration / brake force profile over time (`.png`)
- [ ] Written analysis: detection method, sensors used, response latency

---

### C.2 — Lane Keep Assist (LKA)

**Scenario:** Continuous track — straight section followed by variable-curvature curved section. Multiple lane marking types (solid, dashed, varying visibility). Parameters vary between runs.

**Metrics:**

| Metric | Definition | Threshold |
|--------|------------|-----------|
| RMSE | Lateral deviation of front axle from lane centre | Penalised beyond ±0.1 m |
| ILC | % of time front axle within lane boundaries | Higher is better |
| DNF | Front axle exits lane and does not return | Within 5 seconds |

**Deliverables → `results/LKA/`**
- [ ] Ego-vehicle POV video — full run, straight + curved (`.mp4`)
- [ ] Lateral deviation (tRoad) vs. Time plot (`.png`)
- [ ] RMSE value + ILC percentage with calculation methodology (`.pdf` or Jupyter)
- [ ] Analysis: greatest deviation zones, algorithm response to curvature
- [ ] Comparison across **at least two** track configurations or lane-marking types

---

### C.3 — Endurance (AEB + LKA Combined)

**Three continuous stages, full autonomous control:**

| Stage | Description |
|-------|-------------|
| 1 — Straight Lane Keeping | Navigate straight road at lane centre |
| 2 — Curved Road Navigation | Navigate variable-curvature section |
| 3 — Dynamic Object Insertion | Detect cut-in, stop, resume safely when object clears |

> ⚠️ Stage 3 cut-in parameters must vary between runs. No hard-coding.

**Deliverables → `results/Endurance/`**
- [ ] Full run video — all 3 stages continuously (`.mp4`)
- [ ] Speed profile vs. Time, annotated by stage (`.png`)
- [ ] Lateral deviation vs. Time during straight and curved sections (`.png`)
- [ ] Distance vs. Time to cut-in target during Stage 3 (`.png`)
- [ ] Stage-by-stage performance analysis including LKA ↔ AEB mode transition

---

### C.4 — Self-Created Scenarios *(Strongly Encouraged)*

Design additional scenarios following **ISO 21448 (SOTIF)** to explore system limits. Examples:
- AEB with a pedestrian crossing
- AEB with a progressively decelerating vehicle ahead
- LKA with degraded / partial lane markings (dirt, glare, fading)
- Combined AEB + LKA with multiple sequential obstacles
- Adverse weather (rain, fog — if supported by your simulator)

Submit results and analysis to `results/Self_Created/`.

---

## Part D — Simulation Environment Specs

- **Road layout:** Left-hand drive (LHD) — Indian traffic convention
- **Minimum:** Two lanes per direction
- **Lane boundaries:** Clearly marked; straight and curved sections

**Supported import formats:** OpenDRIVE (`.xodr`), FBX, UDatasmith, glTF / GLB

**Vehicle model requirements:**
- Mass, wheelbase, and track width must match your actual aBAJA build
- Use physically representative tyre models
- Submit 3D vehicle mesh (`docs/`)

---

## Part E — Restrictions

| ID | Rule |
|----|------|
| R1 | No ground-truth object states (position, velocity, lane data) as algorithm inputs |
| R2 | No simulator-internal APIs exposing privileged world-state data |
| R3 | Algorithm inputs from configured sensor models only |
| R4 | Approved automotive sensor types only (Camera, LiDAR, RADAR, IMU, GNSS) |
| R5 | Sensor configuration must be validated by Technical Review before presentation |
| R6 | No manipulation or falsification of any simulation output |
| R7 | Submitted code must produce reproducible results |
| R8 | All code, config, and scenario files version-controlled in this GitHub repo with meaningful commit history |
| R9 | No external data feeds (real-world maps, live traffic, pre-recorded streams) |
| R10 | No cross-team sharing of core algorithm code |

---

## Part F — Submission Requirements

### Repository Structure

```
├── src/                     ← All source code (perception, planning, control)
├── config/                  ← Sensor configs, vehicle parameters
├── scenarios/               ← Scenario files (mandatory + self-created)
├── results/
│   ├── AEB/                ← Video, plots, data logs
│   ├── LKA/                ← Video, plots, RMSE/ILC report
│   ├── Endurance/          ← Video, combined plots
│   └── Self_Created/       ← Additional scenario results
├── docs/                    ← Architecture doc, sensor config, hardware report
├── README.md                ← Repository overview and run instructions
└── Submission_Guidelines.md
```

### File Naming Convention

All files must follow: **`TeamID_TeamName_[Deliverable]`**

Replace `TeamID` with your numeric team ID and `TeamName` with your team name (underscores instead of spaces). Examples:

| Deliverable | Example Filename |
|-------------|-----------------|
| AEB video | `264023_Jarvis_AEBVideo.mp4` |
| LKA lateral deviation plot | `264023_Jarvis_LKA_DeviationPlot.png` |
| System architecture document | `264023_Jarvis_SystemArchitecture.pdf` |
| Sensor configuration file | `264023_Jarvis_SensorConfig.pdf` |
| Hardware compatibility report | `264023_Jarvis_HardwareReport.pdf` |
| Endurance combined plots | `264023_Jarvis_Endurance_Plots.png` |

> ⚠️ Submissions not following this naming convention or the folder structure **may not be evaluated**.

### Complete Deliverables Checklist

| # | Deliverable | Folder | Format |
|---|-------------|--------|--------|
| 1 | Full autonomous driving stack source code | `src/` | Code |
| 2 | AEB — Ego-vehicle POV video | `results/AEB/` | `.mp4` |
| 3 | AEB — BEV trajectory plot (colour-coded by velocity) | `results/AEB/` | `.png` |
| 4 | AEB — Distance vs. Time plot | `results/AEB/` | `.png` |
| 5 | LKA — Ego-vehicle POV video | `results/LKA/` | `.mp4` |
| 6 | LKA — Lateral deviation vs. Time plot | `results/LKA/` | `.png` |
| 7 | LKA — RMSE & ILC Report | `results/LKA/` | `.pdf` |
| 8 | Endurance — Full run video (all 3 stages) | `results/Endurance/` | `.mp4` |
| 9 | Endurance — Combined data plots | `results/Endurance/` | `.png` |
| 10 | System Architecture Document | `docs/` | `.pdf` |
| 11 | Sensor Configuration File | `docs/` | `.pdf` |
| 12 | Hardware-Algorithm Compatibility Report | `docs/` | `.pdf` |
| 13 | Self-Created Scenarios — results & analysis | `results/Self_Created/` | `.pdf` |
| 14 | 3D Vehicle Mesh | `docs/` | Simulator format |

> 📹 Large video files: upload to Google Drive / OneDrive and link in your `README.md`. Ensure sharing is open to the judging panel.

### Deadlines
- Announced via the **official aBAJA SAEINDIA forum**
- Sensor config validation: **at least 2 weeks before** the presentation date
- **Do not push to main after the submission deadline**

---

## Part G — Evaluation & Scoring

### Scoring *(out of 70 → converted to 50 final points)*

| Criteria | Marks |
|----------|-------|
| Technical depth — AEB simulation analysis | 5 |
| Technical depth — LKA simulation analysis | 5 |
| Technical depth — Endurance simulation analysis | 10 |
| Sensor selection and justification | 10 |
| System architecture quality and modularity | 10 |
| Critical analysis & limitations via self-created scenarios | 10 |
| Real-world transferability, hardware-algorithm compatibility, budget justification | 10 |
| Presentation quality and Q&A performance | 10 |
| **Total** | **70** |

### Presentation Format
- **40 minutes per team** — 20 min presentation + 20 min Q&A
- No maximum slide count

### What Judges Look For
- Depth of understanding of your own ADAS system
- Rigour with which simulation was used to make and validate engineering decisions
- Whether design choices are grounded in data and critical analysis
- Whether your system could realistically be deployed on a real vehicle

> A team that honestly acknowledges limitations and explains *why* their system behaved as it did will score **higher** than a team with a "perfect" simulation but shallow analysis.

### Presentation Must Cover
1. System overview & architecture (modules, data flow, safety mechanisms)
2. Sensor configuration & justification (specs, mounting, noise, trade-offs)
3. AEB analysis (TTC calculations, braking profiles, false positives/negatives)
4. LKA analysis (RMSE, ILC, straight vs. curved performance, steering behaviour)
5. Endurance analysis (stage-by-stage, LKA ↔ AEB mode transitions)
6. Self-created scenario findings (failure modes, edge cases, proposed improvements)
7. Real-world transferability & budget justification (compute platform, benchmarks, cost)

---

## Part H — Technical Review & Academic Integrity

**Sensor Configuration Review:** Submit to the Technical Committee for assessment of physical plausibility, parameterisation accuracy, and absence of prohibited ground-truth feeds. Address all feedback before the presentation date.

**Academic Integrity:** The Technical Committee may request live simulation demos, ask teams to explain any code or data during Q&A, and investigate suspected violations. Teams found in violation — including plagiarism or data falsification — will be **disqualified from aBAJA 2026** and may be barred from future events.

---

*aBAJA SAEINDIA 2026 — Software Virtual World Simulation Event*
*For queries, contact the Technical Committee via the official aBAJA SAEINDIA forum.*

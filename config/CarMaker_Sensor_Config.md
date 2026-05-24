# CarMaker Sensor & Vehicle Configuration — LKA

Simulator: **IPG CarMaker 15.0**
Vehicle file: `CM_15/Data/Vehicle/Examples/Demo_IPG_CompanyCar_SensorGroundTruth_LanesAndRoots`

## Camera RSI Sensor

| Parameter     | Value              |
|---------------|--------------------|
| Resolution    | 640 × 480 px       |
| Frame rate    | ~30 Hz             |
| Output        | RGB frames via TCP |
| TCP host/port | localhost : 2210   |
| ISO           | 800                |
| F-Stop        | 2.8                |
| Shutter speed | 250 Hz             |
| Tone mapping  | Filmic             |
| DVA name      | `CARS00`           |

> **Note**: CarMaker defaults (ISO 100, F/11, no tone mapping) produced near-black
> frames (pixel values 10–30). Corrected values above give adequate contrast for the
> BEV + Sliding Window pipeline. See `results/LKA/LKA_Observations.pdf` for full analysis.

## Road / Lane Sensor (DVA)

| DVA Quantity | Description |
|---|---|
| `Car.Road.Route.DevDist` | Lateral deviation from route [m] |
| `Car.Road.Route.CurveXY` | Road curvature [1/m] |
| `Sensor.Road.Vhcl.VehSensor_1.Route.Deviation.Dist` | Sensor-based lateral deviation [m] |

Sensor mount: `VehSensor_1` (Road Sensor `RD00`)

## Inertial / Vehicle State (DVA)

| DVA Quantity | Description |
|---|---|
| `Vhcl.v` | Vehicle speed [m/s] |
| `Car.YawRate` | Yaw rate [rad/s] |
| `Car.ay` | Lateral acceleration [m/s²] |

## Vehicle Parameters

| Parameter | Value |
|---|---|
| Model | Demo_IPG_CompanyCar |
| Mass | 1391 kg |
| Wheelbase | 2.622 m |
| Steering ratio | 16 : 1 |

## Control Parameters (see `src/LKA/config.py`)

| Parameter | Value |
|---|---|
| Target speed | 30 km/h |
| Control loop rate | 30 Hz |
| Pure Pursuit lookahead | 5.0 m |
| Max road-wheel angle | ±0.20 rad |
| Rate limit | 0.03 rad/frame |
| PID Kp / Ki / Kd | 0.10 / 0.005 / 0.08 |

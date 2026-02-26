# Vehicle Dynamics MBD — 14-DOF Heavy Vehicle & AARB Control

> A Model-Based Design (MBD) framework for simulating and controlling the
> lateral/roll dynamics of a heavy vehicle (Chevrolet Blazer 2001 reference)
> using MATLAB/Simulink, with ISO 19364:2016-compliant validation maneuvers.

[![MATLAB R2020b+](https://img.shields.io/badge/MATLAB-R2020b%2B-blue.svg)](https://www.mathworks.com/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

---

## Table of Contents

1. [Project Overview](#project-overview)
2. [Architecture](#architecture)
3. [Prerequisites](#prerequisites)
4. [Quick Start](#quick-start)
5. [Validation Maneuvers](#validation-maneuvers)
6. [Controllers](#controllers)
7. [Contributing](#contributing)
8. [License](#license)

---

## Project Overview

This repository implements a **14 Degrees-of-Freedom (DOF)** vehicle dynamics
model in MATLAB/Simulink for analyzing:

- **Roll stability** under combined lateral and longitudinal load transfers.
- **Lateral dynamics** (yaw rate, side-slip angle, wheel loads).
- **Active Anti-Roll Bar (AARB) control** strategies to mitigate roll-over risk.

Four control algorithms are benchmarked side-by-side:

| Controller   | Toolbox Required            |
|--------------|-----------------------------|
| PID          | Control System Toolbox      |
| Fuzzy-PID    | Fuzzy Logic Toolbox         |
| LQR          | Control System Toolbox      |
| MPC          | Model Predictive Control Toolbox |

---

## Architecture

```
VehicleDynamicsMBD-/
├── data/                   # Vehicle parameters & lookup tables (.mat, .sldd)
│   ├── vehicle_params.mat
│   └── tire_data.mat
├── models/                 # Core Simulink plant models (.slx, .sldd)
│   ├── vehicle_14dof.slx
│   └── tire_model.slx
├── controllers/            # AARB controller subsystems (.slx, .m)
│   ├── pid_controller.slx
│   ├── fuzzy_pid_controller.slx
│   ├── lqr_controller.m
│   └── mpc_controller.m
├── scenarios/              # Driving scenario definitions (.m, .slx)
│   ├── fishhook.m
│   ├── double_lane_change.m
│   ├── j_turn.m
│   └── step_steer.m
├── validation/             # ISO 19364:2016 acceptance criteria (.m)
│   └── iso19364_checks.m
├── utils/                  # Shared helper functions (.m)
│   └── plot_results.m
├── tests/                  # Unit & regression tests (.m)
│   └── test_vehicle_model.m
├── docs/                   # Additional documentation
│   └── CONTRIBUTING.md
├── results/                # Simulation output (git-ignored)
├── build/                  # Simulink codegen/cache (git-ignored)
├── setup_environment.m     # One-click path & build configuration
├── main_simulation.m       # Entry-point simulation script
├── .gitignore
└── README.md
```

---

## Prerequisites

| Requirement | Version |
|---|---|
| MATLAB | R2020b or later |
| Simulink | Included with MATLAB |
| Control System Toolbox | Any compatible version |
| Fuzzy Logic Toolbox | Required for Fuzzy-PID controller |
| Model Predictive Control Toolbox | Required for MPC controller |

> **Note:** The core 14-DOF plant model and PID/LQR controllers work with
> only MATLAB, Simulink, and the Control System Toolbox.

---

## Quick Start

```matlab
% 1. Open MATLAB and navigate to the repository root
cd('path/to/VehicleDynamicsMBD-')

% 2. Configure the MATLAB path and Simulink build directories
run('setup_environment.m')

% 3. Run the default simulation (Double Lane Change + PID controller)
run('main_simulation.m')
```

### setup_environment.m

`setup_environment.m` performs the following automatically:

- Adds all project subfolders (`models`, `controllers`, `scenarios`,
  `validation`, `utils`, `tests`, `data`) to the MATLAB path.
- Configures Simulink's code-generation and cache directories to point to
  the local `build/` folder via `Simulink.fileGenControl`, keeping generated
  artefacts out of the source tree.

### main_simulation.m

`main_simulation.m` is the top-level entry point. Edit the configuration
variables at the top of the script to choose:

- **Maneuver** – `'fishhook'`, `'dlc'`, `'j_turn'`, or `'step_steer'`
- **Controller** – `'pid'`, `'fuzzy_pid'`, `'lqr'`, or `'mpc'`

---

## Validation Maneuvers

All maneuvers target compliance with **ISO 19364:2016**
(*Passenger cars — Vehicle dynamic simulation and validation*):

| Maneuver | ISO Reference | Key Metric |
|---|---|---|
| Fishhook | ISO 19364 §6.3 | Roll rate, lateral acceleration |
| Double Lane Change (DLC) | ISO 19364 §6.4 | Yaw rate, side-slip angle |
| J-Turn | ISO 19364 §6.5 | Steady-state roll angle |
| Step Steer | ISO 19364 §6.6 | Yaw rate response time |

---

## Controllers

### PID
Classic proportional-integral-derivative controller tuned with the
MATLAB Control System Toolbox `pidtune` function.

### Fuzzy-PID
Gain-scheduled PID whose gains are adjusted by a Mamdani fuzzy inference
system based on roll angle and roll-rate error signals.

### LQR
Linear Quadratic Regulator derived from a linearized half-car roll model.
State weighting matrices **Q** and **R** are stored in `data/lqr_params.mat`.

### MPC
Model Predictive Controller using the linearized plant; constraints on
actuator force and vehicle roll angle are enforced at each sampling step.

---

## Contributing

See [docs/CONTRIBUTING.md](docs/CONTRIBUTING.md) for branch naming
conventions, coding style guidelines (MAB / MATLAB Advisory Board for
Simulink, PEP 8 for Python side-scripts), and docstring requirements.

---

## License

Distributed under the MIT License. See [LICENSE](LICENSE) for details.

Copyright © 2026 Vitor Machado de Toledo
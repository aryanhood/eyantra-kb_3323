# Self-Balancing Bot Control & CoppeliaSim (`eYRC KB #3323`)

Control-theory, state-estimation, and simulation modules built for a two-wheeled self-balancing robot during the e-Yantra Robotics Competition (`KB #3323`). This repo pairs symbolic state-space linearization and LQR/PID controller design with IMU complementary filtering and CoppeliaSim actuation scripts (see [`self_balance_robot`](https://github.com/aryanhood/self_balance_robot) for the standalone PID vs. LQR disturbance-rejection benchmark).

---

## Repository Contents

| Component | Path | Description |
|---|---|---|
| Symbolic Linearization & LQR Analysis | `math/linearization_and_lqr_analysis.py`, `math/lqr_math_and_parameters.json` | Equilibrium solver, Jacobian evaluation (`A`, `B`), eigenvalue stability classification, and CARE LQR gain computation (`sympy` + `control`). |
| Plant & PID Configuration | `config/plant.json`, `config/pid.json` | Physical parameters (`m_body = 0.6 kg`, `wheel_radius = 0.03 m`, `dt = 0.01 s`) and tuned PID limits/filter coefficients (`kp = 60.0, ki = 0.02, kd = 2.0, alpha = 0.7`). |
| Runtime Controllers & Sensor Filter | `src/control/lqr/lqr.py`, `src/control/pid/pid.py`, `src/control/filters/complementary_filter.py` | Precomputed state-feedback `LQRController`, anti-windup filtered `PIDController`, and IMU complementary filter (`alpha = 0.98`). |
| Simulation & Logged Traces | `src/simulation/compare_pid_vs_lqr.py`, `logs/closed_loop.csv`, `logs/metrics.json` | Closed-loop playback plotting (`theta` and `u`) and logged metrics (`settling_time = 1.79 s`, `final_theta = 5.72e-5 rad`). |
| CoppeliaSim & C Stub | `external/coppeliasim/pid_balancer_child_script.lua`, `src/embedded/main_embedded_stub.c` | CoppeliaSim Python child script (`sysCall_init` / `sysCall_actuation`) and host C trace stub reading `closed_loop.csv`. |
| Unit Tests | `tests/test_filter.py` | Complementary filter convergence and PID/LQR controller sanity checks. |

---

## Quick Start

```bash
pip install -r requirements.txt
pytest -q
python math/linearization_and_lqr_analysis.py
```

MIT License

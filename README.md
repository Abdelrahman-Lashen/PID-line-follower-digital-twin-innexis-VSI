# 🤖 Line-Following Robot — Digital Twin via Innexis VSI

A **digital twin simulation** of a differential-drive line-following robot built on top of **Innexis™ Virtual System Interconnect (VSI)**. Three lightweight Python clients communicate over the VSI fabric to simulate, control, and visualize the robot's behavior in real time.

---

## 📁 Recommended Repository Name

```
line-follower-digital-twin-vsi
```

---

## 🗂️ Project Structure

```
line-follower-digital-twin-vsi/
│
├── client1_simulator/        # Plant + environment simulation
│   ├── simulator.py          # Robot kinematics, path generation, noise injection
│   └── gateway_config.dt     # VSI digital twin description
│
├── client2_controller/       # PID control logic
│   ├── controller.py         # Lateral & heading error, velocity commands
│   └── pid_tuner.py          # Gain sweep utility (E1)
│
├── client3_visualizer/       # Visualization & logging
│   ├── visualizer.py         # Matplotlib/pygame real-time plot
│   └── kpi_logger.py         # Overshoot, settling time, steady-state error
│
├── experiments/              # Experiment scripts
│   ├── E1_gain_sweep.py
│   ├── E2_curved_path.py
│   ├── E3_noise_disturbance.py
│   └── E4_pd_vs_pid.py
│
├── results/                  # Output plots and KPI tables
├── report/                   # PDF report source
├── screencast/               # 2–3 min demo recording
│
├── digital_twin.dt           # Main VSI digital twin config (vsiBuild input)
├── run_simulation.sh         # One-command launcher (wraps vsiSim)
├── requirements.txt
└── README.md
```

---

## 🏗️ System Architecture

The project follows a **three-client architecture** connected over the Innexis VSI backplane:

| Client | Role | Responsibilities |
|--------|------|-----------------|
| **Client 1** — Simulator | Plant + Environment | Robot kinematics, path generation (straight / curved), sensor noise & disturbances |
| **Client 2** — Controller | PID Control | Minimizes lateral & heading error, outputs velocity commands |
| **Client 3** — Visualizer | Logger & Plotter | Trajectory vs. path plots, KPI logging (overshoot, settling time, steady-state error) |

All three clients communicate over the **VSI Fabric Server** using the IVSI backplane protocol, with time synchronization managed by `vsiSim`.

---

## ⚙️ Prerequisites

- **Innexis™ VSI** (v2026.1 or later) installed and sourced
- Python 3.9+
- NumPy, Matplotlib (required)
- `pygame` (optional — for real-time 2D visualization)

```bash
# Source Innexis VSI environment
source $INNEXIS_VSI_HOME/env_vsi.bash

# Install Python dependencies
pip install -r requirements.txt
```

---

## 🚀 Quick Start

### 1. Build the digital twin

```bash
vsiBuild digital_twin.dt
```

### 2. Run the simulation

```bash
vsiSim digital_twin.dt --run
# or use the convenience wrapper:
bash run_simulation.sh
```

### 3. Run experiments individually

```bash
python experiments/E1_gain_sweep.py
python experiments/E2_curved_path.py
python experiments/E3_noise_disturbance.py
python experiments/E4_pd_vs_pid.py
```

Results (plots + KPI CSVs) are saved to `results/`.

---

## 🧪 Experiments

### E1 — PID Gain Sweep
Choose 3–5 PID gain sets. For each, run multiple random spawns on a **straight path** and report:
- Overshoot
- Settling time
- Steady-state error

### E2 — Curved Path Robustness
Apply the best-performing controller(s) from E1 on a **curved path** (piecewise circular arcs or Bézier chain). Compare metrics and discuss curvature effects.

### E3 — Noise & Disturbance Rejection
Add sensor noise and external disturbances at varying levels. Analyze robustness and report success rate per condition.

### E4 — PD vs PID Ablation
Compare **PD (Kᵢ = 0)** vs full **PID** on a curved path under noise. Discuss the integral term's contribution.

---

## 📊 KPIs Tracked

| Metric | Description |
|--------|-------------|
| Overshoot | Max lateral deviation beyond the reference path |
| Settling time | Time to stay within ±threshold of the path |
| Steady-state error | Mean lateral error once settled |
| Success rate | % of spawns that successfully complete the path |

---

## 🔌 VSI Gateway Architecture

```
[Client 1 — Simulator] ──pose, path──▶ VSI Fabric Server ──▶ [Client 2 — Controller]
                                                           ◀──velocity commands──
[Client 3 — Visualizer] ◀──all signals──────────────────────────────────────────
```

Gateway type used: `Python2DtEthernet` (or `Python2DtCAN` depending on config).

Time synchronization is managed by `vsiSim` using the VSI Route backplane.

---

## 📦 Deliverables

- [x] Source code for all three clients and experiment scripts
- [x] Plots and KPI tables for E1–E4
- [x] 2–3 minute screencast (`screencast/demo.mp4`)
- [x] Report: modeling equations, controller design, experiment results, VSI gateway architecture

---

## 👨‍💻 author

- Abdelrahman Mohamed Abdelhamid Lashen
- Course: Digital Twin "Siemens"
- Supervisors : Dr. Mohamed Abdelsalam & Dr. Mohamed El-Leithy

---

## 📄 References

- *Innexis™ Virtual System Interconnect — Digital Twin Builder and Simulator User Guide*, Siemens, v2026.1
- Project specification: `DT_PROJECT.pdf`

---

## 📝 License

Academic use only. © 2026 — All rights reserved.

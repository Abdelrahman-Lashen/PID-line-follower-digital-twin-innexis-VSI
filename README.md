# 🤖 PID Line Follower Digital Twin — Innexis VSI

A differential-drive robot line-following Digital Twin built on the **Siemens Innexis Virtual System Interconnect (VSI)** platform. The system uses a three-client Python architecture communicating over a virtual CAN backplane.

---

## 📽️ Demo

> 🎥 **Screencast:** https://drive.google.com/file/d/1OKXiDG8rRPoTP-P_9F74_l38nQMLKp04/view?usp=sharing

---

## 🏗️ Architecture

```
┌─────────────────────────────────────────────┐
│         Innexis VSI Fabric (CAN Bus)        │
│                                             │
│  ┌───────────┐  frames 10-16  ┌──────────┐  │
│  │ SIMULATOR │ ─────────────► │CONTROLLER│  │
│  │ Client 0  │ ◄───────────── │ Client 1 │  │
│  │ Port 50101│   frame 20     │ Port50102│  │
│  └───────────┘                └──────────┘  │
│        │ frames 10,11,15,16,20              │
│        ▼                                    │
│  ┌────────────┐                             │
│  │ VISUALIZER │ → KPI logs + plots          │
│  │  Client 2  │                             │
│  │ Port 50103 │                             │
│  └────────────┘                             │
└─────────────────────────────────────────────┘
```

| Client | Role | Port |
|--------|------|------|
| Simulator | Robot kinematics + path + noise | 50101 |
| Controller | PID control law | 50102 |
| Visualizer | KPI logging + trajectory plots | 50103 |

---

## 📁 Repository Structure

```
PID-line-follower-digital-twin-innexis-VSI/
│
├── src/
│   ├── simulator/simulator.py      # Client 0 — robot physics + path
│   ├── controller/controller.py    # Client 1 — PID control law
│   └── visualizer/visualizer.py    # Client 2 — KPI logging + plots
│
├── standalone/
│   ├── sim_engine.py               # All physics classes (no VSI needed)
│   └── run_simulation.py           # Single run entry point
│
├── experiments/
│   ├── E1_gain_sweep.py            # E1 — 5 gain sets × 5 spawns
│   └── E2_E3_E4.py                 # E2, E3, E4 experiments
│
├── vsi_config/
│   └── vsiBuildCommands            # DTDL script for vsiBuild
│
├── outputs/
│   ├── E1/                         # E1 experiment plots
│   ├── E2/                         # E2 experiment plots
│   ├── E3/                         # E3 experiment plots
│   └── E4/                         # E4 experiment plots
│
├── run_all_experiments.py          # Run all 4 experiments at once
├── report.pdf                      # Full project report
└── README.md
```

---

## ⚙️ Setup on VPC

### 1 — Source VSI environment
```bash
source /data/tools/pave/innexis_home/vsi_2025.2/env_vsi.bash
```

### 2 — Generate the Digital Twin
```bash
cd vsi_config
vsiBuild -f vsiBuildCommands
```

### 3 — Copy clients into generated skeleton
```bash
cp src/simulator/simulator.py   vsi_config/linefollower_dt/src/simulator/simulator.py
cp src/controller/controller.py vsi_config/linefollower_dt/src/controller/controller.py
cp src/visualizer/visualizer.py vsi_config/linefollower_dt/src/visualizer/visualizer.py
```

### 4 — Run the VSI simulation
```bash
cd vsi_config/linefollower_dt
make -f Makefile.client0 sim 2>&1 | sed 's/^/[SIM] /' &
make -f Makefile.client1 sim 2>&1 | sed 's/^/[CTRL] /' &
make -f Makefile.client2 sim 2>&1 | sed 's/^/[VIZ] /' &
sleep 3
echo "run" | vsiSim linefollower_dt.dt
```

---

## 🧪 Run Experiments Locally (no VSI needed)

```bash
cd robot_dt
python3 run_all_experiments.py
```

Results saved to `outputs/E1/`, `outputs/E2/`, `outputs/E3/`, `outputs/E4/`

---

## 📊 Experiment Results

### E1 — PID Gain Sweep (Straight Path)

| Gain Set | Kp/Ki/Kd | Avg Overshoot | Avg Settling | Avg SS Error |
|----------|-----------|--------------|-------------|-------------|
| G1_Low | 0.8/0.05/0.2 | 1.490 m | 13.59 s | 0.0037 m |
| G2_Med | 2.0/0.1/0.5 | 0.761 m | 4.04 s | 0.0015 m |
| **G3_High ★** | **5.0/0.1/0.5** | **0.874 m** | **2.64 s** | **0.0019 m** |
| G4_PD | 2.0/0.0/0.5 | 1.096 m | 4.32 s | 0.0000 m |
| G5_Aggr | 4.0/0.3/1.5 | 1.278 m | 3.78 s | 0.0046 m |

### E2 — Curved Path Robustness

| Gain Set | Avg Settling | Avg SS Error |
|----------|-------------|-------------|
| G2_Med | 29.94 s (never settled) | 0.032 m |
| **G3_High ★** | **1.93 s** | **0.022 m** |
| G5_Aggr | 0.52 s | 0.025 m |

### E3 — Noise Robustness

| Noise Level | σ | p_dist | Avg SS Error |
|------------|---|--------|-------------|
| No Noise | 0.00 m | 0% | 0.0022 m |
| Low | 0.01 m | 0% | 0.0082 m |
| Medium | 0.05 m | 1% | 0.0444 m |
| High | 0.10 m | 2% | 0.0953 m |

### E4 — PD vs PID (Curved Path + Noise)

| Controller | Avg Overshoot | Avg SS Error |
|------------|-------------|-------------|
| PD (Ki=0) | 0.671 m | 0.058 m |
| **PID ★** | **0.555 m** | 0.064 m |

---

## 🔑 VSI Live Simulation KPIs

| Run | Overshoot | Settling Time | SS Error |
|-----|-----------|--------------|---------|
| Run 1 | 1.2681 m | 5.71 s | 0.0015 m |
| **Run 2 ★ (Best)** | **0.4759 m** | **4.03 s** | **0.0022 m** |
| Run 3 (Recorded) | 1.4501 m | 4.76 s | 0.0069 m |

---

## 👨‍💻 Author

**Abdelrahman Mohamed Abdelhamid Lashen**

- 📚 Course: Digital Twin — Siemens
- 👨‍🏫 Supervisors: Dr. Mohamed Abdelsalam & Dr. Mohamed El-Leithy

---

## 📜 License

This project was developed as part of the Siemens Innexis VSI Digital Twin course.

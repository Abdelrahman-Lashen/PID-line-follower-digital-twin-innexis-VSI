import sys, os, random
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from standalone.run_simulation import run_single
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def run_e2():
    print("\n=== E2: Curved Path Robustness ===")
    GAINS = {
        "G2_Med"  : dict(Kp=2.0, Ki=0.1, Kd=0.5),
        "G3_High" : dict(Kp=5.0, Ki=0.1, Kd=0.5),
        "G5_Aggr" : dict(Kp=4.0, Ki=0.3, Kd=1.5),
    }
    out = {}
    for gname, gains in GAINS.items():
        runs = []
        for i in range(3):
            kpis = run_single(
                path_type="curved",
                spawn_y=random.uniform(-1, 1),
                spawn_theta=random.uniform(-0.3, 0.3),
                out_dir=f"outputs/E2/{gname}",
                label=f"spawn{i}",
                save_plots=True,
                **gains
            )
            runs.append(kpis)
            print(f"  {gname} spawn{i}: OS={kpis['overshoot']:.3f} ST={kpis['settling_time']:.2f} SS={kpis['steady_state_error']:.4f}")
        out[gname] = {k: sum(r[k] for r in runs)/len(runs) for k in runs[0]}
    labels = list(out.keys())
    metrics = ["overshoot", "settling_time", "steady_state_error"]
    fig, axes = plt.subplots(1, 3, figsize=(13, 5))
    for ax, m in zip(axes, metrics):
        vals = [out[g][m] for g in labels]
        bars = ax.bar(labels, vals, color=['#4CAF50','#F44336','#9C27B0'])
        ax.set_title(m.replace('_',' ').title())
        ax.grid(True, alpha=0.3)
        for bar, v in zip(bars, vals):
            ax.text(bar.get_x()+bar.get_width()/2, bar.get_height()+0.01, f'{v:.3f}', ha='center', va='bottom', fontsize=9)
    plt.suptitle("E2 — Curved Path KPI Comparison", fontsize=13)
    plt.tight_layout()
    os.makedirs("outputs/E2", exist_ok=True)
    plt.savefig("outputs/E2/E2_kpi_comparison.png", dpi=120)
    plt.close()
    print("  Saved: outputs/E2/E2_kpi_comparison.png")
    return out

def run_e3():
    print("\n=== E3: Noise and Disturbance Rejection ===")
    NOISE = {
        "No_Noise"  : dict(sigma_pos=0.0,  p_dist=0.0),
        "Low_Noise" : dict(sigma_pos=0.01, p_dist=0.0),
        "Med_Noise" : dict(sigma_pos=0.05, p_dist=0.01),
        "High_Noise": dict(sigma_pos=0.10, p_dist=0.02),
    }
    out = {}
    for nname, nparams in NOISE.items():
        runs = []
        for i in range(3):
            kpis = run_single(
                path_type="straight",
                Kp=5.0, Ki=0.1, Kd=0.5,
                spawn_y=random.uniform(-2, 2),
                spawn_theta=random.uniform(-0.4, 0.4),
                out_dir=f"outputs/E3/{nname}",
                label=f"spawn{i}",
                save_plots=True,
                **nparams
            )
            runs.append(kpis)
            print(f"  {nname} spawn{i}: OS={kpis['overshoot']:.3f} SS={kpis['steady_state_error']:.4f}")
        out[nname] = {k: sum(r[k] for r in runs)/len(runs) for k in runs[0]}
    labels = list(out.keys())
    metrics = ["overshoot", "settling_time", "steady_state_error"]
    fig, axes = plt.subplots(1, 3, figsize=(13, 5))
    for ax, m in zip(axes, metrics):
        vals = [out[g][m] for g in labels]
        bars = ax.bar(labels, vals, color=['#2196F3','#4CAF50','#FF9800','#F44336'])
        ax.set_title(m.replace('_',' ').title())
        ax.set_xticklabels(labels, rotation=15, ha='right')
        ax.grid(True, alpha=0.3)
        for bar, v in zip(bars, vals):
            ax.text(bar.get_x()+bar.get_width()/2, bar.get_height()+0.001, f'{v:.3f}', ha='center', va='bottom', fontsize=8)
    plt.suptitle("E3 — Noise & Disturbance Rejection KPI Comparison", fontsize=13)
    plt.tight_layout()
    os.makedirs("outputs/E3", exist_ok=True)
    plt.savefig("outputs/E3/E3_kpi_comparison.png", dpi=120)
    plt.close()
    print("  Saved: outputs/E3/E3_kpi_comparison.png")
    return out

def run_e4():
    print("\n=== E4: PD vs PID Ablation (Curved Path + Noise) ===")
    CONTROLLERS = {
        "PD"  : dict(Kp=2.0, Ki=0.0, Kd=0.5),
        "PID" : dict(Kp=2.0, Ki=0.1, Kd=0.5),
    }
    out = {}
    for cname, gains in CONTROLLERS.items():
        runs = []
        for i in range(3):
            kpis = run_single(
                path_type="curved",
                sigma_pos=0.05, p_dist=0.01,
                spawn_y=random.uniform(-1, 1),
                spawn_theta=random.uniform(-0.3, 0.3),
                out_dir=f"outputs/E4/{cname}",
                label=f"spawn{i}",
                save_plots=True,
                **gains
            )
            runs.append(kpis)
            print(f"  {cname} spawn{i}: OS={kpis['overshoot']:.3f} SS={kpis['steady_state_error']:.4f}")
        out[cname] = {k: sum(r[k] for r in runs)/len(runs) for k in runs[0]}
    labels = list(out.keys())
    metrics = ["overshoot", "settling_time", "steady_state_error"]
    fig, axes = plt.subplots(1, 3, figsize=(11, 5))
    for ax, m in zip(axes, metrics):
        vals = [out[g][m] for g in labels]
        bars = ax.bar(labels, vals, color=['#FF9800','#2196F3'])
        ax.set_title(m.replace('_',' ').title())
        ax.grid(True, alpha=0.3)
        for bar, v in zip(bars, vals):
            ax.text(bar.get_x()+bar.get_width()/2, bar.get_height()+0.001, f'{v:.3f}', ha='center', va='bottom', fontsize=10)
    plt.suptitle("E4 — PD vs PID (Curved Path + Noise)", fontsize=13)
    plt.tight_layout()
    os.makedirs("outputs/E4", exist_ok=True)
    plt.savefig("outputs/E4/E4_pd_vs_pid.png", dpi=120)
    plt.close()
    print("  Saved: outputs/E4/E4_pd_vs_pid.png")
    return out

if __name__ == "__main__":
    run_e2(); run_e3(); run_e4()
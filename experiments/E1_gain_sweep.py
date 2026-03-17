import sys, os, random
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from standalone.run_simulation import run_single
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

GAIN_SETS = {
    "G1_Low"  : dict(Kp=0.8, Ki=0.05, Kd=0.2),
    "G2_Med"  : dict(Kp=2.0, Ki=0.1,  Kd=0.5),
    "G3_High" : dict(Kp=5.0, Ki=0.1,  Kd=0.5),
    "G4_PD"   : dict(Kp=2.0, Ki=0.0,  Kd=0.5),
    "G5_Aggr" : dict(Kp=4.0, Ki=0.3,  Kd=1.5),
}
N_SPAWNS = 5

def run_e1():
    print("\n=== E1: PID Gain Sweep (Straight Path) ===")
    out = {}
    for gname, gains in GAIN_SETS.items():
        runs = []
        for i in range(N_SPAWNS):
            kpis = run_single(
                path_type="straight",
                spawn_y=random.uniform(-2, 2),
                spawn_theta=random.uniform(-0.4, 0.4),
                out_dir=f"outputs/E1/{gname}",
                label=f"spawn{i}",
                save_plots=True,
                **gains
            )
            runs.append(kpis)
            print(f"  {gname} spawn{i}: OS={kpis['overshoot']:.3f} ST={kpis['settling_time']:.2f} SS={kpis['steady_state_error']:.4f}")
        out[gname] = {k: sum(r[k] for r in runs)/len(runs) for k in runs[0]}

    labels = list(out.keys())
    metrics = ["overshoot", "settling_time", "steady_state_error"]
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    for ax, m in zip(axes, metrics):
        vals = [out[g][m] for g in labels]
        bars = ax.bar(labels, vals, color=['#2196F3','#4CAF50','#F44336','#FF9800','#9C27B0'])
        ax.set_title(m.replace('_',' ').title())
        ax.set_xticklabels(labels, rotation=20, ha='right')
        ax.grid(True, alpha=0.3)
        for bar, v in zip(bars, vals):
            ax.text(bar.get_x()+bar.get_width()/2, bar.get_height()+0.01, f'{v:.3f}', ha='center', va='bottom', fontsize=8)
    plt.suptitle("E1 — PID Gain Sweep KPI Comparison (Straight Path)", fontsize=13)
    plt.tight_layout()
    os.makedirs("outputs/E1", exist_ok=True)
    plt.savefig("outputs/E1/E1_kpi_comparison.png", dpi=120)
    plt.close()
    print("  Saved: outputs/E1/E1_kpi_comparison.png")
    return out

if __name__ == "__main__":
    run_e1()
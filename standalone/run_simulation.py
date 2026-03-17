import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sim_engine import StraightLinePath, CurvedPath, DifferentialRobot, SensorNoise, PIDController, VisualizerLogger, compute_kpis

def run_single(
    path_type="straight",
    Kp=2.0, Ki=0.1, Kd=0.5, alpha=0.5,
    sigma_pos=0.0, p_dist=0.0,
    spawn_x=0.0, spawn_y=1.5, spawn_theta=0.2,
    duration=30.0, dt=0.01,
    out_dir="outputs", label="run",
    save_plots=True,
):
    path = CurvedPath() if path_type == "curved" else StraightLinePath()
    robot  = DifferentialRobot(spawn_x, spawn_y, spawn_theta)
    noise  = SensorNoise(sigma_pos, p_dist)
    pid    = PIDController(Kp, Ki, Kd, alpha, dt)
    logger = VisualizerLogger(out_dir=out_dir, label=label)
    omega_cmd = 0.0
    t = 0.0
    for _ in range(int(duration / dt)):
        robot.step(omega_cmd, dt)
        rx, ry, rtheta = noise.corrupt(robot.x, robot.y, robot.theta)
        px, py, tx, ty, _ = path.nearest_point(rx, ry)
        omega_cmd = pid.compute(rx, ry, rtheta, px, py, tx, ty)
        logger.record(t, rx, ry, px, py, omega_cmd)
        t += dt
    if save_plots:
        return logger.finalize()
    else:
        return compute_kpis(logger.lat_errs, logger.times)
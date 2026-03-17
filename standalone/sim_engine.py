import math, random, os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

class StraightLinePath:
    def __init__(self):
        self.length = 100.0
    def nearest_point(self, x, y):
        s = max(0.0, min(x, self.length))
        return s, 0.0, 1.0, 0.0, s

class CurvedPath:
    N_SAMPLES = 400
    def __init__(self):
        self._pts = self._sample()
    @staticmethod
    def _cubic_bezier(p0, p1, p2, p3, t):
        mt = 1.0 - t
        x = mt**3*p0[0] + 3*mt**2*t*p1[0] + 3*mt*t**2*p2[0] + t**3*p3[0]
        y = mt**3*p0[1] + 3*mt**2*t*p1[1] + 3*mt*t**2*p2[1] + t**3*p3[1]
        return x, y
    @staticmethod
    def _cubic_bezier_tangent(p0, p1, p2, p3, t):
        mt = 1.0 - t
        dx = 3*(mt**2*(p1[0]-p0[0]) + 2*mt*t*(p2[0]-p1[0]) + t**2*(p3[0]-p2[0]))
        dy = 3*(mt**2*(p1[1]-p0[1]) + 2*mt*t*(p2[1]-p1[1]) + t**2*(p3[1]-p2[1]))
        norm = math.sqrt(dx*dx + dy*dy) + 1e-9
        return dx/norm, dy/norm
    def _sample(self):
        segs = [((0,0),(5,0),(10,4),(15,4)),((15,4),(20,4),(25,0),(30,0))]
        pts = []
        n = self.N_SAMPLES // 2
        for seg in segs:
            for i in range(n):
                t = i / n
                pts.append((self._cubic_bezier(*seg, t), self._cubic_bezier_tangent(*seg, t)))
        return pts
    def nearest_point(self, rx, ry):
        best_dist = float('inf'); best_i = 0
        for i, (p, _) in enumerate(self._pts):
            d = (p[0]-rx)**2 + (p[1]-ry)**2
            if d < best_dist:
                best_dist = d; best_i = i
        p, tang = self._pts[best_i]
        return p[0], p[1], tang[0], tang[1], best_i / len(self._pts)

class DifferentialRobot:
    WHEEL_BASE = 0.3
    V_BASE = 1.0
    def __init__(self, x0=0.0, y0=0.0, theta0=0.0):
        self.x = x0; self.y = y0; self.theta = theta0
    def step(self, omega_cmd, dt):
        L = self.WHEEL_BASE
        vR = self.V_BASE + omega_cmd * L / 2.0
        vL = self.V_BASE - omega_cmd * L / 2.0
        v = (vR + vL) / 2.0
        w = (vR - vL) / L
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta = (self.theta + w*dt + math.pi) % (2*math.pi) - math.pi

class SensorNoise:
    def __init__(self, sigma_pos=0.0, p_disturbance=0.0):
        self.sigma_pos = sigma_pos
        self.p_disturbance = p_disturbance
    def corrupt(self, x, y, theta):
        nx = x + random.gauss(0, self.sigma_pos) if self.sigma_pos > 0 else x
        ny = y + random.gauss(0, self.sigma_pos) if self.sigma_pos > 0 else y
        if random.random() < self.p_disturbance:
            nx += random.uniform(-0.5, 0.5)
            ny += random.uniform(-0.5, 0.5)
        return nx, ny, theta

class PIDController:
    MAX_OMEGA = 5.0
    def __init__(self, Kp=2.0, Ki=0.1, Kd=0.5, alpha=0.5, dt=0.01):
        self.Kp=Kp; self.Ki=Ki; self.Kd=Kd; self.alpha=alpha; self.dt=dt
        self._integral=0.0; self._prev_err=0.0
    def _wrap(self, angle):
        return (angle + math.pi) % (2*math.pi) - math.pi
    def compute(self, rx, ry, rtheta, px, py, tx, ty):
        e_lat = (rx-px)*ty - (ry-py)*tx
        e_heading = self._wrap(math.atan2(ty, tx) - rtheta)
        e = e_lat + self.alpha * e_heading
        self._integral += e * self.dt
        derivative = (e - self._prev_err) / self.dt
        self._prev_err = e
        u = self.Kp*e + self.Ki*self._integral + self.Kd*derivative
        return max(-self.MAX_OMEGA, min(self.MAX_OMEGA, u))

def compute_kpis(lateral_errors, times, settle_thresh=0.05):
    if not lateral_errors:
        return {"overshoot": 0.0, "settling_time": 0.0, "steady_state_error": 0.0}
    overshoot = max(lateral_errors)
    settling_time = times[-1]
    for i in range(len(lateral_errors)):
        if all(e < settle_thresh for e in lateral_errors[i:]):
            settling_time = times[i]; break
    n_ss = max(1, len(lateral_errors) // 5)
    ss_error = sum(lateral_errors[-n_ss:]) / n_ss
    return {"overshoot": overshoot, "settling_time": settling_time, "steady_state_error": ss_error}

class VisualizerLogger:
    def __init__(self, out_dir="outputs", label="run"):
        self.out_dir = out_dir; self.label = label
        self.robot_xs=[]; self.robot_ys=[]; self.path_xs=[]; self.path_ys=[]
        self.lat_errs=[]; self.omegas=[]; self.times=[]
    def record(self, t, rx, ry, px, py, omega):
        self.robot_xs.append(rx); self.robot_ys.append(ry)
        self.path_xs.append(px); self.path_ys.append(py)
        self.lat_errs.append(math.sqrt((rx-px)**2+(ry-py)**2))
        self.omegas.append(omega); self.times.append(t)
    def finalize(self):
        kpis = compute_kpis(self.lat_errs, self.times)
        prefix = os.path.join(self.out_dir, self.label)
        os.makedirs(self.out_dir, exist_ok=True)
        fig, ax = plt.subplots(figsize=(9,5))
        ax.plot(self.path_xs, self.path_ys, 'k--', linewidth=2, label='Reference Path')
        ax.plot(self.robot_xs, self.robot_ys, 'b-', linewidth=1.5, label='Robot')
        ax.plot(self.robot_xs[0], self.robot_ys[0], 'go', markersize=8, label='Start')
        ax.plot(self.robot_xs[-1], self.robot_ys[-1], 'r^', markersize=8, label='End')
        ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_title(f'Trajectory — {self.label}')
        ax.legend(); ax.grid(True, alpha=0.3); ax.set_aspect('equal')
        plt.tight_layout(); plt.savefig(prefix+'_trajectory.png', dpi=120); plt.close()
        fig, ax = plt.subplots(figsize=(9,4))
        ax.plot(self.times, self.lat_errs, 'r-', linewidth=1.5)
        ax.axhline(0.05, color='gray', linestyle='--', label='Threshold')
        ax.set_xlabel('Time (s)'); ax.set_ylabel('|Lateral Error| (m)')
        ax.set_title(f'Lateral Error — {self.label}')
        ax.legend(); ax.grid(True, alpha=0.3)
        plt.tight_layout(); plt.savefig(prefix+'_error.png', dpi=120); plt.close()
        return kpis


class TrajectoryPlanner:
    def __init__(self, max_velocity, max_acceleration, max_jerk):
        self.V = max_velocity
        self.A = max_acceleration
        self.J = max_jerk


    # ---------- 单步解析控制 ----------
    def step(self, target, p, v, a, dt):
        return 0,0,0,0
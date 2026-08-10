import math

class TrajectoryPlanner:
    def __init__(self, max_velocity, max_acceleration, max_jerk):
        self.V = max_velocity
        self.A = max_acceleration
        self.J = max_jerk

    def _will_hit_velocity_limit(self, v, a):
        dv = a * abs(a) / (2 * self.J)
        if abs(v + dv) >= self.V:
            return True
        return False

    def _brake_distance(self, v, a): # 计算停止时相对位移、整个过程左右位移极值、此策略下当前时刻加加速度方向
        if v * a >= 0:
            utmost_v = v + a * abs(a) / (2 * self.J)
            utmost_a = -self._sign(v) * math.sqrt(abs(utmost_v) * self.J)
            if abs(utmost_a) <= self.A:
                dt1 = abs(utmost_a - a) / self.J
                dp1 = v * dt1 + (1/2) * a * dt1**2 + (1/6) * -self._sign(v) * self.J * dt1**3

                dt2 = abs(utmost_a) / self.J
                dp2 = (1/6) * self._sign(v) * self.J * dt2**3

                dp = dp1 + dp2
                if dp > 0:
                    return dp, (0.0, dp), -self._sign(v)
                else:
                    return dp, (dp, 0.0), -self._sign(v)
            else:
                true_utmost_a = -self._sign(v) * self.A
                dt1 = abs(true_utmost_a - a) / self.J
                dp1 = v * dt1 + (1/2) * a * dt1**2 + (1/6) * -self._sign(v) * self.J * dt1**3
                dv1 = a * dt1 + (1/2) * -self._sign(v) * self.J * dt1**2

                dt3 = self.A / self.J
                dp3 = (1/6) * self._sign(v) * self.J * dt3**3
                dv3 = (1/2) * -self._sign(v) * self.J * dt3**2

                v2 = v + dv1
                v3 = -dv3
                dt2 = abs(v3 - v2) / self.A
                dp2 = v2 * dt2 + (1/2) * true_utmost_a * dt2**2

                dp = dp1 + dp2 + dp3
                if dp > 0:
                    return dp, (0.0, dp), -self._sign(v)
                else:
                    return dp, (dp, 0.0), -self._sign(v)
        else:
            brake_a_dv = a * abs(a) / (2*self.J)
            if v * (v + brake_a_dv) >= 0:
                utmost_a = -self._sign(v) * math.sqrt(abs(v)*self.J + a**2/2)
                if abs(utmost_a) <= self.A:
                    dt1 = abs(utmost_a - a) / self.J
                    dp1 = v * dt1 + (1/2) * a * dt1**2 + (1/6) * -self._sign(v) * self.J * dt1**3

                    dt2 = abs(utmost_a) / self.J
                    dp2 = (1/6) * self._sign(v) * self.J * dt2**3

                    dp = dp1 + dp2
                    if dp > 0:
                        return dp, (0.0, dp), -self._sign(v)
                    else:
                        return dp, (dp, 0.0), -self._sign(v)
                else:
                    true_utmost_a = -self._sign(v) * self.A
                    dt1 = abs(true_utmost_a - a) / self.J
                    dp1 = v * dt1 + (1/2) * a * dt1**2 + (1/6) * -self._sign(v) * self.J * dt1**3
                    dv1 = a * dt1 + (1/2) * -self._sign(v) * self.J * dt1**2

                    dt3 = self.A / self.J
                    dp3 = (1/6) * self._sign(v) * self.J * dt3**3
                    dv3 = (1/2) * -self._sign(v) * self.J * dt3**2

                    v2 = v + dv1
                    v3 = -dv3
                    dt2 = abs(v3 - v2) / self.A
                    dp2 = v2 * dt2 + (1/2) * true_utmost_a * dt2**2

                    dp = dp1 + dp2 + dp3
                    if dp > 0:
                        return dp, (0.0, dp), -self._sign(v)
                    else:
                        return dp, (dp, 0.0), -self._sign(v)
            else:
                dt1 = abs(a) / self.J
                dp1 = v * dt1 + (1/2) * a * dt1**2 + (1/6) * self._sign(v) * self.J * dt1**3
                utmost_dp1_dt = (abs(a) - math.sqrt(a**2 - 2 * abs(v) * self.J)) / self.J
                utmost_dp1 = v * utmost_dp1_dt + (1/2) * a * utmost_dp1_dt**2 + (1/6) * self._sign(v) * self.J * utmost_dp1_dt**3
                dp1_left, dp1_right = min(0.0, dp1, utmost_dp1), max(0.0, dp1, utmost_dp1)


                v2 = v + (1/2) * a * dt1
                a2 = 0.0
                dp2, (dp2_left, dp2_right), _ = self._brake_distance(v2, a2)

                dp = dp1+dp2
                dp_left = min(0.0, dp2_left+dp1, dp1_left)
                dp_right = max(0.0, dp2_right+dp1, dp1_right)

                return dp, (dp_left, dp_right), self._sign(v)


    def _sign(self, x):
        if abs(x) < 1e-12:
            return 0
        elif x < 0:
            return -1
        return 1

    def step(self, target, p, v, a, dt):
        new_p, new_v, new_a, jerk = 0.0, 0.0, 0.0, 0.0

        error = target - p

        if self._will_hit_velocity_limit(v, a):
            jerk = -self._sign(v) * self.J
        else:
            dp, (dp_left, dp_right), jerk_sign = self._brake_distance(v, a)
            jerk = self._sign(error - dp) * self.J


        if jerk * a > 0 and abs(a + jerk * dt) > self.A:
            jerk = 0.0

        new_a = a + jerk * dt
        new_v = v + new_a * dt
        new_p = p + new_v * dt

        return new_p, new_v, new_a, jerk
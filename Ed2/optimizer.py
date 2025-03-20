import fastsim as fsim
import numpy as np
from scipy.optimize import minimize, Bounds

class TrainEnergyOptimizer:
    def __init__(self, distance_m=1000.0, time_s=60.0, max_speed_mps=30.0, max_accel=1.1, control_points=5, veh_id=43):
        self.distance_m = distance_m
        self.time_s = time_s
        self.max_speed_mps = max_speed_mps
        self.max_accel = max_accel
        self.control_points = control_points
        self.veh_id = veh_id

    # 創立速度陣列
    def create_speed_profile(self, control_points, current_position=0, current_speed=0, remaining_time=None):
        if current_position == 0 and current_speed == 0:
            time_s = np.linspace(0, self.time_s, int(self.time_s) + 1)
            points = np.array([0, *control_points, 0])
            t_points = np.linspace(0, self.time_s, len(points))
            speed_mps = np.interp(time_s, t_points, points)
            distance = np.trapz(speed_mps, time_s)
            if distance > 0:
                speed_mps *= self.distance_m / distance
            return time_s, speed_mps
        else:
            remaining_distance = self.distance_m - current_position
            if remaining_distance <= 0:
                return np.array([0]), np.array([0])
            if remaining_time is None:
                current_speed_mps = current_speed / 3.6
                avg_control_speed = np.mean(control_points) if len(control_points) > 0 else 0
                avg_speed = max((current_speed_mps + avg_control_speed) / 2, 1.0)
                remaining_time = remaining_distance / avg_speed
            time_steps = int(remaining_time) + 1
            time_s = np.linspace(0, remaining_time, time_steps)
            points = np.array([current_speed / 3.6, *control_points, 0])
            t_points = np.linspace(0, remaining_time, len(points))
            speed_mps = np.interp(time_s, t_points, points)
            distance = np.trapz(speed_mps, time_s)
            if distance > 0 and abs(distance - remaining_distance) > 1.0:
                speed_mps *= remaining_distance / distance
            return time_s, speed_mps

    # 模擬能耗計算
    def simulate_energy(self, control_points, current_position=0, current_speed=0):
        time_s, speed_mps = self.create_speed_profile(control_points, current_position, current_speed)
        if len(time_s) <= 1:
            return 0.0
        accel = np.diff(speed_mps) / np.diff(time_s)
        if np.max(np.abs(accel)) > self.max_accel:
            return 1e6
        cyc = fsim.cycle.Cycle(
            time_s=time_s,
            mps=speed_mps,
            grade=np.zeros_like(time_s),
            road_type=np.ones_like(time_s),
            name="optimization_cycle" if current_position == 0 else "recommendation_cycle"
        )
        veh = fsim.vehicle.Vehicle.from_vehdb(self.veh_id)
        veh.max_regen = 0.0
        sim = fsim.simdrive.SimDrive(cyc, veh)
        sim.sim_drive()
        energy_consumption = sim.ess_cur_kwh[0] - sim.ess_cur_kwh[-1]
        return energy_consumption

    def optimize(self):
        initial_guess = np.ones(self.control_points) * self.max_speed_mps * 0.5
        bounds = Bounds([0.0] * self.control_points, [self.max_speed_mps] * self.control_points)
        result = minimize(self.simulate_energy, initial_guess, method='SLSQP', bounds=bounds, options={'maxiter': 200})
        time_s, speed_mps = self.create_speed_profile(result.x)
        return {'optimal_time': time_s, 'optimal_speed': speed_mps * 3.6, 'optimal_energy': self.simulate_energy(result.x)}

    def recommend_speed_profile(self, current_position, current_speed):
        remaining_distance = self.distance_m - current_position
        if remaining_distance <= 0:
            return {
                'times': np.array([0]),
                'speeds': np.array([0]),
                'energy': 0.0,
                'control_points': []
            }
        adjusted_control_points = max(2, min(self.control_points, int(remaining_distance / 200) + 1))
        initial_speed_mps = current_speed / 3.6
        initial_guess = np.linspace(initial_speed_mps * 0.8, initial_speed_mps * 0.2, adjusted_control_points)
        bounds = Bounds([0.0] * adjusted_control_points, [self.max_speed_mps] * adjusted_control_points)
        def objective(x):
            return self.simulate_energy(x, current_position, current_speed)
        result = minimize(objective, initial_guess, method='SLSQP', bounds=bounds, options={'maxiter': 100})
        time_s, speed_mps = self.create_speed_profile(result.x, current_position, current_speed)
        speed_kmh = speed_mps * 3.6
        return {
            'times': time_s,
            'speeds': speed_kmh,
            'control_points': result.x * 3.6,
            'energy': objective(result.x)
        }
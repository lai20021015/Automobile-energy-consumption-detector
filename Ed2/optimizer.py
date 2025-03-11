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

    def create_speed_profile(self, control_points):
        time_s = np.linspace(0, self.time_s, int(self.time_s) + 1)
        points = np.array([0, *control_points, 0]) 
        t_points = np.linspace(0, self.time_s, len(points))
        speed_mps = np.interp(time_s, t_points, points)
        distance = np.trapz(speed_mps, time_s)
        if distance > 0:
            speed_mps *= self.distance_m / distance
        return time_s, speed_mps

    def simulate_energy(self, control_points):
        time_s, speed_mps = self.create_speed_profile(control_points)
        accel = np.diff(speed_mps)/np.diff(time_s)
        if np.max(np.abs(accel)) > self.max_accel:
            return 1e6

        cyc = fsim.cycle.Cycle(time_s=time_s, mps=speed_mps, grade=np.zeros_like(time_s))
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
        return {'optimal_time': time_s, 'optimal_speed': speed_mps, 'optimal_energy': self.simulate_energy(result.x)}

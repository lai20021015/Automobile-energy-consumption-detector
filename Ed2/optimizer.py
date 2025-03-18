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

        # 修正: 添加 road_type 和 name 參數
        cyc = fsim.cycle.Cycle(
            time_s=time_s, 
            mps=speed_mps, 
            grade=np.zeros_like(time_s),
            road_type=np.ones_like(time_s),  # 添加 road_type 參數
            name="optimization_cycle"  # 添加 name 參數
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
    
    # 新增: 從當前狀態創建速度剖面
    def create_speed_profile_from_current(self, control_points, current_position, current_speed, remaining_time=None):
        """
        從當前狀態創建速度剖面
        
        Args:
            control_points: 控制點速度值 (m/s)
            current_position: 當前位置 (m)
            current_speed: 當前速度 (km/h)
            remaining_time: 剩餘時間 (s)，如果為None則自動計算
            
        Returns:
            time_s: 時間數組
            speed_mps: 速度數組 (m/s)
        """
        # 計算剩餘距離
        remaining_distance = self.distance_m - current_position
        
        if remaining_distance <= 0:
            return np.array([0]), np.array([0])
        
        # 如果沒有指定剩餘時間，根據平均速度估算
        if remaining_time is None:
            # 當前速度轉換為m/s
            current_speed_mps = current_speed / 3.6
            # 計算控制點的平均速度 (m/s)
            avg_control_speed = np.mean(control_points) if len(control_points) > 0 else 0
            # 平均速度不能為零
            avg_speed = max((current_speed_mps + avg_control_speed) / 2, 1.0)
            # 根據平均速度和剩餘距離估計所需時間
            remaining_time = remaining_distance / avg_speed
        
        # 創建時間陣列
        time_steps = int(remaining_time) + 1
        time_s = np.linspace(0, remaining_time, time_steps)
        
        # 創建速度控制點，包含起點速度和終點速度(0)
        points = np.array([current_speed / 3.6, *control_points, 0]) 
        t_points = np.linspace(0, remaining_time, len(points))
        
        # 根據控制點插值生成完整速度曲線
        speed_mps = np.interp(time_s, t_points, points)
        
        # 計算實際距離
        distance = np.trapz(speed_mps, time_s)
        
        # 調整速度剖面以達到目標距離
        if distance > 0 and abs(distance - remaining_distance) > 1.0:
            speed_mps *= remaining_distance / distance
        
        return time_s, speed_mps
    
    # 新增: 模擬剩餘路程的能耗
    def simulate_remaining_energy(self, control_points, current_position, current_speed):
        """
        模擬從當前狀態到終點的能耗
        
        Args:
            control_points: 控制點速度值 (m/s)
            current_position: 當前位置 (m)
            current_speed: 當前速度 (km/h)
            
        Returns:
            energy_consumption: 能耗值
        """
        # 創建從當前狀態到終點的速度剖面
        time_s, speed_mps = self.create_speed_profile_from_current(
            control_points, current_position, current_speed
        )
        
        if len(time_s) <= 1:
            return 0.0
        
        # 計算加速度並檢查是否超過限制
        accel = np.diff(speed_mps) / np.diff(time_s)
        if np.max(np.abs(accel)) > self.max_accel:
            return 1e6  # 懲罰過大加速度
        
        # 使用FASTSim模擬能耗
        # 修正: 添加 road_type 和 name 參數
        cyc = fsim.cycle.Cycle(
            time_s=time_s, 
            mps=speed_mps, 
            grade=np.zeros_like(time_s),
            road_type=np.ones_like(time_s),  # 添加 road_type 參數
            name="recommendation_cycle"  # 添加 name 參數
        )
        veh = fsim.vehicle.Vehicle.from_vehdb(self.veh_id)
        veh.max_regen = 0.0
        sim = fsim.simdrive.SimDrive(cyc, veh)
        sim.sim_drive()
        
        energy_consumption = sim.ess_cur_kwh[0] - sim.ess_cur_kwh[-1]
        
        return energy_consumption
    
    # 新增: 從當前狀態優化
    def recommend_speed_profile(self, current_position, current_speed):
        """
        從當前狀態優化剩餘路程的速度控制
        
        Args:
            current_position: 當前位置 (m)
            current_speed: 當前速度 (km/h)
            
        Returns:
            dict: 包含優化結果的字典
        """
        # 計算剩餘距離
        remaining_distance = self.distance_m - current_position
        
        if remaining_distance <= 0:
            return {
                'times': np.array([0]),
                'speeds': np.array([0]),
                'energy': 0.0,
                'control_points': []
            }
        
        # 根據剩餘距離調整控制點數量
        adjusted_control_points = max(2, min(self.control_points, int(remaining_distance / 200) + 1))
        
        # 初始猜測：從當前速度逐漸降低到零
        initial_speed_mps = current_speed / 3.6
        initial_guess = np.linspace(initial_speed_mps * 0.8, initial_speed_mps * 0.2, adjusted_control_points)
        
        # 設置速度限制
        bounds = Bounds([0.0] * adjusted_control_points, [self.max_speed_mps] * adjusted_control_points)
        
        # 定義目標函數
        def objective(x):
            return self.simulate_remaining_energy(x, current_position, current_speed)
        
        # 執行優化
        result = minimize(
            objective, 
            initial_guess, 
            method='SLSQP', 
            bounds=bounds, 
            options={'maxiter': 100}
        )
        
        # 獲取優化結果
        time_s, speed_mps = self.create_speed_profile_from_current(
            result.x, current_position, current_speed
        )
        
        # 轉換回 km/h
        speed_kmh = speed_mps * 3.6
        
        return {
            'times': time_s,
            'speeds': speed_kmh,
            'control_points': result.x * 3.6,  # 控制點速度 (km/h)
            'energy': objective(result.x)
        }
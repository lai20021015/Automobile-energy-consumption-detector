import fastsim as fsim
import numpy as np
from scipy.optimize import minimize, Bounds

class TrainEnergyOptimizer:
    def __init__(self, distance_m=1000.0, time_s=60.0, max_speed_mps=30.0, max_accel=1.1, control_points=None, veh_id=43, speed_limits=None):
        self.distance_m = distance_m
        self.time_s = time_s
        self.max_speed_mps = max_speed_mps
        self.max_accel = max_accel
        self.control_points = control_points if control_points is not None else int(distance_m / 100)
        self.veh_id = veh_id
        self.speed_limits = speed_limits

        # 快取車輛對象
        self.cached_vehicle = fsim.vehicle.Vehicle.from_vehdb(self.veh_id)
        self.cached_vehicle.max_regen = 0.0

    # 創立速度陣列
    def create_speed_profile(self, control_points):
        time_s = np.linspace(0, self.time_s, int(self.time_s) + 1)
        points = np.array([0, *control_points, 0]) 
        t_points = np.linspace(0, self.time_s, len(points))
        speed_mps = np.interp(time_s, t_points, points)
        distance = np.trapz(speed_mps, time_s)
        if distance > 0:
            speed_mps *= self.distance_m / distance #調整速度array 達到目標距離
        return time_s, speed_mps
    
    def create_speed_profile_from_current(self, control_points, current_position, current_speed, remaining_time=None):
        # 計算剩餘距離
        remaining_distance = self.distance_m - current_position
        if remaining_distance <= 0:
            return np.array([0]), np.array([0])

        # 如果沒有指定剩餘時間，根據平均速度估算
        if remaining_time is None:
            current_speed_mps = current_speed / 3.6
            avg_control_speed = np.mean(control_points) if len(control_points) > 0 else 0
            avg_speed = max((current_speed_mps + avg_control_speed) / 2, 1.0)
            remaining_time = remaining_distance / avg_speed

        # 創建時間陣列（降低解析度，每 0.5 秒一個步長）
        time_steps = max(int(remaining_time / 0.5), 10)  # 最少 10 個步長
        time_s = np.linspace(0, remaining_time, time_steps)

        # 創建速度控制點，包含起點速度和終點速度(0)
        points = np.array([current_speed / 3.6, *control_points, 0])
        t_points = np.linspace(0, remaining_time, len(points))

        # 根據控制點插值生成完整速度曲線
        speed_mps = np.interp(time_s, t_points, points)

        # 調整速度剖面以達到目標距離
        distance = np.trapz(speed_mps, time_s)
        if distance > 0 and abs(distance - remaining_distance) > 1.0:
            speed_mps *= remaining_distance / distance

        return time_s, speed_mps
    
    # 模擬能耗計算
    def simulate_energy(self, control_points): 
        time_s, speed_mps = self.create_speed_profile(control_points)
        accel = np.diff(speed_mps)/np.diff(time_s)
        if np.max(np.abs(accel)) > self.max_accel:
            return 1e6 #加速過大, 新增penalty項

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
        result = minimize(self.simulate_energy, initial_guess, method='SLSQP', bounds=bounds, options={'maxiter': 20})
        time_s, speed_mps = self.create_speed_profile(result.x)
        
        # 確保初始化屬性
        self.optimal_result = {
            'optimal_time': time_s,
            'optimal_speed': speed_mps * 3.6,
            'optimal_energy': self.simulate_energy(result.x)
        }
        self.time_optimal = time_s
        self.speed_optimal = speed_mps * 3.6
        self.distance_optimal = np.cumsum(speed_mps) / 3.6  # 確保是數組
        return self.optimal_result
    
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
        # 創建從當前狀態到下兩個控制點的速度剖面
        time_s, speed_mps = self.create_speed_profile_from_current(
            control_points, current_position, current_speed
        )

        # 如果速度剖面過短，直接返回零能耗
        if len(time_s) <= 1:
            return 0.0

        # 使用 FASTSim 模擬能耗
        cyc = fsim.cycle.Cycle(
            time_s=time_s,
            mps=speed_mps,
            grade=np.zeros_like(time_s),
            road_type=np.ones_like(time_s),
            name="recommendation_cycle"
        )
        veh = self.cached_vehicle
        sim = fsim.simdrive.SimDrive(cyc, veh)
        sim.sim_drive()

        energy_consumption = sim.ess_cur_kwh[0] - sim.ess_cur_kwh[-1]
        return energy_consumption
    
    def get_next_recommended_speeds(self, current_position, current_speed):
        """
        基於當前位置和速度提供未來兩個控制點的建議速度
        Args:
            current_position: 當前位置（米）
            current_speed: 當前速度（km/h）
        Returns:
            next_two_speeds: 未來兩個控制點的建議速度（km/h）
        """
        # 設置局部最佳化的參數
        control_points = 2  # 只針對下兩個控制點進行最佳化
        remaining_distance = self.distance_m - current_position
        if remaining_distance <= 0:
            return 0, 0  # 已到達終點，無需建議速度

        # 初始猜測速度（從當前速度逐漸減少）
        initial_speed_mps = current_speed / 3.6
        initial_guess = np.linspace(initial_speed_mps, 0, control_points)

        # 設置速度限制
        bounds = Bounds([0.0] * control_points, [self.max_speed_mps] * control_points)

        # 定義目標函數
        def objective(x):
            return self.simulate_remaining_energy(x, current_position, current_speed)

        # 執行局部最佳化
        result = minimize(
            objective,
            initial_guess,
            method='SLSQP',
            bounds=bounds,
            options={'maxiter': 5}
        )

        # 確保優化成功
        if not result.success:
            return current_speed, max(0, current_speed - 10)  # 如果優化失敗，返回當前速度和稍低的速度

        # 獲取優化結果
        speed_mps = result.x
        speed_kmh = speed_mps * 3.6

        # 返回未來兩個控制點的建議速度
        if len(speed_kmh) >= 2:
            return speed_kmh[0], speed_kmh[1]
        elif len(speed_kmh) == 1:
            return speed_kmh[0], 0
        else:
            return 0, 0
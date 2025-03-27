import fastsim as fsim
import numpy as np
from scipy.optimize import minimize, Bounds

class TrainEnergyOptimizer:
    def __init__(self, 
                distance_m=1000.0,   # Travel distance (meters)
                time_s=60.0,         # Travel time (seconds)
                max_speed_mps=30.0,  # Maximum speed (meters/second)
                max_accel=1.1,       # Maximum acceleration (meters/second²)
                control_points=None,  # 控制點數量，設為 None 以根據距離動態計算
                veh_id=43,           # FASTSim車輛ID
                speed_limits=None    # 速限資訊，格式為 [(距離, 速限), ...]
                ):
        self.distance_m = distance_m
        self.time_s = time_s
        self.max_speed_mps = max_speed_mps
        self.max_accel = max_accel
        
        # 根據距離動態計算控制點，每 100 公尺一個點
        if control_points is None:
            self.control_points = max(3, int(distance_m / 100))
        else:
            self.control_points = control_points
            
        self.veh_id = veh_id
        
        # 設定速限資訊
        self.speed_limits = speed_limits or [(0, max_speed_mps)]
        
        # 預先創建車輛對象以提高效率
        self.veh = fsim.vehicle.Vehicle.from_vehdb(self.veh_id)
        self.veh.max_regen = 0.0
        
        # 保存最佳化結果，避免重複計算
        self.optimal_result = None
        self.optimal_time = None
        self.optimal_speed = None
        self.optimal_distance = None
        self.last_recommendation_time = 0  # 用於控制建議更新頻率

    def get_speed_limit_at_position(self, position):
        """
        根據位置獲取當前速限
        
        Args:
            position: 當前位置（公尺）
            
        Returns:
            speed_limit: 當前位置的速限（m/s）
        """
        # 預設為最高速度
        current_limit = self.max_speed_mps
        
        # 尋找適用的速限
        for limit_pos, limit_speed in sorted(self.speed_limits):
            if position >= limit_pos:
                current_limit = limit_speed
            else:
                break
                
        return current_limit

    def create_speed_profile(self, control_points):
        time_s = np.linspace(0, self.time_s, int(self.time_s) + 1)
        points = np.array([0, *control_points, 0]) 
        t_points = np.linspace(0, self.time_s, len(points))
        
        # 基本速度曲線
        speed_mps = np.interp(time_s, t_points, points)
        
        # 應用速限
        if self.speed_limits and len(self.speed_limits) > 1:
            # 首先計算每個時間點對應的距離
            distance = np.zeros_like(time_s)
            for i in range(1, len(time_s)):
                dt = time_s[i] - time_s[i-1]
                avg_speed = (speed_mps[i] + speed_mps[i-1]) / 2
                distance[i] = distance[i-1] + avg_speed * dt
            
            # 調整速度曲線以遵守速限
            for i, d in enumerate(distance):
                speed_mps[i] = min(speed_mps[i], self.get_speed_limit_at_position(d))
        
        # 調整距離以滿足總距離要求
        distance = np.trapz(speed_mps, time_s)
        if distance > 0:
            speed_mps *= self.distance_m / distance
            
        return time_s, speed_mps

    def simulate_energy(self, control_points):
        time_s, speed_mps = self.create_speed_profile(control_points)
        
        # 檢查加速度是否超過限制
        accel = np.diff(speed_mps)/np.diff(time_s)
        if np.max(np.abs(accel)) > self.max_accel:
            return 1e6  # 懲罰超過加速度限制的方案
        
        # 檢查是否超過速限
        for i in range(len(time_s)):
            # 計算當前點的累積距離
            if i == 0:
                curr_distance = 0
            else:
                dt = time_s[i] - time_s[i-1]
                avg_speed = (speed_mps[i] + speed_mps[i-1]) / 2
                curr_distance += avg_speed * dt
                
            # 檢查是否超過速限
            speed_limit = self.get_speed_limit_at_position(curr_distance)
            if speed_mps[i] > speed_limit * 1.05:  # 允許 5% 的誤差
                return 1e6  # 懲罰超過速限的方案

        # 執行能源模擬
        cyc = fsim.cycle.Cycle(time_s=time_s, mps=speed_mps, grade=np.zeros_like(time_s))
        sim = fsim.simdrive.SimDrive(cyc, self.veh)
        sim.sim_drive()
        energy_consumption = sim.ess_cur_kwh[0] - sim.ess_cur_kwh[-1]
        
        # 增加準時性懲罰
        time_penalty = abs(time_s[-1] - self.time_s) * 10  # 偏離目標時間的懲罰
        
        return energy_consumption + time_penalty

    def get_dynamic_speed_recommendation(self, current_speed, remain_distance, current_time=None):
        """
        根據當前速度和剩餘距離動態計算速度建議
        =====>應該要新增speed_limits 
        Args:
            current_speed: 當前速度（km/h）
            remain_distance: 剩餘距離（公尺）
            current_time: 當前已行駛時間（秒），若未提供則基於平均速度估計
            
        Returns:
            recommendation: 字符串，"加速"、"減速"或"維持速度"
            target_speed: 建議目標速度（km/h）
        """
        # 如果尚未優化，先進行優化
        if self.optimal_result is None:
            self.optimize(maxiter=20)
        
        # 將當前速度轉換為 m/s
        current_speed_mps = current_speed / 3.6
        
        # 計算已行駛距離
        traveled_distance = self.distance_m - remain_distance
        
        # 確定當前的位置在最佳路徑中的大致點
        # 計算剩餘距離對應的索引位置
        if self.optimal_distance is None:
            # 計算累積距離
            self.optimal_time = self.optimal_result['optimal_time']
            self.optimal_speed = self.optimal_result['optimal_speed']
            self.optimal_distance = np.zeros_like(self.optimal_time)
            for i in range(1, len(self.optimal_time)):
                dt = self.optimal_time[i] - self.optimal_time[i-1]
                avg_speed = (self.optimal_speed[i] + self.optimal_speed[i-1]) / 2
                self.optimal_distance[i] = self.optimal_distance[i-1] + avg_speed * dt
        
        # 找到當前位置最近的索引
        current_idx = np.searchsorted(self.optimal_distance, traveled_distance)
        if current_idx >= len(self.optimal_speed):
            return "DEC", 0  # 已接近終點，應該減速停車
        
        # 計算理想速度
        ideal_speed_mps = self.optimal_speed[current_idx]
        ideal_speed = ideal_speed_mps * 3.6  # 轉換為 km/h
        
        # 計算實際剩餘時間（如果提供了當前時間）
        if current_time is not None:
            remain_time = self.time_s - current_time
        else:
            # 若未提供當前時間，根據平均速度估計
            avg_speed = current_speed_mps
            if avg_speed > 0:
                remain_time = remain_distance / avg_speed
            else:
                remain_time = self.time_s  # 預設情況
        
        # 檢查當前位置的速限
        current_speed_limit = self.get_speed_limit_at_position(traveled_distance) * 3.6  # 轉換為 km/h

        # 確保建議速度不超過速限 optional 可以關閉
        ideal_speed = min(ideal_speed, current_speed_limit)

        # 判斷當前位置的理想速度與當前速度的差距
        speed_diff = ideal_speed - current_speed
        
        # 計算達到目的地所需的平均速度（m/s）
        required_avg_speed = remain_distance / remain_time if remain_time > 0 else 0
        
        # 綜合考慮當前理想速度和完成行程所需的平均速度
        # 加權因子: 以距離終點的遠近調整權重
        distance_ratio = remain_distance / self.distance_m
        
        # 距離終點越遠，越重視遵循理想曲線；越近則越重視準時到達
        if distance_ratio > 0.7:  # 距離終點還很遠
            # 主要根據理想速度曲線
            if speed_diff > 5:
                recommendation = "ACC"
            elif speed_diff < -5:
                recommendation = "DEC"
            else:
                recommendation = "MAT"
        elif distance_ratio > 0.3:  # 中等距離
            # 平衡考慮理想速度和所需平均速度
            required_avg_speed_kmh = required_avg_speed * 3.6
            adjusted_target = (ideal_speed + required_avg_speed_kmh) / 2
            
            if current_speed < adjusted_target - 5:
                recommendation = "ACC"
            elif current_speed > adjusted_target + 5:
                recommendation = "DEC"
            else:
                recommendation = "MAT"
        else:  # 接近終點
            # 主要確保按時到達
            required_avg_speed_kmh = required_avg_speed * 3.6
            
            if required_avg_speed_kmh > current_speed + 3:
                recommendation = "ACC"
            elif required_avg_speed_kmh < current_speed - 7:  # 提前一些開始減速
                recommendation = "DEC"
            else:
                recommendation = "MAT"
        
        return recommendation, ideal_speed

    def optimize(self, initial_guess=None, maxiter=20):
        """執行優化，但限制迭代次數以提高效率"""
        # 如果已經有優化結果且不需要更新，直接返回
        if self.optimal_result is not None and maxiter <= 20:
            return self.optimal_result
        
        # 初始猜測值
        if initial_guess is None:
            initial_guess = np.ones(self.control_points) * self.max_speed_mps * 0.5
        
        # 設置邊界條件
        bounds = Bounds(
            lb=[0.0] * self.control_points,
            ub=[self.max_speed_mps] * self.control_points
        )
        
        # 執行優化，限制迭代次數
        result = minimize(
            self.simulate_energy,
            initial_guess,
            method='SLSQP',
            bounds=bounds,
            options={'maxiter': maxiter}  # 限制迭代次數
        )
        
        # 產生最終結果
        time_s, speed_mps = self.create_speed_profile(result.x)
        
        # 執行最終模擬以獲得詳細數據
        cyc = fsim.cycle.Cycle(
            time_s=time_s,
            mps=speed_mps,
            grade=np.zeros_like(time_s),
            road_type=np.zeros_like(time_s),
            name="optimal_cycle"
        )
        sim = fsim.simdrive.SimDrive(cyc, self.veh)  # 使用預先創建的車輛對象
        sim.sim_drive()
        
        # 保存優化結果
        self.optimal_result = {
            'optimal_time': time_s,
            'optimal_speed': speed_mps,
            'optimal_energy': sim.ess_cur_kwh[0] - sim.ess_cur_kwh[-1],
            'simulation': sim
        }
        
        return self.optimal_result
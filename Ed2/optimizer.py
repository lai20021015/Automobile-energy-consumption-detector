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
        
        # 每 100 公尺一個點
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
        input:position: 當前位置（公尺）
        Returns:speed_limit: 當前位置的速限（m/s）
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
        """
        模擬能耗(電池電量消耗)
        input: 控制點的速度
        Return: 能源消耗（kWh）
        """
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
        
        return energy_consumption

    def get_dynamic_speed_recommendation(self, current_speed, remain_distance, current_time=None):
        """
        使用隨機森林模型根據當前狀態動態計算最佳速度建議
        input:
            current_speed: 當前速度（km/h）
            remain_distance: 剩餘距離（公尺）
            current_time: 當前已行駛時間（秒）
        Returns:
            recommendation: 字符串，"ACC"、"DEC"或"MAT" [加速減速保持]
            target_speed: 建議目標速度（km/h）
        """

        # 檢查RF模型是否可用
        if not hasattr(self, 'rf_model'):
            import joblib
            self.rf_model = joblib.load("src/models/RFmodel_v1.joblib")
            print("成功載入RF模型")
            
        # 將當前速度轉換為 m/s
        current_speed_ms = current_speed / 3.6
        
        # 計算已行駛距離
        traveled_distance = self.distance_m - remain_distance
        
        remain_time = max(0.1, self.time_s - current_time)  # 避免除以零
        
        # 檢查當前位置的速限
        current_speed_limit = self.get_speed_limit_at_position(traveled_distance)
        
        # 計算理想的平均速度以準時到達
        required_avg_speed_ms = remain_distance / remain_time
        required_avg_speed_kmh = required_avg_speed_ms * 3.6
        
        # 要評估的速度範圍 (根據目前速度和所需平均速度進行調整)
        speed_step = 5  # km/h
        min_eval_speed = max(5, current_speed - 15)  # 最低評估速度
        max_eval_speed = min(current_speed + 15, current_speed_limit * 3.6)  # 最高評估速度
        

        # 停車邏輯 ↓↓↓
        # 特別處理接近終點的情況
        stopping_distance = (current_speed_ms ** 2) / (2 * self.max_accel)
        safe_stopping_distance = stopping_distance * 1.5  # 50%的安全緩衝

        # 如果剩餘距離小於安全停車距離，進入減速模式
        if remain_distance <= safe_stopping_distance:
            # 計算理想減速曲線
            physics_safe_speed_ms = np.sqrt(2 * self.max_accel * remain_distance * 0.7)  # 0.7是安全係數
            physics_safe_speed_kmh = physics_safe_speed_ms * 3.6
            
            # 如果極近終點，強制減速到很低的速度
            if remain_distance < 50: # 單位:米
                return "DEC", min(current_speed, 5.0)  # 最高5km/h
            
            # 一般減速情況
            if current_speed > physics_safe_speed_kmh + 2:
                return "DEC", physics_safe_speed_kmh
        # 停車邏輯結束 ↑↑↑
        
        # 要評估的速度範圍 (根據目前速度和所需平均速度進行調整)
        speed_step = 5  # km/h
        # ... 以下是原有代碼 ...

        # 根據時間進度調整速度範圍
        progress = current_time / self.time_s
        if progress < 0.3:  # 初始階段
            # 傾向於加速，建立動能
            min_eval_speed = max(min_eval_speed, current_speed)
        elif progress > 0.7:  # 接近終點
            # 傾向於減速，準備停車
            max_eval_speed = min(max_eval_speed, required_avg_speed_kmh * 1.2)
        
        # 生成評估速度列表
        eval_speeds_kmh = [min_eval_speed]
        speed = min_eval_speed
        while speed <= max_eval_speed:
            speed += speed_step
            eval_speeds_kmh.append(speed)
        
        # 如果是初始狀態 (速度接近0)，強制評估加速選項
        if current_speed < 5 and remain_distance > 10:
            eval_speeds_kmh = [5, 10, 15, 20]
        
        # 將評估速度轉換為 m/s
        eval_speeds_ms = [s / 3.6 for s in eval_speeds_kmh]
        
        # 確保所有評估速度都不超過速限
        eval_speeds_ms = [min(s, current_speed_limit) for s in eval_speeds_ms]
        
        # 評估每個速度的表現
        scores = []
        for speed_ms in eval_speeds_ms:
            # 使用RF模型預測能耗
            features = [[speed_ms, remain_time, 0.0]]  # [速度(m/s), 時間(s), 坡度]
            energy = self.rf_model.predict(features)[0]
            
            # 計算這個速度是否能夠按時到達目的地
            estimated_arrival_time = remain_distance / speed_ms if speed_ms > 0 else float('inf')
            time_penalty = 0
            
            if estimated_arrival_time > remain_time:
                # 如果這個速度無法按時到達，給予懲罰
                time_penalty = (estimated_arrival_time - remain_time) * 2  # 每延遲1秒懲罰2單位
            
            # 計算綜合得分 (考慮能耗和時間)
            # 較低的分數更好
            score = energy + time_penalty
            scores.append(score)
        
        # 找出得分最低的速度
        if scores:
            best_index = scores.index(min(scores))
            best_speed_kmh = eval_speeds_kmh[best_index]
        else:
            # 如果沒有評估速度 (不太可能發生)，使用所需平均速度
            best_speed_kmh = required_avg_speed_kmh
        
        # 確定建議類型
        speed_diff = best_speed_kmh - current_speed
        
        if speed_diff > 3:
            recommendation = "ACC"
        elif speed_diff < -3:
            recommendation = "DEC"
        else:
            recommendation = "MAT"
        
        return recommendation, best_speed_kmh
    
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
    def calculate_cumulative_energy_based_on_distance(self, optimal_result, mass=0):
        time_s = optimal_result['optimal_time']
        speed_mps = optimal_result['optimal_speed']

        delta_t = np.diff(np.append(0, time_s))
        distance_m = np.cumsum(speed_mps * delta_t)

        grade = 0.0  # 假設平坦

        if hasattr(self, 'rf_model'):
            # 正確地批量組合特徵
            features = np.array([[v, dt, grade] for v, dt in zip(speed_mps, delta_t)])
            instant_energy = self.rf_model.predict(features)  # 預測多筆能耗
        else:
            # Fallback：基本動能模型
            instant_energy = 0.5 * mass * speed_mps**2 * delta_t / 3600000  # 單位換算為 kWh

        cumulative_energy = np.cumsum(instant_energy)

        return cumulative_energy, distance_m
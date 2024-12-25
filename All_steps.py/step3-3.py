import fastsim as fsim
import numpy as np
from scipy.optimize import minimize, Bounds, LinearConstraint
import matplotlib.pyplot as plt
import pandas as pd
from typing import Tuple, List, Dict

class TrainEnergyOptimizer:
    """列車能耗優化器"""
    
    def __init__(self, 
                 distance_m: float = 1000.0,   # 行駛距離(公尺)
                 time_s: float = 60.0,         # 行駛時間(秒)
                 max_speed_mps: float = 30.0,  # 最高速度(公尺/秒)→改成np.array
                 max_accel: float = 1.1,       # 最大加速度(公尺/平方秒)
                 control_points: int = 5,       # 控制點數量
                 veh_id: int = 43              # FASTSim車輛ID
                 ):
        """初始化優化器
        
        Args:
            distance_m: 目標距離(公尺)
            time_s: 目標時間(秒) 
            max_speed_mps: 最高速度限制(公尺/秒) #####改成array (比如說轉彎處需要更慢)
            max_accel: 最大加速度限制(公尺/平方秒)
            control_points: 速度曲線控制點數量
            veh_id: FASTSim車輛ID
        """
        self.distance_m = distance_m
        self.time_s = time_s
        self.max_speed_mps = max_speed_mps
        self.max_accel = max_accel
        self.control_points = control_points
        self.veh_id = veh_id
        
        

    def create_speed_profile(self, control_points: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """根據控制點生成速度曲線
        
        Args:
            control_points: 速度控制點陣列
            
        Returns:
            time_s: 時間陣列
            speed_mps: 速度陣列
        """
        time_s = np.linspace(0, self.time_s, int(self.time_s) + 1)
        
        # 強制起始和終點速度為0
        points = np.array([0, *control_points, 0]) 
        t_points = np.linspace(0, self.time_s, len(points))
        
        # 使用三次樣條進行插值
        speed_mps = np.interp(time_s, t_points, points)
        
        # 調整速度使得總距離等於目標距離
        distance = np.trapz(speed_mps, time_s)
        if distance > 0:
            speed_mps *= self.distance_m/distance
            
        return time_s, speed_mps

    def simulate_energy(self, control_points: np.ndarray) -> float:
        """計算給定速度曲線的能耗
        
        Args:
            control_points: 速度控制點陣列
            
        Returns:
            float: 總能耗值(包含懲罰項)
        """
        try:
            time_s, speed_mps = self.create_speed_profile(control_points)
            
            # 檢查加速度限制
            accel = np.diff(speed_mps)/np.diff(time_s)
            if np.max(np.abs(accel)) > self.max_accel:
                return 1e6
            
            # 建立循環對象
            cyc = fsim.cycle.Cycle(
                time_s=time_s,
                mps=speed_mps,
                grade=np.zeros_like(time_s),
                road_type=np.zeros_like(time_s),
                name="optimized_cycle"
            )
            
            # 建立車輛模型並停用再生煞車
            veh = fsim.vehicle.Vehicle.from_vehdb(self.veh_id)
            veh.max_regen = 0.0
            
            # 運行模擬
            sim = fsim.simdrive.SimDrive(cyc, veh)
            sim.sim_drive()
            
            # 計算能耗
            energy_consumption = sim.ess_cur_kwh[0] - sim.ess_cur_kwh[-1]
            
            # 加入約束條件的懲罰項
            time_penalty = abs(time_s[-1] - self.time_s) * 100
            distance_error = abs(np.trapz(speed_mps, time_s) - self.distance_m)
            speed_penalty = max(0, np.max(speed_mps) - self.max_speed_mps) * 1000
            
            return energy_consumption + time_penalty + distance_error * 100 + speed_penalty
            
        except Exception as e:
            print(f"模擬過程發生錯誤: {e}")
            return 1e6

    def optimize(self) -> Dict:
        """
        輸入限制距離
        輸入固定距離
        得到最小能耗的np.array
        Returns:
            Dict: 優化結果，包含:
                - optimal_time: 最佳時間序列
                - optimal_speed: 最佳速度序列
        """
        # 初始猜測值
        initial_guess = np.ones(self.control_points) * self.max_speed_mps * 0.5
        
        # 設置邊界條件
        bounds = Bounds(
            lb=[0.0] * self.control_points,
            ub=[self.max_speed_mps] * self.control_points
        )
        
        # 執行優化
        result = minimize(
            self.simulate_energy,
            initial_guess,
            method='SLSQP',
            bounds=bounds,
            options={'maxiter': 200}
        )
        
        # 產生最終結果
        time_s, speed_mps = self.create_speed_profile(result.x)
        
        # 執行最終模擬以獲得詳細數據
        veh = fsim.vehicle.Vehicle.from_vehdb(self.veh_id)
        veh.max_regen = 0.0
        cyc = fsim.cycle.Cycle(
            time_s=time_s,
            mps=speed_mps,
            grade=np.zeros_like(time_s),
            road_type=np.zeros_like(time_s),
            name="optimal_cycle"
        )
        sim = fsim.simdrive.SimDrive(cyc, veh)
        sim.sim_drive()
        
        return {
            'optimal_time': time_s,
            'optimal_speed': speed_mps,
            'optimal_energy': sim.ess_cur_kwh[0] - sim.ess_cur_kwh[-1],
            'simulation': sim
        }

    def plot_results(self, results: Dict):
        """繪製優化結果圖表
        Args:
            results: optimize()方法返回的結果字典
        """
        # 設置中文字體
        plt.rcParams['font.sans-serif'] = ['Microsoft JhengHei'] 
        plt.rcParams['axes.unicode_minus'] = False

        time_s = results['optimal_time']
        speed_mps = results['optimal_speed']
        sim = results['simulation']
        
        # 計算加速度
        accel = np.diff(speed_mps)/np.diff(time_s)
        
        # 計算能耗
        energy_consumption = sim.ess_cur_kwh[0] - sim.ess_cur_kwh[:-1]
        
        # 繪製圖表
        plt.figure(figsize=(15, 12))
        
        # 速度曲線
        plt.subplot(411)
        plt.plot(time_s, speed_mps * 3.6, 'b-', label='實際速度')
        plt.ylabel('速度 (公里/小時)')
        plt.title('最佳化速度剖面')
        plt.legend()
        plt.grid(True)
        
        # 加速度曲線
        plt.subplot(412)
        plt.plot(time_s[:-1], accel, 'r-', label='加速度')
        plt.axhline(y=self.max_accel, color='r', linestyle='--', label=f'最大加速度限制')
        plt.axhline(y=-self.max_accel, color='r', linestyle='--')
        plt.ylabel('加速度 (公尺/平方秒)')
        plt.legend()
        plt.grid(True)
        
        # 能耗分析
        plt.subplot(413)
        plt.plot(time_s[:-1], sim.drag_kw[1:], label='空氣阻力損耗')
        plt.plot(time_s[:-1], sim.rr_kw[1:], label='滾動阻力損耗')
        plt.plot(time_s[:-1], sim.accel_kw[1:], label='加速能耗')
        plt.ylabel('功率 (kW)')
        plt.legend()
        plt.grid(True)
        
        # 累積能耗
        plt.subplot(414)
        plt.plot(time_s[:-1], energy_consumption, 'g-', label='累積能耗')
        plt.xlabel('時間 (秒)')
        plt.ylabel('能耗 (kWh)')
        plt.legend()
        plt.grid(True)
        
        plt.tight_layout()
        plt.show()
        
        # 計算各項能耗佔比
        drag_energy = np.trapz(sim.drag_kw[1:], time_s[:-1])/3600  # kWh
        rr_energy = np.trapz(sim.rr_kw[1:], time_s[:-1])/3600     # kWh
        accel_energy = np.trapz(sim.accel_kw[1:], time_s[:-1])/3600  # kWh
        total_energy = abs(results['optimal_energy'])
        
        print("\n能耗分析:")
        print(f"總能耗: {total_energy:.3f} kWh")
        print(f"每公里能耗: {(total_energy/(self.distance_m/1000)):.3f} kWh/km")
        print("\n各項能耗佔比:")
        print(f"空氣阻力損耗: {drag_energy:.3f} kWh ({drag_energy/total_energy*100:.1f}%)")
        print(f"滾動阻力損耗: {rr_energy:.3f} kWh ({rr_energy/total_energy*100:.1f}%)")
        print(f"加速能耗: {accel_energy:.3f} kWh ({accel_energy/total_energy*100:.1f}%)")

def add_validation_methods(TrainEnergyOptimizer):
    """添加驗證方法到TrainEnergyOptimizer類別"""
    
    def calculate_theoretical_minimum(self) -> float:
        """計算理論最小能耗"""
        # 獲取車輛參數
        veh = fsim.vehicle.Vehicle.from_vehdb(self.veh_id)
        
        # 基本參數
        mass_kg = veh.veh_kg
        distance_m = self.distance_m
        time_s = self.time_s
        
        # 1. 計算理想等加速-等減速情況
        accel_time = self.time_s / 3  # 假設加速、巡航(等速)、減速各佔1/3時間
        cruise_time = self.time_s / 3
        decel_time = self.time_s / 3
        
        # 計算巡航速度（使總距離等於目標距離）
        v_cruise = (2 * distance_m) / (2 * cruise_time + accel_time + decel_time)
        
        # 2. 計算各階段能耗
        
        # 2.1 加速階段動能
        accel = v_cruise / accel_time
        kinetic_energy = 0.5 * mass_kg * v_cruise**2  # Joules
        
        # 2.2 空氣阻力功
        air_density = 1.225  # kg/m^3 (理論值))
        drag_coef = veh.drag_coef
        frontal_area = veh.frontal_area_m2
        
        def air_resistance_work(v, t):
            return 0.5 * air_density * drag_coef * frontal_area * v**3 * t
        
        air_work = (
            air_resistance_work(v_cruise/2, accel_time) +  # 加速階段avg
            air_resistance_work(v_cruise, cruise_time) +   # 巡航階段
            air_resistance_work(v_cruise/2, decel_time)    # 減速階段avg
        )
        
        # 2.3 滾動摩擦功
        rr_coef = veh.wheel_rr_coef
        gravity = 9.80665 # m/s^2
        rolling_work = rr_coef * mass_kg * gravity * distance_m
        
        # 總理論最小能耗 (kWh)
        total_energy_j = kinetic_energy + air_work + rolling_work
        total_energy_kwh = total_energy_j / (3.6e6)  # 轉換為kWh
        
        return total_energy_kwh
    
    def sensitivity_analysis(self, 
                           control_points_range: list = [3, 5, 7, 9],
                           initial_guesses: int = 5) -> pd.DataFrame:
        """執行參數敏感度分析
        
        Args:
            control_points_range: 要測試的控制點數量清單
            initial_guesses: 每個控制點數量要測試的初始猜測次數
            
        Returns:
            DataFrame 包含分析結果
        """
        results = []
        
        for n_points in control_points_range:
            self.control_points = n_points
            
            for i in range(initial_guesses):
                # 使用不同的初始猜測
                initial_guess = np.random.uniform(
                    0, 
                    self.max_speed_mps * 0.8, 
                    n_points
                )
                
                # 優化
                try:
                    result = self.optimize(initial_guess=initial_guess)
                    results.append({
                        'control_points': n_points,
                        'initial_guess': i,
                        'energy_consumption': result['optimal_energy'],
                        'converged': True
                    })
                except Exception as e:
                    results.append({
                        'control_points': n_points,
                        'initial_guess': i,
                        'energy_consumption': np.nan,
                        'converged': False
                    })
        
        df = pd.DataFrame(results)
        return df
    
    def validate_solution(self):
        """驗證解決方案的最優性"""
        # 1. 計算理論最小值
        theoretical_min = self.calculate_theoretical_minimum()
        
        # 2. 執行敏感度分析
        sensitivity_df = self.sensitivity_analysis()
        
        # 3. 顯示分析結果
        print("\n=== 解決方案驗證 ===")
        print(f"\n理論最小能耗: {theoretical_min:.3f} kWh")
        
        print("\n參數敏感度分析:")
        print("\n每個控制點數量的最佳結果:")
        best_results = sensitivity_df.groupby('control_points')['energy_consumption'].min()
        print(best_results)
        
        print("\n收斂性分析:")
        convergence = sensitivity_df.groupby('control_points')['converged'].mean() * 100
        print(f"收斂率 (%):")
        print(convergence)
        
        # 4. 繪製分析圖表
        plt.figure(figsize=(12, 6))
        
        plt.subplot(121)
        box_data = [sensitivity_df[sensitivity_df['control_points'] == cp]['energy_consumption'] 
                   for cp in sensitivity_df['control_points'].unique()]
        plt.boxplot(box_data, tick_labels=sensitivity_df['control_points'].unique())
        plt.axhline(y=theoretical_min, color='r', linestyle='--', label='理論最小值')
        plt.xlabel('控制點數量')
        plt.ylabel('能耗 (kWh)')
        plt.title('能耗分布')
        plt.legend()
        
        plt.subplot(122)
        convergence.plot(kind='bar')
        plt.xlabel('控制點數量')
        plt.ylabel('收斂率 (%)')
        plt.title('優化收斂性')
        
        plt.tight_layout()
        plt.show()
        
        return theoretical_min, sensitivity_df

    # 將新方法添加到類別
    TrainEnergyOptimizer.calculate_theoretical_minimum = calculate_theoretical_minimum
    TrainEnergyOptimizer.sensitivity_analysis = sensitivity_analysis
    TrainEnergyOptimizer.validate_solution = validate_solution
    
    return TrainEnergyOptimizer

# 修改主程式
if __name__ == "__main__":
    # 更新優化器類別
    TrainEnergyOptimizer = add_validation_methods(TrainEnergyOptimizer)
    
    # 建立優化器實例 optimizer是一個優化器
    optimizer = TrainEnergyOptimizer( 
        distance_m=1000.0,   # 1公里
        time_s=60.0,        # 60秒
        max_speed_mps=30.0, # 最高速度 108 km/h
        max_accel=1.1,      # 最大加速度 1.1 m/s^2
        control_points=5,   # 5個控制點
        veh_id=43          # 使用預設車輛
    )
    
    # 執行優化
    results = optimizer.optimize()
    
    
    # 繪製結果
    optimizer.plot_results(results)
    
    # 驗證解決方案
    # theoretical_min, sensitivity_df = optimizer.validate_solution()
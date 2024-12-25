import fastsim as fsim
import numpy as np
import matplotlib.pyplot as plt
from typing import List, Dict

class PacingStrategy:
    def __init__(self, name: str, distance_km: float, speed_segments: List[Dict]):
        """
        初始化配速策略
        name: 策略名稱
        distance_km: 總距離(公里)
        speed_segments: 速度段列表，每段包含:
            - speed_kmh: 速度(公里/小時)
            - distance_fraction: 該速度段佔總距離的比例(0-1)
            - accel_rate_mps2: 加速率(可選)
        """
        self.name = name
        self.distance_m = distance_km * 1000
        self.speed_segments = speed_segments
        
        # 驗證距離比例總和是否為1
        total_fraction = sum(seg['distance_fraction'] for seg in speed_segments)
        if not np.isclose(total_fraction, 1.0, rtol=1e-5):
            raise ValueError(f"距離比例總和必須為1，目前為{total_fraction}")

def create_pacing_cycle(strategy: PacingStrategy, dt_s: float = 1.0):
    """
    根據配速策略創建行駛循環
    """
    current_distance = 0.0
    time_points = [0.0]
    speed_points = [0.0]
    
    for segment in strategy.speed_segments:
        segment_distance = strategy.distance_m * segment['distance_fraction']
        target_speed_mps = segment['speed_kmh'] / 3.6
        
        # 獲取當前速度（上一段的結束速度）
        current_speed = speed_points[-1]
        
        # 處理加速階段
        accel_rate = segment.get('accel_rate_mps2', 2.0)  # 默認加速度
        if current_speed != target_speed_mps:
            # 計算達到目標速度所需的時間和距離
            accel_time = abs(target_speed_mps - current_speed) / accel_rate
            accel_distance = abs(0.5 * (target_speed_mps + current_speed) * accel_time)
            
            # 加速階段的時間點和速度點
            t = np.arange(0, accel_time, dt_s)
            if current_speed < target_speed_mps:
                v = current_speed + accel_rate * t
            else:
                v = current_speed - accel_rate * t
            
            time_points.extend(time_points[-1] + t[1:])
            speed_points.extend(v[1:])
            
            current_distance += accel_distance
            segment_distance -= accel_distance
        
        # 恆速階段
        if segment_distance > 0:
            constant_time = segment_distance / target_speed_mps
            t = np.arange(0, constant_time, dt_s)
            time_points.extend(time_points[-1] + t[1:])
            speed_points.extend([target_speed_mps] * (len(t)-1))
    
    return fsim.cycle.Cycle.from_dict({
        'time_s': np.array(time_points),
        'mps': np.array(speed_points),
        'grade': np.zeros_like(time_points),
        'road_type': np.zeros_like(time_points)
    })

def analyze_pacing_strategy(veh, strategy: PacingStrategy):
    """
    分析特定配速策略的能耗
    """
    cycle = create_pacing_cycle(strategy)
    sim = fsim.simdrive.SimDrive(cycle, veh)
    sim.sim_drive()
    
    # 計算各項指標
    total_time = cycle.time_s[-1]
    total_energy = np.trapz(sim.fc_kw_out_ach, cycle.time_s)
    avg_power = np.mean(sim.fc_kw_out_ach)
    total_distance = sum(sim.dist_mi) * 1.60934  # 轉換為公里
    
    # 繪圖
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(8, 8))
    fig.suptitle(f'配速策略分析: {strategy.name}', fontsize=16)
    
    # 速度曲線
    ax1.plot(sim.cyc.time_s, sim.mps_ach * 3.6, 'b-', label='車速')
    ax1.set_ylabel('速度 (公里/小時)')
    ax1.set_xlabel('時間 (秒)')
    ax1.grid(True)
    ax1.legend()
    
    # 空氣阻力功率損耗
    ax2.plot(sim.cyc.time_s, sim.drag_kw, 'r-', label='空氣阻力損耗功率')
    ax2.set_ylabel('功率 (千瓦)')
    ax2.set_xlabel('時間 (秒)')
    ax2.grid(True)
    ax2.legend()
    
    # 引擎輸出功率
    ax3.plot(sim.cyc.time_s, sim.fc_kw_out_ach, 'g-', label='引擎輸出功率')
    ax3.set_ylabel('功率 (千瓦)')
    ax3.set_xlabel('時間 (秒)')
    ax3.grid(True)
    ax3.legend()
    
    plt.tight_layout()
    
    return {
        "總時間(秒)": total_time,
        "總距離(公里)": total_distance,
        "總能耗(千焦)": total_energy,
        "平均功率(千瓦)": avg_power,
        "平均速度(公里/時)": (total_distance/total_time) * 3600
    }

def compare_pacing_strategies():
    """
    比較不同配速策略
    """
    # 設定中文字體
    plt.rcParams['font.sans-serif'] = ['Microsoft JhengHei']
    plt.rcParams['axes.unicode_minus'] = False
    
    # 載入車輛
    veh = fsim.vehicle.Vehicle.from_file('2012_Ford_Fusion.csv')
    
    # 定義不同的配速策略
    strategies = [
        PacingStrategy(
            name="均速策略",
            distance_km=10.0,
            speed_segments=[
                {"speed_kmh": 60, "distance_fraction": 1.0}
            ]
        ),
        PacingStrategy(
            name="變速策略",
            distance_km=10.0,
            speed_segments=[
                {"speed_kmh": 40, "distance_fraction": 0.3},
                {"speed_kmh": 80, "distance_fraction": 0.4},
                {"speed_kmh": 50, "distance_fraction": 0.3}
            ]
        ),
        PacingStrategy(
            name="漸進策略",
            distance_km=10.0,
            speed_segments=[
                {"speed_kmh": 50, "distance_fraction": 0.2},
                {"speed_kmh": 60, "distance_fraction": 0.3},
                {"speed_kmh": 70, "distance_fraction": 0.3},
                {"speed_kmh": 50, "distance_fraction": 0.2}
            ]
        )
    ]
    
    results = {}
    for strategy in strategies:
        results[strategy.name] = analyze_pacing_strategy(veh, strategy)
        plt.show()
    
    # 打印比較結果
    print("\n配速策略比較:")
    metrics = list(next(iter(results.values())).keys())
    for metric in metrics:
        print(f"\n{metric}:")
        for strategy_name, result in results.items():
            print(f"{strategy_name}: {result[metric]:.2f}")

if __name__ == "__main__":
    compare_pacing_strategies()
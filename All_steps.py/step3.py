import fastsim as fsim
import numpy as np
import matplotlib.pyplot as plt

def create_speed_profile(cruise_speed_kmh: float, 
                        distance_km: float = 5.0,
                        accel_rate_mps2: float = 2.0,
                        decel_rate_mps2: float = 2.0) -> dict:
    """
    創建包含加速、巡航、減速三個階段的速度曲線
    
    Parameters:
    cruise_speed_kmh: 巡航速度(公里/時)
    distance_km: 總距離(公里)
    accel_rate_mps2: 加速率(米/秒平方)
    decel_rate_mps2: 減速率(米/秒平方)
    """
    # 轉換單位
    cruise_speed_mps = cruise_speed_kmh / 3.6
    total_distance_m = distance_km * 1000
    
    # 計算加速和減速階段
    accel_time = cruise_speed_mps / accel_rate_mps2
    decel_time = cruise_speed_mps / decel_rate_mps2
    
    # 計算加速和減速距離
    accel_distance = 0.5 * accel_rate_mps2 * accel_time ** 2
    decel_distance = 0.5 * cruise_speed_mps * decel_time
    
    # 計算巡航距離和時間
    cruise_distance = total_distance_m - (accel_distance + decel_distance)
    cruise_time = cruise_distance / cruise_speed_mps
    
    # 確保有足夠的距離進行加速和減速
    if cruise_distance < 0:
        raise ValueError(f"距離太短，無法達到 {cruise_speed_kmh} km/h 的巡航速度")
    
    # 創建時間序列
    dt = 1.0  # 1秒的時間步長
    
    # 加速階段
    t_accel = np.arange(0, accel_time, dt)
    v_accel = accel_rate_mps2 * t_accel
    
    # 巡航階段
    t_cruise = np.arange(accel_time, accel_time + cruise_time, dt)
    v_cruise = np.ones_like(t_cruise) * cruise_speed_mps
    
    # 減速階段
    t_decel = np.arange(accel_time + cruise_time, 
                        accel_time + cruise_time + decel_time, dt)
    v_decel = cruise_speed_mps - decel_rate_mps2 * (t_decel - (accel_time + cruise_time))
    
    # 合併所有階段
    time_s = np.concatenate([t_accel, t_cruise, t_decel])
    speed_mps = np.concatenate([v_accel, v_cruise, v_decel])
    
    return {
        'time_s': time_s,
        'mps': speed_mps,
        'grade': np.zeros_like(time_s),
        'road_type': np.zeros_like(time_s),
        'metrics': {
            'accel_time': accel_time,
            'cruise_time': cruise_time,
            'decel_time': decel_time,
            'accel_distance': accel_distance,
            'cruise_distance': cruise_distance,
            'decel_distance': decel_distance,
            'total_time': accel_time + cruise_time + decel_time,
            'total_distance': total_distance_m
        }
    }

def evaluate_speed_profile(veh: fsim.vehicle.Vehicle, 
                          cruise_speed_kmh: float,
                          distance_km: float = 5.0) -> dict:
    """評估特定巡航速度的能耗表現"""
    try:
        # 創建速度曲線
        profile = create_speed_profile(cruise_speed_kmh, distance_km)
        
        # 創建循環
        cycle = fsim.cycle.Cycle.from_dict({
            'time_s': profile['time_s'],
            'mps': profile['mps'],
            'grade': profile['grade'],
            'road_type': profile['road_type']
        })
        
        # 執行模擬
        sim = fsim.simdrive.SimDrive(cycle, veh)
        sim.sim_drive()
        
        # 計算關鍵指標
        total_energy = np.trapz(sim.fc_kw_out_ach, sim.cyc.time_s)
        
        return {
            "cruise_speed_kmh": cruise_speed_kmh,
            "total_energy": total_energy,
            "energy_per_km": total_energy / distance_km,
            "avg_power": np.mean(sim.fc_kw_out_ach),
            "drag_loss": np.mean(sim.drag_kw),
            "metrics": profile['metrics'],
            "sim": sim
        }
    except ValueError as e:
        return None

def find_optimal_cruise_speed(veh: fsim.vehicle.Vehicle,
                            speed_range: tuple[float, float] = (30, 120),
                            step: float = 5.0,
                            distance_km: float = 5.0) -> dict:
    """在給定範圍內尋找最佳巡航速度"""
    speeds = np.arange(speed_range[0], speed_range[1] + step, step)
    results = []
    
    # 評估每個速度
    for speed in speeds:
        result = evaluate_speed_profile(veh, speed, distance_km)
        if result:  # 只加入有效的結果
            results.append(result)
    
    if not results:
        raise ValueError("沒有找到可行的速度方案")
    
    # 找出能耗最低的速度
    min_energy_result = min(results, key=lambda x: x["energy_per_km"])
    
    # 繪圖
    plt.figure(figsize=(15, 12))
    
    # 1. 能耗分析
    plt.subplot(3, 1, 1)
    plt.plot([r["cruise_speed_kmh"] for r in results],
             [r["energy_per_km"] for r in results],
             'b-', label='每公里能耗')
    plt.axvline(x=min_energy_result["cruise_speed_kmh"], color='r', linestyle='--',
                label=f'最佳巡航速度: {min_energy_result["cruise_speed_kmh"]:.1f} km/h')
    plt.ylabel('能耗 (kJ/km)')
    plt.title('能耗分析')
    plt.grid(True)
    plt.legend()
    
    # 2. 功率分析
    plt.subplot(3, 1, 2)
    plt.plot([r["cruise_speed_kmh"] for r in results],
             [r["drag_loss"] for r in results],
             'g-', label='空氣阻力損耗')
    plt.plot([r["cruise_speed_kmh"] for r in results],
             [r["avg_power"] for r in results],
             'r-', label='平均功率')
    plt.axvline(x=min_energy_result["cruise_speed_kmh"], color='r', linestyle='--')
    plt.ylabel('功率 (kW)')
    plt.title('功率分析')
    plt.grid(True)
    plt.legend()
    
    # 3. 最佳速度的速度時間曲線
    plt.subplot(3, 1, 3)
    opt_sim = min_energy_result["sim"]
    plt.plot(opt_sim.cyc.time_s, opt_sim.mps_ach * 3.6, 'b-', label='實際速度')
    plt.xlabel('時間 (s)')
    plt.ylabel('速度 (km/h)')
    plt.title(f'速度時間曲線 (最佳巡航速度: {min_energy_result["cruise_speed_kmh"]:.1f} km/h)')
    
    # 標記不同階段
    metrics = min_energy_result["metrics"]
    plt.axvspan(0, metrics["accel_time"], alpha=0.2, color='g', label='加速階段')
    plt.axvspan(metrics["accel_time"], 
                metrics["accel_time"] + metrics["cruise_time"],
                alpha=0.2, color='b', label='巡航階段')
    plt.axvspan(metrics["accel_time"] + metrics["cruise_time"],
                metrics["total_time"],
                alpha=0.2, color='r', label='減速階段')
    
    plt.grid(True)
    plt.legend()
    
    plt.tight_layout()
    plt.show()
    
    return min_energy_result

def test_optimal_speed():
    """測試並找出最佳巡航速度"""
    # 設定中文字體
    plt.rcParams['font.sans-serif'] = ['Microsoft JhengHei']
    plt.rcParams['axes.unicode_minus'] = False
    
    # 載入車輛
    veh = fsim.vehicle.Vehicle.from_file('2012_Ford_Fusion.csv')
    
    # 尋找最佳速度
    optimal_result = find_optimal_cruise_speed(
        veh, 
        speed_range=(30, 120),  # 巡航速度範圍：30-120 km/h
        step=5.0,               # 速度間隔：5 km/h
        distance_km=5.0         # 測試距離：5 km
    )
    
    print("\n最佳速度分析結果:")
    print(f"最佳巡航速度: {optimal_result['cruise_speed_kmh']:.1f} km/h")
    print(f"每公里能耗: {optimal_result['energy_per_km']:.2f} kJ/km")
    print(f"平均功率: {optimal_result['avg_power']:.2f} kW")
    print(f"空氣阻力損耗: {optimal_result['drag_loss']:.2f} kW")
    print("\n時間分配:")
    print(f"加速時間: {optimal_result['metrics']['accel_time']:.1f} 秒")
    print(f"巡航時間: {optimal_result['metrics']['cruise_time']:.1f} 秒")
    print(f"減速時間: {optimal_result['metrics']['decel_time']:.1f} 秒")
    print(f"總時間: {optimal_result['metrics']['total_time']:.1f} 秒")

if __name__ == "__main__":
    test_optimal_speed()
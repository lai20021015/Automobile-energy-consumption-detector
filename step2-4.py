import fastsim as fsim
import numpy as np
import matplotlib.pyplot as plt
"""
    創建三階段速度循環
    v1: 第一目標速度(公里/時)
    v2: 第二目標速度(公里/時)
"""
def create_three_stage_cycle(v1_kmh, v2_kmh, total_distance_km=10.0, dt_s=1.0):
    """
    創建三階段速度循環
    v1_kmh: 第一目標速度(公里/時)
    v2_kmh: 第二目標速度(公里/時)
    total_distance_km: 總距離(公里)
    dt_s: 時間步長(秒)
    """
    # 轉換速度單位為 m/s
    v1_mps = v1_kmh / 3.6
    v2_mps = v2_kmh / 3.6
    
    # 設定加速度
    accel_rate = 2.0  # m/s²
    
    # 計算每個階段的距離
    stage_distance = total_distance_km * 1000 / 3  # 每階段距離(米)
    
    # 初始化列表
    times = [0]
    speeds = [0]
    current_time = 0
    
    # 第一階段：加速到v1並保持
    accel_time1 = v1_mps / accel_rate
    t_accel1 = np.arange(dt_s, accel_time1, dt_s)
    v_accel1 = accel_rate * t_accel1
    
    times.extend(current_time + t_accel1)
    speeds.extend(v_accel1)
    current_time = times[-1]
    
    # 第一階段巡航
    cruise_distance1 = stage_distance - (0.5 * v1_mps * accel_time1)
    cruise_time1 = cruise_distance1 / v1_mps
    t_cruise1 = np.arange(dt_s, cruise_time1, dt_s)
    
    times.extend(current_time + t_cruise1)
    speeds.extend([v1_mps] * len(t_cruise1))
    current_time = times[-1]
    
    # 第二階段：變速到v2並保持
    accel_time2 = abs(v2_mps - v1_mps) / accel_rate
    t_accel2 = np.arange(dt_s, accel_time2, dt_s)
    if v2_mps > v1_mps:
        v_accel2 = v1_mps + accel_rate * t_accel2
    else:
        v_accel2 = v1_mps - accel_rate * t_accel2
    
    times.extend(current_time + t_accel2)
    speeds.extend(v_accel2)
    current_time = times[-1]
    
    # 第二階段巡航
    cruise_distance2 = stage_distance - (0.5 * abs(v2_mps * v2_mps - v1_mps * v1_mps) / accel_rate)
    cruise_time2 = cruise_distance2 / v2_mps
    t_cruise2 = np.arange(dt_s, cruise_time2, dt_s)
    
    times.extend(current_time + t_cruise2)
    speeds.extend([v2_mps] * len(t_cruise2))
    current_time = times[-1]
    
    # 第三階段：減速到停止
    decel_time = v2_mps / accel_rate
    t_decel = np.arange(dt_s, decel_time, dt_s)
    v_decel = v2_mps - accel_rate * t_decel
    
    times.extend(current_time + t_decel)
    speeds.extend(v_decel)
    
    # 轉換為numpy數組
    times = np.array(times)
    speeds = np.array(speeds)
    
    # 創建循環字典
    cycle_dict = {
        'time_s': times,
        'mps': speeds,
        'grade': np.zeros_like(times),
        'road_type': np.zeros_like(times)
    }
    
    return fsim.cycle.Cycle.from_dict(cycle_dict)

def simulate_and_analyze(veh, cyc, title="能耗分析"):
    """模擬並分析結果"""
    sim = fsim.simdrive.SimDrive(cyc, veh)
    sim.sim_drive()
    
    # 計算關鍵指標
    total_energy = np.trapz(sim.fc_kw_out_ach, sim.cyc.time_s)
    avg_power = np.mean(sim.fc_kw_out_ach)
    peak_power = np.max(sim.fc_kw_out_ach)
    total_distance = np.sum(sim.dist_mi) * 1.60934  # 轉換為公里
    
    # 繪圖
    fig, axes = plt.subplots(3, 1, figsize=(8, 7))
    fig.suptitle(title, fontsize=16)
    
    # 速度曲線
    axes[0].plot(sim.cyc.time_s, sim.mps_ach * 3.6, 'b-', label='車速')
    axes[0].set_ylabel('速度 (公里/時)')
    axes[0].grid(True)
    axes[0].legend()
    
    # 空氣阻力和引擎功率
    axes[1].plot(sim.cyc.time_s, sim.drag_kw, 'r-', label='空氣阻力損耗')
    axes[1].plot(sim.cyc.time_s, sim.fc_kw_out_ach, 'g-', label='引擎輸出功率')
    axes[1].set_ylabel('功率 (千瓦)')
    axes[1].grid(True)
    axes[1].legend()
    
    # 累積能耗
    energy_cumulative = np.cumsum(sim.fc_kw_out_ach * np.diff(sim.cyc.time_s, prepend=0))
    axes[2].plot(sim.cyc.time_s, energy_cumulative, 'b-', label='累積能耗')
    axes[2].set_ylabel('累積能耗 (千焦)')
    axes[2].set_xlabel('時間 (秒)')
    axes[2].grid(True)
    axes[2].legend()
    
    plt.tight_layout()
    plt.show()
    
    return {
        "總能耗(kJ)": total_energy,
        "平均功率(kW)": avg_power,
        "最大功率(kW)": peak_power,
        "總時間(s)": sim.cyc.time_s[-1],
        "總距離(km)": total_distance
    }

def main():
    # 設定中文字體
    plt.rcParams['font.sans-serif'] = ['Microsoft JhengHei']
    plt.rcParams['axes.unicode_minus'] = False
    
    # 載入車輛
    veh = fsim.vehicle.Vehicle.from_file('2012_Ford_Fusion.csv')
    
    while True:
        try:
            # 使用者輸入
            print("\n請輸入兩個目標速度 (輸入 'q' 退出):")
            user_input = input("第一目標速度 v1 (公里/時) = ")
            if user_input.lower() == 'q':
                break
            v1 = float(user_input)
            
            v2 = float(input("第二目標速度 v2 (公里/時) = "))
            
            # 創建並分析行駛循環
            cyc = create_three_stage_cycle(v1, v2)
            results = simulate_and_analyze(
                veh, cyc, f"三階段速度分析 (v1: {v1:.1f} km/h, v2: {v2:.1f} km/h)")
            
            # 打印結果
            print("\n分析結果:")
            for key, value in results.items():
                print(f"{key}: {value:.2f}")
            
        except ValueError:
            print("請輸入有效的數字!")
        except Exception as e:
            print(f"發生錯誤: {e}")
            import traceback
            traceback.print_exc()
        
        # 是否繼續
        if input("\n是否繼續分析? (y/n): ").lower() != 'y':
            break

if __name__ == "__main__":
    main()
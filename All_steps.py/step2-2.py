import fastsim as fsim
import numpy as np
import matplotlib.pyplot as plt
"""
    2-2:
    設定最大速度、固定距離，進行模擬分析。
"""
def create_custom_cycle(max_speed_kmh: float, distance_km: float, accel_rate_mps2: float = 2.0):
    """
    創建自定義行駛循環
    max_speed_kmh: 最大速度(公里/小時)
    distance_km: 行駛距離(公里)
    accel_rate_mps2: 加速率(米/平方秒)
    """
    max_speed_mps = max_speed_kmh / 3.6
    
    # 計算加速與減速時間和距離
    accel_time = max_speed_mps / accel_rate_mps2
    accel_distance = 0.5 * accel_rate_mps2 * accel_time ** 2
    
    # 計算恆速行駛距離和時間
    total_distance = distance_km * 1000  # 轉換為米
    cruise_distance = total_distance - 2 * accel_distance  # 減去加速和減速的距離
    cruise_time = cruise_distance / max_speed_mps
    
    # 創建時間序列
    dt = 1.0  # 時間步長(秒)
    
    # 加速階段
    t_accel = np.arange(0, accel_time, dt)
    v_accel = accel_rate_mps2 * t_accel
    
    # 恆速階段
    t_cruise = np.arange(accel_time, accel_time + cruise_time, dt)
    v_cruise = np.ones_like(t_cruise) * max_speed_mps
    
    # 減速階段
    t_decel = np.arange(accel_time + cruise_time, accel_time + cruise_time + accel_time, dt)
    v_decel = max_speed_mps - accel_rate_mps2 * (t_decel - (accel_time + cruise_time))
    
    # 合併所有階段
    time_s = np.concatenate([t_accel, t_cruise, t_decel])
    mps = np.concatenate([v_accel, v_cruise, v_decel])
    
    return fsim.cycle.Cycle.from_dict({
        'time_s': time_s,
        'mps': mps,
        'grade': np.zeros_like(time_s),
        'road_type': np.zeros_like(time_s)
    })

def simulate_and_analyze(veh, cycle, title="能耗分析"):
    """
    執行模擬並分析結果
    """
    sim = fsim.simdrive.SimDrive(cycle, veh)
    sim.sim_drive()
    
    # 計算關鍵指標
    total_time = cycle.time_s[-1]
    total_energy = np.trapz(sim.fc_kw_out_ach, sim.cyc.time_s)
    avg_power = np.mean(sim.fc_kw_out_ach)
    total_distance = sum(sim.dist_mi) * 1.60934  # 轉換為公里
    
    # 繪圖
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(7.5, 8))
    fig.suptitle(title, fontsize=16)
    
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
    plt.show()
    
    return {
        "總時間(秒)": total_time,
        "總距離(公里)": total_distance,
        "總能耗(千焦)": total_energy,
        "平均功率(千瓦)": avg_power,
        "平均速度(公里/時)": (total_distance/total_time) * 3600
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
            print("\n請輸入行駛參數 (輸入 'q' 退出):")
            user_input = input("最大速度(公里/時) = ")
            if user_input.lower() == 'q':
                break
            max_speed = float(user_input)
            
            # distance = float(input("行駛距離(公里) = "))
            distance = 5.0 #設定為五公里

            # 創建並分析行駛循環
            cycle = create_custom_cycle(max_speed, distance)
            results = simulate_and_analyze(veh, cycle, 
                                        f"行駛分析 (速度: {max_speed:.1f} km/h, 距離: {distance:.1f} km)")
            
            # 打印結果
            print("\n分析結果:")
            for key, value in results.items():
                print(f"{key}: {value:.2f}")
            
        except ValueError:
            print("請輸入有效的數字!")
        except Exception as e:
            print(f"發生錯誤: {e}")
        
        # 是否繼續
        if input("\n是否繼續分析? (y/n): ").lower() != 'y':
            break

if __name__ == "__main__":
    main()
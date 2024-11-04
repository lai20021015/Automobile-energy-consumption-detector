import fastsim as fsim
import numpy as np
import matplotlib.pyplot as plt

# 建立一個簡單的行駛循環,速度從0逐漸加速到最高速然後減速
def create_simple_cycle(max_speed_mps, duration_s, dt_s=1.0):
    """
    創建一個簡單的加速-減速循環
    max_speed_mps: 最大速度(米/秒)
    duration_s: 總時間(秒) 
    dt_s: 時間間隔(秒)
    """
    time_s = np.arange(0, duration_s, dt_s)
    # 建立一個三角形速度曲線
    half_time = duration_s/2
    mps = np.where(time_s <= half_time, 
                   time_s * max_speed_mps/half_time,
                   max_speed_mps - (time_s-half_time) * max_speed_mps/half_time)
    
    # 建立循環字典
    cycle_dict = {
        'time_s': time_s,
        'mps': mps,
        'grade': np.zeros_like(time_s),  # 平地
        'road_type': np.zeros_like(time_s)  # 一般道路
    }
    
    return fsim.cycle.Cycle.from_dict(cycle_dict)

def main():
    # 載入車輛模型 (這裡用Ford Fusion作為示例)
    veh = fsim.vehicle.Vehicle.from_file('2012_Ford_Fusion.csv')
    
    # 創建測試循環 (最高速度30m/s約108km/h, 持續300秒)
    cyc = create_simple_cycle(max_speed_mps=30, duration_s=20)
    
    # 建立模擬
    sim = fsim.simdrive.SimDrive(cyc, veh)
    
    # 執行模擬
    sim.sim_drive()
    
    # 繪圖
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 5))
    
    # 速度曲線
    ax1.plot(sim.cyc.time_s, sim.mps_ach * 3.6, 'b-', label='Vehicle Speed')
    ax1.set_ylabel('Speed (km/h)')
    ax1.set_xlabel('Time (s)')
    ax1.grid(True)
    ax1.legend()
    
    # 空氣阻力功率損耗
    ax2.plot(sim.cyc.time_s, sim.drag_kw, 'r-', label='Aerodynamic Power Loss')
    ax2.set_ylabel('Power (kW)')
    ax2.set_xlabel('Time (s)')
    ax2.grid(True)
    ax2.legend()
    
    # 總功率需求
    ax3.plot(sim.cyc.time_s, sim.fc_kw_out_ach, 'g-', label='Engine Power Output')
    ax3.set_ylabel('Power (kW)')
    ax3.set_xlabel('Time (s)')
    ax3.grid(True)
    ax3.legend()
    
    plt.tight_layout()
    plt.show()
    
    # 打印一些關鍵參數
    print(f"Vehicle Parameters:")
    print(f"Drag Coefficient: {veh.drag_coef}")
    print(f"Frontal Area: {veh.frontal_area_m2} m²")
    print(f"Vehicle Mass: {veh.veh_kg} kg")

if __name__ == "__main__":
    main()
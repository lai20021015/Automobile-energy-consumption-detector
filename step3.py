import fastsim as fsim
import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt

def create_speed_profile(control_points, T=60, D=1000):
    """
    根據控制點生成速度曲線
    - 確保起始速度為0
    - 確保終點速度為0
    """
    time_s = np.linspace(0, T, T+1)
    
    # 強制起始和終點速度為0
    control_points = np.array([0, *control_points, 0])  # 首尾添加0
    t_points = np.linspace(0, T, len(control_points))
    speed_mps = np.interp(time_s, t_points, control_points)
    
    # 調整速度使得總距離等於目標距離
    distance = np.trapz(speed_mps, time_s)
    speed_mps *= D/distance
    
    return time_s, speed_mps

def simulate_energy(control_points):
    """計算給定速度曲線的能耗"""
    try:
        time_s, speed_mps = create_speed_profile(control_points)
        
        # 檢查加速度限制
        accel = np.diff(speed_mps)/np.diff(time_s)
        if np.max(np.abs(accel)) > MAX_ACCEL:
            return 1e6  # 返回一個很大的懲罰值
        
        # 創建循環對象
        cyc = fsim.cycle.Cycle(
            time_s=time_s,
            mps=speed_mps,
            grade=np.zeros_like(time_s),
            road_type=np.zeros_like(time_s),
            name="optimized_cycle"
        )
        
        # 運行模擬
        veh = fsim.vehicle.Vehicle.from_vehdb(43)  # Tesla Model 3
        sim = fsim.simdrive.SimDrive(cyc, veh)
        sim.sim_drive()
        
        # 計算總能耗
        total_energy = abs(sim.ess_cur_kwh[-1] - sim.ess_cur_kwh[0])
        
        return total_energy
    except:
        return 1e6  # 如果模擬失敗，返回懲罰值

def distance_constraint(control_points):
    """確保總距離為1公里"""
    time_s, speed_mps = create_speed_profile(control_points)
    distance = np.trapz(speed_mps, time_s)
    return abs(distance - 1000)  # 應該接近0

# 設置全局參數
MAX_SPEED = 30  # m/s (約108 km/h)
MIN_SPEED = 0   # m/s
MAX_ACCEL = 2.5 # m/s^2
N_CONTROL = 3   # 中間控制點數量（不包括起始和終點）

# 優化設置
initial_guess = np.linspace(5, 5, N_CONTROL)  # 初始猜測值
bounds = [(MIN_SPEED, MAX_SPEED) for _ in range(N_CONTROL)]  # 速度範圍

# 約束條件
constraints = [
    {'type': 'eq', 'fun': distance_constraint},
]

# 執行優化
result = minimize(
    simulate_energy,
    initial_guess,
    method='SLSQP',
    bounds=bounds,
    constraints=constraints,
    options={'maxiter': 100}
)

# 使用最優控制點生成最終的速度曲線
optimal_time, optimal_speed = create_speed_profile(result.x)

# 計算加速度
accel = np.diff(optimal_speed)/np.diff(optimal_time)

# 模擬最優結果
cyc = fsim.cycle.Cycle(
    time_s=optimal_time,
    mps=optimal_speed,
    grade=np.zeros_like(optimal_time),
    road_type=np.zeros_like(optimal_time),
    name="optimal_cycle"
)
veh = fsim.vehicle.Vehicle.from_vehdb(43)
sim = fsim.simdrive.SimDrive(cyc, veh)
sim.sim_drive()

# 繪製結果
plt.figure(figsize=(10, 8))

# 速度曲線
plt.subplot(411)
plt.plot(optimal_time, optimal_speed * 3.6, 'b-', label='Optimal Speed')
plt.plot(optimal_time, np.ones_like(optimal_time) * (1000/60) * 3.6, '--', label='Constant Speed')
plt.ylabel('Speed (km/h)')
plt.title('Metro Station-to-Station Analysis')
plt.legend()
plt.grid(True)

# 加速度曲線
plt.subplot(412)
plt.plot(optimal_time[:-1], accel, 'r-', label='Acceleration')
plt.axhline(y=MAX_ACCEL, color='r', linestyle='--', label=f'Max Accel ({MAX_ACCEL} m/s²)')
plt.axhline(y=-MAX_ACCEL, color='r', linestyle='--', label=f'Min Accel (-{MAX_ACCEL} m/s²)')
plt.ylabel('Acceleration (m/s²)')
plt.legend()
plt.grid(True)

# 功率曲線
plt.subplot(413)
plt.plot(optimal_time, sim.mc_mech_kw_out_ach, 'g-', label='Motor Power')
plt.ylabel('Power (kW)')
plt.legend()
plt.grid(True)

# 電池能量
plt.subplot(414)
plt.plot(optimal_time, sim.ess_cur_kwh, 'r-', label='Battery Energy')
plt.xlabel('Time (s)')
plt.ylabel('Battery Energy (kWh)')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()

# 輸出結果
print("\nOptimization Results:")
print(f"Initial energy consumption (constant speed): {simulate_energy(initial_guess):.3f} kWh")
print(f"Optimized energy consumption: {simulate_energy(result.x):.3f} kWh")
print("\nOptimal control points (km/h):")
optimal_points = np.array([0, *result.x, 0])
print(optimal_points * 3.6)

# 計算關鍵指標
print("\nKey Metrics:")
print(f"Maximum Speed: {np.max(optimal_speed)*3.6:.1f} km/h")
print(f"Maximum Acceleration: {np.max(accel):.2f} m/s²")
print(f"Maximum Deceleration: {np.min(accel):.2f} m/s²")
print(f"Total Distance: {np.trapz(optimal_speed, optimal_time):.0f} m")
print(f"Total Time: {optimal_time[-1]:.0f} s")  
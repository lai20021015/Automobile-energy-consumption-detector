import fastsim as fsim
import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt

# 設置中文字體
plt.rcParams['font.sans-serif'] = ['Microsoft JhengHei']  # 可以顯示中文
plt.rcParams['axes.unicode_minus'] = False  # 讓負號正常顯示

def create_speed_profile(control_points, T=60, D=1000):
   """
   根據控制點生成速度曲線
   - 確保起始速度為0
   - 確保終點速度為0
   - 總距離1km
   - 總時間約60秒
   """
   time_s = np.linspace(0, T, T+1)
   
   # 強制起始和終點速度為0
   control_points = np.array([0, *control_points, 0])
   t_points = np.linspace(0, T, len(control_points))
   speed_mps = np.interp(time_s, t_points, control_points)
   
   # 調整速度使得總距離等於目標距離
   distance = np.trapz(speed_mps, time_s)
   if distance > 0:  # 避免除以零
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
       energy = abs(sim.ess_cur_kwh[-1] - sim.ess_cur_kwh[0])
       
       # 添加時間限制的懲罰項
       time_penalty = abs(time_s[-1] - 60) * 100  # 偏離60秒的懲罰
       
       return energy + time_penalty
   
   except:
       return 1e6

def distance_constraint(control_points):
   """確保總距離為1公里"""
   time_s, speed_mps = create_speed_profile(control_points)
   distance = np.trapz(speed_mps, time_s)
   return abs(distance - 1000)  # 應該接近0

# 設置全局參數
MAX_SPEED = 30  # m/s (約108 km/h)
MIN_SPEED = 0   # m/s
MAX_ACCEL = 1.1 # m/s^2 (適合捷運的加速度限制)
N_CONTROL = 3   # 中間控制點數量

# 優化設置
initial_guess = np.array([15, 15, 15])  # 初始猜測值
bounds = [(MIN_SPEED, MAX_SPEED) for _ in range(N_CONTROL)]  # 速度範圍

# 約束條件
constraints = [
   {'type': 'eq', 'fun': distance_constraint}
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

# 生成最終速度曲線
optimal_time, optimal_speed = create_speed_profile(result.x)

# 計算加速度
accel = np.diff(optimal_speed)/np.diff(optimal_time)

# 運行最終模擬
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
plt.figure(figsize=(15, 12))

# 速度曲線
plt.subplot(311)
plt.plot(optimal_time, optimal_speed * 3.6, 'b-', label='實際速度')
plt.ylabel('速度 (公里/小時)')
plt.title('最佳化速度剖面')
plt.legend()
plt.grid(True)

# 加速度曲線
plt.subplot(312)
plt.plot(optimal_time[:-1], accel, 'r-', label='加速度')
plt.axhline(y=MAX_ACCEL, color='r', linestyle='--', label=f'最大加速度限制')
plt.axhline(y=-MAX_ACCEL, color='r', linestyle='--', label=f'最大減速度限制')
plt.ylabel('加速度 (公尺/平方秒)')
plt.legend()
plt.grid(True)

# 能耗曲線
plt.subplot(313)
plt.plot(optimal_time, sim.ess_cur_kwh - sim.ess_cur_kwh[0], 'g-', label='能源消耗')
plt.xlabel('時間 (秒)')
plt.ylabel('能源消耗 (千瓦時)')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()

# 輸出關鍵指標
print("\n最佳化結果:")
print(f"總距離: {np.trapz(optimal_speed, optimal_time):.0f} 公尺")
print(f"總時間: {optimal_time[-1]:.1f} 秒")
print(f"能源消耗: {abs(sim.ess_cur_kwh[-1] - sim.ess_cur_kwh[0]):.3f} 千瓦時")
print(f"最高速度: {np.max(optimal_speed)*3.6:.1f} 公里/小時")
print(f"最大加速度: {np.max(accel):.2f} 公尺/平方秒")
print(f"最大減速度: {np.min(accel):.2f} 公尺/平方秒")
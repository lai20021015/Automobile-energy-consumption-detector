import fastsim as fsim
import numpy as np
import matplotlib.pyplot as plt

# 1. 創建行駛循環 (1公里距離)
time_s = np.arange(0, 121)  # 120秒
speed_mps = np.ones(len(time_s)) * (1000/120)  # 1000公尺/120秒 ≈ 8.33 m/s
road_type = np.zeros(len(time_s))
grade = np.zeros(len(time_s))

# 2. 創建循環對象
cyc = fsim.cycle.Cycle(
    time_s=time_s,
    mps=speed_mps,
    grade=grade,
    road_type=road_type,
    name="1km_cycle"
)

# 3. 載入特斯拉 Model 3
veh = fsim.vehicle.Vehicle.from_vehdb(43)  # Tesla Model 3 RWD

# 4. 創建模擬對象並運行
sim = fsim.simdrive.SimDrive(cyc, veh)  # 注意參數順序：先cycle後vehicle
sim.sim_drive()

# 5. 獲取結果
distance_km = np.array(sim.dist_mi) * 1.60934  # 轉換為公里
energy_kwh = np.array(sim.ess_energy_kwh)  # 累積能量消耗

# 6. 繪製圖表
plt.figure(figsize=(10, 6))
plt.plot(distance_km, energy_kwh, label='Energy Consumption')
plt.xlabel('Distance (km)')
plt.ylabel('Energy Consumption (kWh)')
plt.title('Distance vs Energy Consumption')
plt.legend()
plt.grid(True)
plt.show()

# 7. 輸出最終結果
print(f"Total distance traveled: {distance_km[-1]:.2f} km")
print(f"Total energy consumption: {energy_kwh[-1]:.2f} kWh")
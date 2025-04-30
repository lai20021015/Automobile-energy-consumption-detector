import matplotlib.pyplot as plt
import pygame
from matplotlib.backends.backend_agg import FigureCanvasAgg
import numpy as np

def draw_comparison_graphs(screen, width, vehicle, time_optimal, speed_optimal_time, distance_optimal):
    fig_time, ax_time = plt.subplots(figsize=(5, 3.75), dpi=80)
    ax_time.plot(time_optimal, speed_optimal_time, 'b-', label='optimal speed')
    ax_time.plot(vehicle.time, vehicle.speed, 'ro', label='current state')
    ax_time.set_xlabel('time(s)')
    ax_time.set_ylabel('velocity (km/h)')
    ax_time.set_title('time-velocity comparison')
    ax_time.grid(True)
    ax_time.legend()
    canvas_time = FigureCanvasAgg(fig_time)
    canvas_time.draw()
    renderer_time = canvas_time.get_renderer()
    raw_data_time = renderer_time.tostring_rgb()
    size_time = canvas_time.get_width_height()
    surf_time = pygame.image.fromstring(raw_data_time, size_time, "RGB")
    screen.blit(surf_time, (width - 400, 50))
    plt.close(fig_time)
    
    fig_dist, ax_dist = plt.subplots(figsize=(5, 3.75), dpi=80)
    ax_dist.plot(distance_optimal, speed_optimal_time, 'r-', label='optimal speed')
    ax_dist.plot(vehicle.position, vehicle.speed, 'bo', label='current state')
    ax_dist.set_xlabel('distance (m)')
    ax_dist.set_ylabel('velocity (km/h)')
    ax_dist.set_title('distance-velocity comparison')
    ax_dist.grid(True)
    ax_dist.legend()
    canvas_dist = FigureCanvasAgg(fig_dist)
    canvas_dist.draw()
    renderer_dist = canvas_dist.get_renderer()
    raw_data_dist = renderer_dist.tostring_rgb()
    size_dist = canvas_dist.get_width_height()
    surf_dist = pygame.image.fromstring(raw_data_dist, size_dist, "RGB")
    screen.blit(surf_dist, (width - 400, 350))
    plt.close(fig_dist)

def draw_optimization_results(optimal_result, mass=1000):
    """
    繪製優化結果，包括速度曲線和累積能耗，x 軸為距離
    input: optimal_result - 優化結果字典
    mass: 質量 (kg)，預設 1000 公斤
    """
    time_s = optimal_result['optimal_time']
    speed_mps = optimal_result['optimal_speed']
    
    # 使用模擬結果中的能耗數據
    if 'simulation' in optimal_result:
        sim = optimal_result['simulation']
        if hasattr(sim, 'ess_cur_kwh'):
            # 從模擬結果中提取累積能耗
            initial_energy = sim.ess_cur_kwh[0]
            cumulative_energy = [initial_energy - e for e in sim.ess_cur_kwh]
        else:
            # 如果沒有模擬結果，使用簡化計算
            delta_t = np.diff(np.append(0, time_s))
            instant_energy = 0.5 * mass * speed_mps**2 * delta_t / 3600000  # kWh
            cumulative_energy = np.cumsum(instant_energy)
    else:
        # 如果沒有模擬結果，使用簡化計算
        delta_t = np.diff(np.append(0, time_s))
        instant_energy = 0.5 * mass * speed_mps**2 * delta_t / 3600000  # kWh
        cumulative_energy = np.cumsum(instant_energy)

    # 計算累積距離
    delta_t = np.diff(np.append(0, time_s))
    distance_m = np.cumsum(speed_mps * delta_t)  # 累積距離 (m)

    # 繪製速度曲線（x 軸為距離）
    plt.figure(figsize=(10, 6))
    plt.plot(distance_m, speed_mps * 3.6, label="Optimal Speed (km/h)", color="blue")
    plt.xlabel("Distance (m)")
    plt.ylabel("Speed (km/h)")
    plt.title("Optimal Speed Profile (Distance as X-axis)")
    plt.legend()
    plt.grid()
    plt.show()

    # 繪製累積能耗曲線
    plt.figure(figsize=(10, 6))
    
    # 確保長度一致
    if len(cumulative_energy) != len(distance_m):
        # 根據距離重採樣能耗數據
        # 這裡使用最簡單的方法：截斷到較短的長度
        min_len = min(len(cumulative_energy), len(distance_m))
        cumulative_energy = cumulative_energy[:min_len]
        distance_m = distance_m[:min_len]
    
    plt.plot(distance_m, cumulative_energy, label="Cumulative Energy Consumption (kWh)", color="red")
    plt.xlabel("Distance (m)")
    plt.ylabel("Energy Consumption (kWh)")
    plt.title("Cumulative Energy Consumption Over Distance")
    plt.legend()
    plt.grid()
    plt.show()
import matplotlib.pyplot as plt
import pygame
from matplotlib.backends.backend_agg import FigureCanvasAgg
import numpy as np
#導入argument: screen, width, vehicle, time_optimal, speed_optimal_time, distance_optimal
def draw_comparison_graphs(screen, width, vehicle, time_optimal, 
                           speed_optimal_time, distance_optimal,
                           recommended_times=None, recommended_speeds=None):
    """
    繪製比較圖表
    
    Args:
        screen: Pygame螢幕
        width: 螢幕寬度
        vehicle: 車輛物件
        time_optimal: 最佳時間陣列
        speed_optimal_time: 最佳速度陣列
        distance_optimal: 最佳距離陣列
        recommended_times: 推薦時間陣列(可選)
        recommended_speeds: 推薦速度陣列(可選)
    """
    # 時間-速度圖
    fig_time, ax_time = plt.subplots(figsize=(5, 3.75), dpi=80)
    
    # 繪製最佳速度曲線
    ax_time.plot(time_optimal, speed_optimal_time, 'b-', label='initial optimal')
    
    # 如果有推薦速度曲線，也繪製出來
    if recommended_times is not None and recommended_speeds is not None and len(recommended_times) > 0:
        # 將推薦時間調整為相對於當前時間
        shifted_times = recommended_times + vehicle.time
        ax_time.plot(shifted_times, recommended_speeds, 'g-', label='recommended')
    
    # 繪製當前狀態點
    ax_time.plot(vehicle.time, vehicle.speed, 'ro', label='current')
    
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
    
    # 距離-速度圖
    fig_dist, ax_dist = plt.subplots(figsize=(5, 3.75), dpi=80)
    
    # 繪製最佳速度-距離曲線 (取消)
    ax_dist.plot(distance_optimal, speed_optimal_time, 'r-', label='initial optimal')
    
    # 如果有推薦速度曲線，計算對應的距離然後繪製
    if recommended_times is not None and recommended_speeds is not None and len(recommended_times) > 0:
        # 計算推薦速度對應的距離
        speeds_mps = np.array(recommended_speeds) / 3.6
        distances = np.cumsum(speeds_mps * np.diff(np.append(0, recommended_times)))
        recommended_distances = vehicle.position + distances
        
        # 繪製推薦速度-距離曲線
        ax_dist.plot(recommended_distances, recommended_speeds, 'g-', label='recommended')
    
    # 繪製當前狀態點
    ax_dist.plot(vehicle.position, vehicle.speed, 'bo', label='current')
    
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

def draw_opt_graphs(screen, width, vehicle, time_optimal, 
                           speed_optimal_time, distance_optimal):
    """
    繪製比較圖表 (僅顯示最佳圖形)
    
    Args:
        screen: Pygame螢幕
        width: 螢幕寬度
        vehicle: 車輛物件
        time_optimal: 最佳時間陣列
        speed_optimal_time: 最佳速度陣列
        distance_optimal: 最佳距離陣列
    """
    # 時間-速度圖
    fig_time, ax_time = plt.subplots(figsize=(5, 3.75), dpi=80)
    
    # 繪製最佳速度曲線
    ax_time.plot(time_optimal, speed_optimal_time, 'b-', label='initial optimal')
    
    # 繪製當前狀態點
    ax_time.plot(vehicle.time, vehicle.speed, 'ro', label='current')
    
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
    
    # 距離-速度圖
    fig_dist, ax_dist = plt.subplots(figsize=(5, 3.75), dpi=80)
    
    # 繪製最佳速度-距離曲線
    ax_dist.plot(distance_optimal, speed_optimal_time, 'r-', label='initial optimal')
    
    # 繪製當前狀態點
    ax_dist.plot(vehicle.position, vehicle.speed, 'bo', label='current')
    
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

def draw_result(time, energy, speed):
    """
    顯示結果時間與能量消耗 (以大字體顯示)
    
    Args:
        time: 總時間
        energy: 總能量消耗
    """
    fig, ax = plt.subplots(figsize=(6, 4))
    ax.text(0.5, 0.7, f"Total Time:\n{time:.2f} seconds", fontsize=20, ha='center', va='center')
    ax.text(0.5, 0.35, f"Total Energy Consumption:\n{energy:.2f} kWh", fontsize=20, ha='center', va='center')
    ax.text(0.5, 0.1, f"Final speed:\n{speed:.2f} kWh", fontsize=15, ha='center', va='center')
    ax.axis('off')  # 隱藏座標軸
    plt.show()
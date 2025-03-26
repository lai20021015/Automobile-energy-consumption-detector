import pygame
import numpy as np
from vehicle_model import Vehicle
from optimizer import TrainEnergyOptimizer
from visualization import draw_comparison_graphs
import random

pygame.init()
width, height = 1200, 800
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption("車輛加減速最佳化模擬器")
WHITE, BLACK = (255, 255, 255), (0, 0, 0)
GREEN, RED, BLUE = (0, 255, 0), (255, 0, 0), (0, 0, 255)

# 初始化車輛和優化器
vehicle = Vehicle()  # 車輛參數物件
optimizer = TrainEnergyOptimizer()  # 最佳化器

# 獲取初始最佳解
initial_results = optimizer.optimize()
# 利用優化, 獲得 time_optimal 和 speed_optimal_time (最佳時間和速度)
time_optimal, speed_optimal_time = initial_results['optimal_time'], initial_results['optimal_speed']
distance_optimal = np.cumsum(speed_optimal_time / 3.6 * np.diff(np.append(0, time_optimal)))


font = pygame.font.Font(None, 36)

def draw_dashboard():
    screen.fill(WHITE)
    
    # 繪製路徑進度條
    pygame.draw.line(screen, BLACK, (50, height - 100), (width - 50, height - 100), 5)
    car_x = 50 + (width - 100) * (vehicle.position / distance_optimal[-1])
    pygame.draw.rect(screen, BLUE, (car_x - 20, height - 130, 40, 20))
    
    # 顯示車輛參數
    texts = [
        f"time: {vehicle.time:.1f} secs",
        f"velocity: {vehicle.speed:.1f} km/h",
        f"location: {vehicle.position:.1f} m",
        f"energy: {vehicle.energy_consumption:.1f} KW"
    ]
    
    for i, text in enumerate(texts):
        screen.blit(font.render(text, True, BLACK), (50, 50 + i * 50))
    

    next_speed_1, next_speed_2 = optimizer.get_next_recommended_speeds(vehicle.position, vehicle.speed)
    recommend_text = f"recommend speed: {next_speed_1:.1f} km/h, {next_speed_2:.1f} km/h"
    # 顯示推薦速度文字
    screen.blit(font.render(recommend_text, True, GREEN), (50, 300))  # 將文字繪製到畫布上
    # 繪製比較圖
    # draw_comparison_graphs(screen, width, vehicle, time_optimal, speed_optimal_time, distance_optimal, 
                        #   recommended_times, recommended_speeds)

# 主遊戲循環
running = True
clock = pygame.time.Clock()

while running:
    dt = 0.5  # 時間步長
    
    # 處理事件
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    


    # 使用方向鍵控制
    keys = pygame.key.get_pressed()
    acceleration = 2.0 if keys[pygame.K_UP] else (-2.0 if keys[pygame.K_DOWN] else 0)
    
    # 更新車輛狀態
    vehicle.update(dt, acceleration)
    
    # 檢查是否到達終點
    if vehicle.position >= distance_optimal[-1]:
        print(f"到達終點, 總時間: {vehicle.time:.1f} 秒, 能耗: {vehicle.energy_consumption:.1f} kW")
        running = False
    
    # 繪製儀表板 每dt更新1次
    draw_dashboard()
    pygame.display.flip()
    clock.tick(10)

pygame.quit()
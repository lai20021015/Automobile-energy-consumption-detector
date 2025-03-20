import pygame
import numpy as np
from vehicle_model import Vehicle
from optimizer import TrainEnergyOptimizer
from visualization import draw_comparison_graphs, draw_opt_graphs, draw_result

pygame.init()
width, height = 1200, 800
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption("車輛加減速最佳化模擬器")
WHITE, BLACK = (255, 255, 255), (0, 0, 0)
GREEN, RED, BLUE = (0, 255, 0), (255, 0, 0), (0, 0, 255)

# 初始化車輛和優化器
car_image = pygame.image.load('./src/train.png')
car_image = pygame.transform.scale(car_image, (220, 150))  # 調整圖片大小# 匯入圖片
vehicle = Vehicle()  # 車輛參數物件
# optimizer = TrainEnergyOptimizer()  
optimizer = TrainEnergyOptimizer(1000, 60, 50, 2, 50, 43)# 最佳化器
# 獲取初始最佳解
initial_results = optimizer.optimize()
# 利用優化, 獲得 time_optimal 和 speed_optimal_time (最佳時間和速度)
time_optimal, speed_optimal_time = initial_results['optimal_time'], initial_results['optimal_speed']
distance_optimal = np.cumsum(speed_optimal_time / 3.6 * np.diff(np.append(0, time_optimal)))

# 新增: 存儲即時推薦的速度曲線
recommended_times = []
recommended_speeds = []
recommended_control_points = []
last_recommendation_time = 0

# 更新推薦秒數
recommendation_interval = 1  # 每5秒更新一次推薦

def draw_dashboard():
    screen.fill(WHITE)
    
    # 繪製路徑進度條
    pygame.draw.line(screen, BLACK, (50, height - 100), (width - 50, height - 100), 5)
    car_x = 50 + (width - 100) * (vehicle.position / distance_optimal[-1])
    car_y = height - 250
    screen.blit(car_image, (car_x - 20, car_y))
    
    # 顯示車輛參數
    font = pygame.font.Font(None, 36)
    texts = [
        f"time: {vehicle.time:.1f} secs",
        f"velocity: {vehicle.speed:.1f} km/h",
        f"location: {vehicle.position:.1f} m",
        f"energy: {vehicle.energy_consumption:.1f} KW"
    ]
    for i, text in enumerate(texts):
        screen.blit(font.render(text, True, BLACK), (50, 50 + i * 50))
    
    # # 顯示推薦速度
    # if len(recommended_control_points) > 0:
    #     pygame.draw.rect(screen, (230, 230, 230), (50, 250, 300, 50))
    #     recommend_text = f"recommend speed: {recommended_control_points[0]:.1f} km/h"
    #     screen.blit(font.render(recommend_text, True, RED), (60, 260))
    
    #繪製比較圖
    # draw_comparison_graphs(screen, width, vehicle, time_optimal, speed_optimal_time, distance_optimal, 
    #                       recommended_times, recommended_speeds)
    draw_opt_graphs(screen, width, vehicle, time_optimal, speed_optimal_time, distance_optimal)

# 主遊戲循環
running = True
clock = pygame.time.Clock()

while running:
    dt = 1  # 時間步長
    
    # 處理事件
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    
    # # 每隔一定時間更新推薦速度
    # if vehicle.time - last_recommendation_time >= recommendation_interval:
    #     # 根據當前位置和速度獲取推薦
    #     recommendation = optimizer.recommend_speed_profile(vehicle.position, vehicle.speed)
    #     recommended_times = recommendation['times']
    #     recommended_speeds = recommendation['speeds']
    #     recommended_control_points = recommendation['control_points']
    #     last_recommendation_time = vehicle.time
    
    # 使用方向鍵控制
    keys = pygame.key.get_pressed()
    acceleration = 3.0 if keys[pygame.K_UP] else (-5.0 if keys[pygame.K_DOWN] else 0)
    
    # 更新車輛狀態
    vehicle.update(dt, acceleration)
    
    # 檢查是否到達終點
    if vehicle.position >= distance_optimal[-1]:
        print(f"到達終點, 總時間: {vehicle.time:.1f} 秒, 能耗: {vehicle.energy_consumption:.1f} kW")
        
        # 顯示結果圖表
        import matplotlib.pyplot as plt
        draw_result(vehicle.time, vehicle.energy_consumption, vehicle.speed)
        # 等待按鈕結束
        waiting = True
        while waiting:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    waiting = False
        running = False
    
    # 繪製儀表板 每dt更新1次
    draw_dashboard()
    pygame.display.flip()
    clock.tick(20)

pygame.quit()
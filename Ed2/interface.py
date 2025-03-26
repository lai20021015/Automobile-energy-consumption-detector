import pygame
import numpy as np
import random
from vehicle_model import Vehicle
from optimizer import TrainEnergyOptimizer
from visualization import draw_comparison_graphs

pygame.init()
width, height = 1200, 800
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption("車輛加減速最佳化模擬器")
WHITE, BLACK, RED, GREEN = (255, 255, 255), (0, 0, 0), (255, 0, 0), (0, 255, 0)

# 創建車輛和優化器
vehicle = Vehicle()
optimizer = TrainEnergyOptimizer(
    distance_m=1000.0,
    time_s=60.0,
    max_speed_mps=30.0,
    max_accel=1.1,
    control_points=None  # 使用動態計算的控制點數量
)

# 先在背景執行優化，限制迭代次數
results = optimizer.optimize(maxiter=20)
time_optimal, speed_optimal_time = results['optimal_time'], results['optimal_speed']
distance_optimal = np.cumsum(speed_optimal_time) / 3.6

def draw_dashboard():
    screen.fill(WHITE)
    pygame.draw.line(screen, BLACK, (50, height - 100), (width - 50, height - 100), 5)
    
    # 繪製車輛
    car_x = 50 + (width - 100) * (vehicle.position / distance_optimal[-1])
    pygame.draw.rect(screen, (0, 0, 255), (car_x - 20, height - 130, 40, 20))
    
    # 計算剩餘距離
    remain_distance = distance_optimal[-1] - vehicle.position
    
    # 獲取動態建議
    recommendation, target_speed = optimizer.get_dynamic_speed_recommendation(
        current_speed=vehicle.speed, 
        remain_distance=remain_distance,
        current_time=vehicle.time
    )
    
    # 設置建議顏色
    if recommendation == "MAT":
        rec_color = GREEN
    else:
        rec_color = RED
    
    # 基本信息顯示
    font = pygame.font.Font(None, 36)
    texts = [
        f"time: {vehicle.time:.1f} secs",
        f"velocity: {vehicle.speed:.1f} km/h",
        f"location: {vehicle.position:.1f} m",
        f"energy: {vehicle.energy_consumption:.1f} KW",
        f"target speed: {target_speed:.1f} km/h",
        f"recommendation: {recommendation}"
    ]
    
    for i, text in enumerate(texts):
        color = BLACK if i < 4 else rec_color if i == 5 else BLACK
        screen.blit(font.render(text, True, color), (50, 50 + i * 50))
    
    # 顯示箭頭指示
    if abs(vehicle.time % 0.25) < 0.1:  # 每秒顯示一次
        if recommendation == "ACC":
            # 向上箭頭
            pygame.draw.polygon(screen, RED, [(width-250, 70), (width-230, 40), (width-210, 70)])
            pygame.draw.line(screen, RED, (width-230, 70), (width-230, 100), 3)
        elif recommendation == "DEC":
            # 向下箭頭
            pygame.draw.polygon(screen, RED, [(width-250, 70), (width-230, 100), (width-210, 70)])
            pygame.draw.line(screen, RED, (width-230, 40), (width-230, 70), 3)
        else:  # 維持速度
            # 水平箭頭
            pygame.draw.line(screen, GREEN, (width-270, 70), (width-190, 70), 3)
            pygame.draw.polygon(screen, GREEN, [(width-200, 60), (width-190, 70), (width-200, 80)])
            pygame.draw.polygon(screen, GREEN, [(width-260, 60), (width-270, 70), (width-260, 80)])
        
        # 顯示建議文字
        arrow_text = f"RECOMMENDATION: {recommendation}"
        screen.blit(font.render(arrow_text, True, rec_color), (width - 400, 120))
    
    # 繪製比較圖表
    # draw_comparison_graphs(screen, width, vehicle, time_optimal, speed_optimal_time, distance_optimal)

# 遊戲主循環
running, clock = True, pygame.time.Clock()
while running:
    dt = 0.25
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    
    # 處理按鍵輸入
    keys = pygame.key.get_pressed()
    acceleration = 2.0 if keys[pygame.K_UP] else (-2.0 if keys[pygame.K_DOWN] else 0)
    
    # 更新車輛狀態
    vehicle.update(dt, acceleration)
    
    # 檢查是否到達終點
    if vehicle.position >= distance_optimal[-1]:
        print(f"arrived at destination! Total time: {vehicle.time:.1f} s, energy: {vehicle.energy_consumption:.1f} kW")
        running = False
    
    # 繪製儀表板
    draw_dashboard()
    pygame.display.flip()
    clock.tick(10)

pygame.quit()
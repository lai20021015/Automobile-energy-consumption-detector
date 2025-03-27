import pygame
import numpy as np
import random
from vehicle_model import Vehicle
from optimizer import TrainEnergyOptimizer
from visualization import draw_comparison_graphs

pygame.init()
width, height = 1000, 750 
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption("Train Acceleration Optimization Simulator")
WHITE, BLACK, RED, GREEN, BLUE, YELLOW = (255, 255, 255), (0, 0, 0), (255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0)

# Define section speed limits
# Format: [(start_position, speed_limit_m/s), ...]
speed_limits = [
    (0, 10.0),       # 0-300m 限速 25 m/s (90 km/h)
    (400, 5.0),     # 300-600m 限速 16.7 m/s (60 km/h)
    (500, 10.0),     # 600-1000m 限速 25 m/s (90 km/h)
]

# 創建車輛和優化器
vehicle = Vehicle()
optimizer = TrainEnergyOptimizer(
    distance_m=5000.0,
    time_s=120.0,
    max_speed_mps=30.0,
    max_accel=1.1,
    control_points=None,  # 使用動態計算的控制點數量
    speed_limits=speed_limits  # 加入速限設定
)

# 先在背景執行優化，限制迭代次數
results = optimizer.optimize(maxiter=20)
time_optimal, speed_optimal_time = results['optimal_time'], results['optimal_speed']
distance_optimal = np.cumsum(speed_optimal_time) / 3.6

def draw_dashboard():
    screen.fill(WHITE)
    
    # Draw route baseline
    pygame.draw.line(screen, BLACK, (50, height - 100), (width - 50, height - 100), 5)
    
    # Draw speed limit zones
    for i, (start_pos, limit) in enumerate(speed_limits):
        # Calculate end position
        end_pos = speed_limits[i+1][0] if i < len(speed_limits) - 1 else optimizer.distance_m
        
        # Convert to screen coordinates
        start_x = 50 + (width - 100) * (start_pos / optimizer.distance_m)
        end_x = 50 + (width - 100) * (end_pos / optimizer.distance_m)
        
        # Draw colored blocks indicating speed limit zones
        limit_color = (255, 255, 200) if limit >= 25 else (200, 255, 200) if limit >= 16.7 else (255, 200, 200)
        pygame.draw.rect(screen, limit_color, (start_x, height - 130, end_x - start_x, 35))
        
        # Draw speed limit text
        limit_font = pygame.font.Font(None, 24)
        limit_text = f"{int(limit * 3.6)} km/h"
        limit_surf = limit_font.render(limit_text, True, BLACK)
        screen.blit(limit_surf, (start_x + 5, height - 125))
    
    # Draw vehicle
    car_x = 50 + (width - 100) * (vehicle.position / distance_optimal[-1])
    pygame.draw.rect(screen, BLUE, (car_x - 20, height - 130, 40, 20))
    
    # Get current position speed limit
    current_speed_limit = optimizer.get_speed_limit_at_position(vehicle.position) * 3.6  # Convert to km/h
    
    # Calculate remaining distance
    remain_distance = distance_optimal[-1] - vehicle.position
    
    # 計算剩餘距離
    remain_distance = distance_optimal[-1] - vehicle.position
    
    # 使用get_dynamic_speed_recommendation獲取建議
    recommendation, recommended_speed = optimizer.get_dynamic_speed_recommendation(
        current_speed=vehicle.speed,
        remain_distance=remain_distance,
        current_time=vehicle.time
    )
    
    # 設置建議顏色
    if recommendation == "MAT":
        rec_color = YELLOW
    elif recommendation == "ACC":
        rec_color = GREEN
    else:
        rec_color = RED
        
    # 簡化能效計算
    optimal_idx = np.searchsorted(optimizer.optimal_distance, vehicle.position) if optimizer.optimal_distance is not None else 0
    if optimal_idx < len(optimizer.optimal_speed):
        optimal_speed = optimizer.optimal_speed[optimal_idx] * 3.6
        speed_efficiency = 100 - min(100, abs(vehicle.speed - optimal_speed) * 2)
    else:
        speed_efficiency = 70
        
    energy_efficiency = int(speed_efficiency)
    
    # Set recommendation color
    if recommendation == "MAT":
        rec_color = YELLOW
    elif recommendation == "ACC":
        rec_color = GREEN
    else:
        rec_color = RED
    
    # Display basic information
    font = pygame.font.Font(None, 36)
    texts = [
        f"Time: {vehicle.time:.1f} s",
        f"Speed: {vehicle.speed:.1f} km/h",
        f"Position: {vehicle.position:.1f} m",
        f"Energy: {vehicle.energy_consumption:.1f} kWh",
        f"Target Speed: {recommended_speed:.1f} km/h",
        f"Current Limit: {current_speed_limit:.1f} km/h",
        f"Recommendation: {recommendation}",
        f"Efficiency: {energy_efficiency}%"
    ]
    
    for i, text in enumerate(texts):
        color = BLACK if i < 5 or i == 5 else rec_color if i == 6 else BLACK
        screen.blit(font.render(text, True, color), (50, 50 + i * 40))
    
    # Display arrow indicator
    if vehicle.time % 0.5 < 0.25:  # Flash every 0.5 seconds
        if recommendation == "ACC":
            # Up arrow
            pygame.draw.polygon(screen, GREEN, [(width-250, 70), (width-230, 40), (width-210, 70)])
            pygame.draw.line(screen, GREEN, (width-230, 70), (width-230, 100), 3)
        elif recommendation == "DEC":
            # Down arrow
            pygame.draw.polygon(screen, RED, [(width-250, 70), (width-230, 100), (width-210, 70)])
            pygame.draw.line(screen, RED, (width-230, 40), (width-230, 70), 3)
        else:  # Maintain speed
            # Horizontal arrow
            pygame.draw.line(screen, YELLOW, (width-270, 70), (width-190, 70), 3)
            pygame.draw.polygon(screen, YELLOW, [(width-200, 60), (width-190, 70), (width-200, 80)])
            pygame.draw.polygon(screen, YELLOW, [(width-260, 60), (width-270, 70), (width-260, 80)])
        
        # Display recommendation text
        arrow_text = f"Recommendation: {recommendation}"
        screen.blit(font.render(arrow_text, True, rec_color), (width - 350, 120))
    
    # Efficiency indicator
    pygame.draw.rect(screen, BLACK, (width - 350, 180, 300, 30), 2)
    pygame.draw.rect(screen, 
                     (max(0, 255 - energy_efficiency * 2.55), min(255, energy_efficiency * 2.55), 0),
                     (width - 350, 180, energy_efficiency * 3, 30))
    pygame.draw.rect(screen, BLACK, (width - 350 + energy_efficiency * 3, 180, 2, 30))
    
    # Draw comparison charts
    # draw_comparison_graphs(screen, width, vehicle, time_optimal, speed_optimal_time, distance_optimal)

# Game main loop
running, clock = True, pygame.time.Clock()
while running:
    dt = 0.75
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    
    # Process key input
    keys = pygame.key.get_pressed()
    acceleration = 2.0 if keys[pygame.K_UP] else (-2.0 if keys[pygame.K_DOWN] else 0)
    
    # Update vehicle state
    vehicle.update(dt, acceleration)
    
    # Check if destination is reached
    if vehicle.position >= distance_optimal[-1]:
        print(f"Destination reached! Total time: {vehicle.time:.1f} s, Energy: {vehicle.energy_consumption:.1f} kWh")
        running = False
    
    # Draw dashboard
    draw_dashboard()
    pygame.display.flip()
    clock.tick(10)

pygame.quit()
import pygame
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_agg import FigureCanvasAgg
import os

# 初始化Pygame
pygame.init()

width, height = 1200, 800
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption("車輛加減速最佳化模擬器")

# 顏色定義
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
BLUE = (0, 0, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)

# 定義最佳化曲線數據
# 基於您的圖表數據
time_optimal = np.linspace(0, 60, 100)
speed_optimal_time = np.zeros_like(time_optimal)
for i, t in enumerate(time_optimal):
    if t < 10:  # 加速階段
        speed_optimal_time[i] = 7.2 * t  # 假設以7.2 km/h/s加速
    elif t < 50:  # 巡航階段
        speed_optimal_time[i] = 72.0  # 巡航速度72 km/h
    else:  # 減速階段
        speed_optimal_time[i] = 72.0 - 7.2 * (t - 50)  # 以7.2 km/h/s減速

# 計算對應的距離曲線
distance_optimal = np.zeros_like(time_optimal)
for i in range(1, len(time_optimal)):
    dt = time_optimal[i] - time_optimal[i-1]
    avg_speed = (speed_optimal_time[i] + speed_optimal_time[i-1]) / 2
    distance_optimal[i] = distance_optimal[i-1] + avg_speed * dt / 3.6  # 轉換為米/秒

# 車輛物理模型
class Vehicle:
    def __init__(self):
        self.position = 0.0  # 米
        self.speed = 0.0     # km/h
        self.acceleration = 0.0  # km/h/s
        self.time = 0.0      # 秒
        self.energy_consumption = 0.0  # 某種能量單位
        
    def update(self, dt, acceleration):
        # 更新加速度
        self.acceleration = acceleration
        
        # 更新速度
        self.speed += self.acceleration * dt
        self.speed = max(0, self.speed)  # 確保速度不為負
        
        # 更新位置
        self.position += self.speed * dt / 3.6  # 轉換為米/秒
        
        # 更新時間
        self.time += dt
        
        # 簡單的能耗模型 (可以使用更複雜的模型)
        # 能耗與速度平方和加速度平方成正比
        self.energy_consumption += (0.01 * self.speed**2 + 0.2 * self.acceleration**2) * dt

# 創建車輛實例
vehicle = Vehicle()

# 繪製函數
def draw_dashboard():
    # 清屏
    screen.fill(WHITE)
    
    # 繪製道路
    road_y = height - 100
    pygame.draw.line(screen, BLACK, (50, road_y), (width - 50, road_y), 5)
    
    # 繪製車輛
    car_x = 50 + (width - 100) * (vehicle.position / distance_optimal[-1])
    # pygame.draw.rect(screen, BLUE, (car_x - 20, road_y - 30, 40, 20))
    car_image = pygame.image.load('train.png')
    # 調整圖片大小
    car_image = pygame.transform.scale(car_image, (400, 250))  # 寬80像素，高40像素
    if car_image:
        # 如果有車輛圖片，則繪製圖片
        # 將車輛圖片的底部中心對準道路
        car_rect = car_image.get_rect()
        car_rect.midbottom = (car_x, road_y)
        screen.blit(car_image, car_rect)
    else:
        # 如果沒有圖片，則繪製矩形
        pygame.draw.rect(screen, BLUE, (car_x - 20, road_y - 30, 40, 20))
    
    # 繪製信息面板
    font = pygame.font.Font(None, 36)
    
    # 時間信息
    time_text = font.render(f"time: {vehicle.time:.1f} secs", True, BLACK)
    screen.blit(time_text, (50, 50))
    
    # 速度信息
    speed_text = font.render(f"velosity: {vehicle.speed:.1f} km/h", True, BLACK)
    screen.blit(speed_text, (50, 100))
    
    # 位置信息
    position_text = font.render(f"location: {vehicle.position:.1f} m", True, BLACK)
    screen.blit(position_text, (50, 150))
    
    # 能耗信息
    energy_text = font.render(f"energy: {vehicle.energy_consumption:.1f} KW", True, BLACK)
    screen.blit(energy_text, (50, 200))
    
    # 繪製最佳化曲線和當前車輛狀態對比
    draw_comparison_graphs()

def draw_comparison_graphs():
    # 創建速度-時間圖
    fig_time, ax_time = plt.subplots(figsize=(5, 3.75), dpi=80)
    ax_time.plot(time_optimal, speed_optimal_time, 'b-', label='optimal speed')
    ax_time.plot(vehicle.time, vehicle.speed, 'ro', label='current state')
    ax_time.set_xlabel('time(s)')
    ax_time.set_ylabel('velocity (km/h)')
    ax_time.set_title('time-velocity comparison')
    ax_time.grid(True)
    ax_time.legend()
    
    # 轉換為Pygame可用的surface
    canvas_time = FigureCanvasAgg(fig_time)
    canvas_time.draw()
    renderer_time = canvas_time.get_renderer()
    raw_data_time = renderer_time.tostring_rgb()
    size_time = canvas_time.get_width_height()
    surf_time = pygame.image.fromstring(raw_data_time, size_time, "RGB")
    screen.blit(surf_time, (width - 400, 50))
    plt.close(fig_time)
    
    # 創建速度-距離圖
    fig_dist, ax_dist = plt.subplots(figsize=(5, 3.75), dpi=80)
    ax_dist.plot(distance_optimal, speed_optimal_time, 'r-', label='optimal speed')
    ax_dist.plot(vehicle.position, vehicle.speed, 'bo', label='current state')
    ax_dist.set_xlabel('distance (m)')
    ax_dist.set_ylabel('velocity (km/h)')
    ax_dist.set_title('distance-velocity comparison')
    ax_dist.grid(True)
    ax_dist.legend()
    
    # 轉換為Pygame可用的surface
    canvas_dist = FigureCanvasAgg(fig_dist)
    canvas_dist.draw()
    renderer_dist = canvas_dist.get_renderer()
    raw_data_dist = renderer_dist.tostring_rgb()
    size_dist = canvas_dist.get_width_height()
    surf_dist = pygame.image.fromstring(raw_data_dist, size_dist, "RGB")
    screen.blit(surf_dist, (width - 400, 350))
    plt.close(fig_dist)

# 遊戲主循環
running = True
clock = pygame.time.Clock()

while running:
    dt = 1  # 時間步長，秒
    
    # 處理事件
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
          running = False
    
    # 獲取鍵盤輸入
    keys = pygame.key.get_pressed()
    
    # 控制加速度
    acceleration = 0
    if keys[pygame.K_UP]:
        acceleration = 10.0  # 加速
    elif keys[pygame.K_DOWN]:
        acceleration = -10.0  # 減速
    
    # 更新車輛狀態
    vehicle.update(dt, acceleration)
    
    # 檢查是否到達終點
    if vehicle.position >= distance_optimal[-1]:
        print("arrived at destination!")
        print(f"Total time: {vehicle.time:.1f} s")
        print(f"energy: {vehicle.energy_consumption:.1f} kw")
        running = False
    
    # 繪製儀表板
    draw_dashboard()
    
    # 更新顯示
    pygame.display.flip()
    
    # 控制幀率
    clock.tick(10)  # 每秒10幀

# 退出Pygame
pygame.quit()
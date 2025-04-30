"""
火車加速優化模擬器 - 主程式
負責初始化、遊戲迴圈和主要邏輯處理
"""
import pygame
import numpy as np
import sys
from vehicle_model import Vehicle
from optimizer import TrainEnergyOptimizer
from visualization import draw_optimization_results
import ui  # 導入 UI 模組

# 初始化 Pygame
pygame.init()
width, height = 1000, 750 
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption("Train Acceleration Optimization Simulator")

# 定義段落速度限制
speed_limits = [
    (0, 25.0),       # 0-200m 限速 25 m/s (90 km/h)
    (200, 30.0),     # 200-900m 限速 30 m/s (108 km/h)
    (900, 30.0),     # 900-1000m 限速 30 m/s (108 km/h)
]

# 創建車輛和優化器
vehicle = Vehicle()
optimizer = TrainEnergyOptimizer(
    distance_m=1000.0,
    time_s=60.0,
    max_speed_mps=30.0,
    max_accel=3.3,
    control_points=None,  # 使用動態計算的控制點數量
    speed_limits=speed_limits  # 加入速限設定
)

# 預先在背景執行優化，限制迭代次數
results = optimizer.optimize(maxiter=30)
time_optimal, speed_optimal_time = results['optimal_time'], results['optimal_speed']
distance_optimal = np.cumsum(speed_optimal_time) / 3.6

# 繪製優化結果圖表
# draw_optimization_results(results)

# 初始化 UI 模組
ui.init(screen, width, height, optimizer)

# 創建主選單按鈕
main_menu_buttons = ui.create_main_menu_buttons()

# 創建返回按鈕
back_button = ui.create_back_button()

# 創建遊戲中的返回按鈕
game_back_button = ui.create_game_back_button()

# 重置車輛狀態
def reset_vehicle():
    global vehicle
    vehicle = Vehicle()

# 更新優化結果
def update_optimization_results():
    global results, time_optimal, speed_optimal_time, distance_optimal
    # 重新執行優化
    results = optimizer.optimize(maxiter=30)
    time_optimal, speed_optimal_time = results['optimal_time'], results['optimal_speed']
    distance_optimal = np.cumsum(speed_optimal_time) / 3.6
    # 繪製新的優化結果圖表
    # draw_optimization_results(results)

# 遊戲主循環
game_state = ui.GameState.MAIN_MENU
running, clock = True, pygame.time.Clock()

while running:
    mouse_pos = pygame.mouse.get_pos()
    mouse_click = False
    
    # 事件處理
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1:  # 左鍵點擊
                mouse_click = True
        
        # 處理設定頁面的文字輸入
        if game_state == ui.GameState.SETTINGS:
            ui.handle_settings_event(event, mouse_pos, mouse_click)
    
    # 根據遊戲狀態處理不同畫面
    if game_state == ui.GameState.MAIN_MENU:
        # 檢查按鈕懸停和點擊
        for i, button in enumerate(main_menu_buttons):
            button.check_hover(mouse_pos)
            if button.is_clicked(mouse_pos, mouse_click):
                if i == 0:  # 開始遊戲
                    game_state = ui.GameState.GAME
                    reset_vehicle()  # 重置車輛狀態
                elif i == 1:  # 進入設定
                    game_state = ui.GameState.SETTINGS
                elif i == 2:  # 查看排行
                    game_state = ui.GameState.LEADERBOARD
                elif i == 3:  # 離開遊戲
                    running = False
        
        ui.draw_main_menu(main_menu_buttons)
    
    elif game_state == ui.GameState.SETTINGS:
        back_button.check_hover(mouse_pos)
        if back_button.is_clicked(mouse_pos, mouse_click):
            # 檢查設定是否變更，如果變更則更新優化結果
            if ui.is_settings_changed():
                update_optimization_results()
            game_state = ui.GameState.MAIN_MENU
        
        ui.draw_settings(back_button)
    
    elif game_state == ui.GameState.LEADERBOARD:
        back_button.check_hover(mouse_pos)
        if back_button.is_clicked(mouse_pos, mouse_click):
            game_state = ui.GameState.MAIN_MENU
        
        ui.draw_leaderboard(back_button)
    
    elif game_state == ui.GameState.GAME:
        dt = 0.5  # 時間步長（秒）
        
        # 處理鍵盤輸入
        keys = pygame.key.get_pressed()
        acceleration = optimizer.max_accel if keys[pygame.K_UP] else (-optimizer.max_accel if keys[pygame.K_DOWN] else 0)
        
        # 更新車輛狀態
        vehicle.update(dt, acceleration)
        
        # 檢查是否到達目的地
        if vehicle.position >= optimizer.distance_m:
            print(f"Destination reached! Total time: {vehicle.time:.1f} s, Energy: {vehicle.energy_consumption:.3f} kWh")
            game_state = ui.GameState.MAIN_MENU
        
        # 檢查遊戲中的返回按鈕
        game_back_button.check_hover(mouse_pos)
        if game_back_button.is_clicked(mouse_pos, mouse_click):
            game_state = ui.GameState.MAIN_MENU
        
        ui.draw_dashboard(vehicle, game_back_button, distance_optimal, time_optimal, speed_optimal_time)
    
    pygame.display.flip()
    clock.tick(20)

pygame.quit()
sys.exit()
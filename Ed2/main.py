"""
火車加速優化模擬器 - 主程式
負責初始化、遊戲迴圈和主要邏輯處理
"""
import pygame
import numpy as np
import sys
from vehicle_model import Vehicle
from optimizer import TrainEnergyOptimizer
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
    max_accel=2.0,
    control_points=None,  # 使用動態計算的控制點數量
    speed_limits=speed_limits  # 加入速限設定
)

# 預先在背景執行優化，限制迭代次數
results = optimizer.optimize(maxiter=30)
time_optimal, speed_optimal_time = results['optimal_time'], results['optimal_speed']
distance_optimal = np.cumsum(speed_optimal_time)

# 初始化 UI 模組
ui.init(screen, width, height, optimizer)

# 創建主選單按鈕
main_menu_buttons = ui.create_main_menu_buttons()

# 創建返回按鈕
back_button = ui.Button(width//2 - 175, height - 150, 350, 70, "Back to Main Menu", ui.LIGHT_GRAY, (180, 180, 180))

# 創建遊戲中的返回按鈕
game_back_button = ui.Button(width - 150, 20, 130, 50, "Menu", ui.LIGHT_GRAY, (180, 180, 180))

# 創建結果畫面按鈕
retry_button, result_back_button = ui.create_result_buttons()

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

# 計算分數
def calculate_score(time, energy):
    """計算遊戲分數"""
    optimal_energy = optimizer.optimal_result['optimal_energy']
    optimal_energy_val = float(optimal_energy[0]) if isinstance(optimal_energy, (list, np.ndarray)) else float(optimal_energy)
    
    time_score = max(0, 50 - abs(time - 60))
    energy_score = 50 * (optimal_energy_val / energy if energy != 0 else 0)
    total_score = time_score + energy_score
    
    return total_score

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
        '''
        # 處理鍵盤輸入
        keys = pygame.key.get_pressed()
        acceleration = 2.0 if keys[pygame.K_UP] else (-4.0 if keys[pygame.K_DOWN] else 0)
        '''
        keys = pygame.key.get_pressed()
        # 新的加速度邏輯：無輸入時自然減速（模擬空氣阻力和摩擦力）
        if keys[pygame.K_UP]:
            acceleration = 4.0  # 按上鍵加速，更大的加速度
        elif keys[pygame.K_DOWN]:
            acceleration = -3.0  # 按下鍵剎車，剎車力度稍大
        else:
            # 自然減速 - 與速度成正比的阻力（空氣阻力）
            current_speed = vehicle.speed
            if current_speed > 0.1:  # 防止速度接近0時抖動
                acceleration = -0.5 * (current_speed / 20.0)  # 速度越快，阻力越大
            else:
                vehicle.speed = 0.0  # 低於閾值直接設為0
                acceleration = 0.0
        # 更新車輛狀態
        vehicle.update(dt, acceleration)
        
        # 檢查是否到達目的地
        if vehicle.position >= optimizer.distance_m:
            print(f"Destination reached! Total time: {vehicle.time:.1f} s, Energy: {vehicle.energy_consumption:.3f} kWh")
            
            # 計算分數
            total_score = calculate_score(vehicle.time, vehicle.energy_consumption)
            
            # 保存遊戲成績
            ui.save_game_score(total_score, vehicle.time, vehicle.energy_consumption, vehicle.position)
            
            # 切換到結果畫面
            game_state = ui.GameState.RESULT
        
        # 檢查遊戲中的返回按鈕
        game_back_button.check_hover(mouse_pos)
        if game_back_button.is_clicked(mouse_pos, mouse_click):
            game_state = ui.GameState.MAIN_MENU
        
        ui.draw_dashboard(vehicle, game_back_button, distance_optimal, time_optimal, speed_optimal_time)
    
    elif game_state == ui.GameState.RESULT:
        # 結果畫面按鈕處理
        retry_button.check_hover(mouse_pos)
        result_back_button.check_hover(mouse_pos)
        
        if retry_button.is_clicked(mouse_pos, mouse_click):
            reset_vehicle()
            game_state = ui.GameState.GAME
        
        if result_back_button.is_clicked(mouse_pos, mouse_click):
            game_state = ui.GameState.MAIN_MENU
        
        ui.draw_result(result_back_button, retry_button)
    
    pygame.display.flip()
    clock.tick(10)

pygame.quit()
sys.exit()
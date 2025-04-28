import pygame
import numpy as np
import random
import sys
from vehicle_model import Vehicle
from optimizer import TrainEnergyOptimizer
from visualization import draw_comparison_graphs

pygame.init()
width, height = 1000, 750 
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption("Train Acceleration Optimization Simulator")
WHITE, BLACK, RED, GREEN, BLUE, YELLOW, LIGHT_GRAY, DARK_GRAY = (255, 255, 255), (0, 0, 0), (255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0), (220, 220, 220), (100, 100, 100)

# 定義字體路徑
font_path = "../src/MSJH.TTC"

# 定義字體，如果找不到指定字體則使用系統默認字體
try:
    title_font = pygame.font.Font(font_path, 72)
    button_font = pygame.font.Font(font_path, 48)
    normal_font = pygame.font.Font(font_path, 36)
except FileNotFoundError:
    print(f"Cannot find font file: {font_path}, using system default font.")
    title_font = pygame.font.Font(None, 72)
    button_font = pygame.font.Font(None, 48)
    normal_font = pygame.font.Font(None, 36)

# 定義按鈕類別
class Button:
    def __init__(self, x, y, width, height, text, color, hover_color):
        self.rect = pygame.Rect(x, y, width, height)
        self.text = text
        self.color = color
        self.hover_color = hover_color
        self.is_hovered = False
        
    def draw(self, surface):
        color = self.hover_color if self.is_hovered else self.color
        pygame.draw.rect(surface, color, self.rect, 0, 15)  # 圓角矩形
        pygame.draw.rect(surface, BLACK, self.rect, 2, 15)  # 邊框
        
        text_surf = button_font.render(self.text, True, BLACK)
        text_rect = text_surf.get_rect(center=self.rect.center)
        surface.blit(text_surf, text_rect)
    
    def check_hover(self, mouse_pos):
        self.is_hovered = self.rect.collidepoint(mouse_pos)
        return self.is_hovered
    
    def is_clicked(self, mouse_pos, mouse_click):
        return self.rect.collidepoint(mouse_pos) and mouse_click

# 定義遊戲狀態
class GameState:
    MAIN_MENU = 0
    SETTINGS = 1
    LEADERBOARD = 2
    GAME = 3
    EXIT = 4

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
distance_optimal = np.cumsum(speed_optimal_time) / 3.6

# 創建主頁面按鈕
def create_main_menu_buttons():
    button_width, button_height = 300, 70
    x_center = width // 2 - button_width // 2
    
    start_button = Button(x_center, 300, button_width, button_height, "Start Game", GREEN, (100, 255, 100))
    settings_button = Button(x_center, 390, button_width, button_height, "Settings", BLUE, (100, 100, 255))
    leaderboard_button = Button(x_center, 480, button_width, button_height, "Leaderboard", YELLOW, (255, 255, 100))
    exit_button = Button(x_center, 570, button_width, button_height, "Exit Game", RED, (255, 100, 100))
    
    return [start_button, settings_button, leaderboard_button, exit_button]

def draw_main_menu():
    screen.fill(WHITE)
    
    # 繪製標題
    title_text = title_font.render("Train Acceleration Simulator", True, BLACK)
    title_rect = title_text.get_rect(center=(width//2, 150))
    screen.blit(title_text, title_rect)
    
    # 繪製所有按鈕
    for button in main_menu_buttons:
        button.draw(screen)
    
    # 繪製版權資訊
    copyright_text = normal_font.render("© 2025 Train Simulation Team", True, DARK_GRAY)
    copyright_rect = copyright_text.get_rect(center=(width//2, height-50))
    screen.blit(copyright_text, copyright_rect)

def draw_settings():
    screen.fill(WHITE)
    # 設定頁面的內容
    title_text = title_font.render("Game Settings", True, BLACK)
    title_rect = title_text.get_rect(center=(width//2, 100))
    screen.blit(title_text, title_rect)
    
    # 這裡可以添加設定選項
    setting_text = normal_font.render("Settings page under development...", True, BLACK)
    setting_rect = setting_text.get_rect(center=(width//2, 300))
    screen.blit(setting_text, setting_rect)
    
    # 返回按鈕
    back_button.draw(screen)

def draw_leaderboard():
    screen.fill(WHITE)
    # 排行榜頁面的內容
    title_text = title_font.render("Game Leaderboard", True, BLACK)
    title_rect = title_text.get_rect(center=(width//2, 100))
    screen.blit(title_text, title_rect)
    
    # 這裡可以添加排行榜資料
    leader_text = normal_font.render("Leaderboard page under development...", True, BLACK)
    leader_rect = leader_text.get_rect(center=(width//2, 300))
    screen.blit(leader_text, leader_rect)
    
    # 返回按鈕
    back_button.draw(screen)

def draw_dashboard():
    screen.fill(WHITE)
    
    # 繪製路線基準線
    pygame.draw.line(screen, BLACK, (50, height - 100), (width - 50, height - 100), 5)
    
    # 繪製速度限制區域
    for i, (start_pos, limit) in enumerate(speed_limits):
        # 計算結束位置
        end_pos = speed_limits[i+1][0] if i < len(speed_limits) - 1 else optimizer.distance_m
        
        # 轉換為螢幕座標
        start_x = 50 + (width - 100) * (start_pos / optimizer.distance_m)
        end_x = 50 + (width - 100) * (end_pos / optimizer.distance_m)
        
        # 繪製彩色區塊指示速度限制區域
        limit_color = (255, 255, 200) if limit >= 25 else (200, 255, 200) if limit >= 16.7 else (255, 200, 200)
        pygame.draw.rect(screen, limit_color, (start_x, height - 130, end_x - start_x, 35))
        
        # 繪製速度限制文字
        limit_font = pygame.font.Font(None, 24)
        limit_text = f"{int(limit * 3.6)} km/h"
        limit_surf = limit_font.render(limit_text, True, BLACK)
        screen.blit(limit_surf, (start_x + 5, height - 125))
    
    # 繪製車輛
    car_x = 50 + (width - 100) * (vehicle.position / distance_optimal[-1])
    pygame.draw.rect(screen, BLUE, (car_x - 20, height - 130, 40, 20))
    
    # 獲取當前位置速度限制
    current_speed_limit = optimizer.get_speed_limit_at_position(vehicle.position) * 3.6  # 轉換為 km/h
    
    # 計算剩餘距離
    remain_distance = distance_optimal[-1] - vehicle.position
    
    # 使用get_dynamic_speed_recommendation獲取建議
    recommendation, recommended_speed = optimizer.get_dynamic_speed_recommendation(
        # 取得建議動作，建議速度值
        current_speed=vehicle.speed,
        remain_distance=remain_distance,
        current_time=vehicle.time
    )
    
    # 設定建議顏色
    if recommendation == "MAT":
        rec_color = YELLOW
    elif recommendation == "ACC":
        rec_color = GREEN
    else:
        rec_color = RED
    
    # 顯示基本信息
    font = pygame.font.Font(None, 36)
    texts = [
        f"Time: {vehicle.time:.1f} s",
        f"Speed: {vehicle.speed:.1f} km/h",
        f"Position: {vehicle.position:.1f} m",
        f"Energy: {vehicle.energy_consumption:.1f} kWh",
        f"Target Speed: {recommended_speed:.1f} km/h",
        f"Current Limit: {current_speed_limit:.1f} km/h",
    ]
    
    for i, text in enumerate(texts):
        color = BLACK if i < 5 or i == 5 else rec_color if i == 6 else BLACK
        screen.blit(font.render(text, True, color), (50, 50 + i * 40))
    
    # 顯示箭頭指示器
    if vehicle.time % 1 < 0.04:  # 每0.04秒閃爍
        if recommendation == "ACC":
            # 向上箭頭
            pygame.draw.polygon(screen, GREEN, [(width-250, 70), (width-230, 40), (width-210, 70)])
            pygame.draw.line(screen, GREEN, (width-230, 70), (width-230, 100), 3)
        elif recommendation == "DEC":
            # 向下箭頭
            pygame.draw.polygon(screen, RED, [(width-250, 70), (width-230, 100), (width-210, 70)])
            pygame.draw.line(screen, RED, (width-230, 40), (width-230, 70), 3)
        else:  # 保持速度
            # 水平箭頭
            pygame.draw.line(screen, YELLOW, (width-270, 70), (width-190, 70), 3)
            pygame.draw.polygon(screen, YELLOW, [(width-200, 60), (width-190, 70), (width-200, 80)])
            pygame.draw.polygon(screen, YELLOW, [(width-260, 60), (width-270, 70), (width-260, 80)])
        
        # 顯示建議文字
        arrow_text = f"Recommendation: {recommendation}"
        screen.blit(font.render(arrow_text, True, rec_color), (width - 350, 120))
    
    # 添加返回主選單按鈕
    game_back_button.draw(screen)

# 重置車輛狀態
def reset_vehicle():
    global vehicle
    vehicle = Vehicle()

# 創建主選單按鈕
main_menu_buttons = create_main_menu_buttons()

# 創建返回按鈕（用於設定和排行榜頁面）
back_button = Button(width//2 - 150, height - 150, 300, 70, "Back to Main Menu", LIGHT_GRAY, (180, 180, 180))

# 創建遊戲中的返回按鈕
game_back_button = Button(width - 150, 20, 130, 50, "Menu", LIGHT_GRAY, (180, 180, 180))

# 遊戲主循環
game_state = GameState.MAIN_MENU
running, clock = True, pygame.time.Clock()

while running:
    mouse_pos = pygame.mouse.get_pos()
    mouse_click = False
    
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1:  # 左鍵點擊
                mouse_click = True
    
    # 根據遊戲狀態處理不同畫面
    if game_state == GameState.MAIN_MENU:
        # 檢查按鈕懸停和點擊
        for i, button in enumerate(main_menu_buttons):
            button.check_hover(mouse_pos)
            if button.is_clicked(mouse_pos, mouse_click):
                if i == 0:  # 開始遊戲
                    game_state = GameState.GAME
                    reset_vehicle()  # 重置車輛狀態
                elif i == 1:  # 進入設定
                    game_state = GameState.SETTINGS
                elif i == 2:  # 查看排行
                    game_state = GameState.LEADERBOARD
                elif i == 3:  # 離開遊戲
                    running = False
        
        draw_main_menu()
    
    elif game_state == GameState.SETTINGS:
        back_button.check_hover(mouse_pos)
        if back_button.is_clicked(mouse_pos, mouse_click):
            game_state = GameState.MAIN_MENU
        
        draw_settings()
    
    elif game_state == GameState.LEADERBOARD:
        back_button.check_hover(mouse_pos)
        if back_button.is_clicked(mouse_pos, mouse_click):
            game_state = GameState.MAIN_MENU
        
        draw_leaderboard()
    
    elif game_state == GameState.GAME:
        dt = 0.5  # 時間步長（秒）
        
        # 處理鍵盤輸入
        keys = pygame.key.get_pressed()
        acceleration = 2.0 if keys[pygame.K_UP] else (-4.0 if keys[pygame.K_DOWN] else 0)
        
        # 更新車輛狀態
        vehicle.update(dt, acceleration)
        
        # 檢查是否到達目的地
        if vehicle.position >= distance_optimal[-1]:
            print(f"Destination reached! Total time: {vehicle.time:.1f} s, Energy: {vehicle.energy_consumption:.1f} kWh")
            game_state = GameState.MAIN_MENU
        
        # 檢查遊戲中的返回按鈕
        game_back_button.check_hover(mouse_pos)
        if game_back_button.is_clicked(mouse_pos, mouse_click):
            game_state = GameState.MAIN_MENU
        
        draw_dashboard()
    
    pygame.display.flip()
    clock.tick(10)

pygame.quit()
sys.exit()
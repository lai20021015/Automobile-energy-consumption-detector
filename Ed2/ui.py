"""
火車加速優化模擬器 - UI 模組
負責所有使用者介面繪製和元素定義
"""
import pygame
import numpy as np
from enum import Enum
from visualization import draw_comparison_graphs

# 定義顏色常量
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
YELLOW = (255, 255, 0)
LIGHT_GRAY = (220, 220, 220)
DARK_GRAY = (100, 100, 100)

# 全局變數
screen = None
width = None
height = None
optimizer = None
title_font = None
button_font = None
normal_font = None

# 設定頁面輸入框內容
distance_input_text = "1000.0"
time_input_text = "60.0"
active_input = None  # 當前活動的輸入框
submit_button = None  # 提交按鈕
settings_changed = False  # 設定是否已更改

# 定義遊戲狀態
class GameState(Enum):
    MAIN_MENU = 0
    SETTINGS = 1
    LEADERBOARD = 2
    GAME = 3
    EXIT = 4

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

# 初始化 UI 模組
def init(screen_surface, screen_width, screen_height, opt):
    """初始化 UI 模組的全局變數"""
    global screen, width, height, optimizer, title_font, button_font, normal_font, submit_button
    global distance_input_text, time_input_text
    
    screen = screen_surface
    width = screen_width
    height = screen_height
    optimizer = opt
    
    # 初始化設定頁面輸入值
    distance_input_text = str(optimizer.distance_m)
    time_input_text = str(optimizer.time_s)
    
    # 創建提交按鈕
    submit_button = Button(width//2 - 75, 400, 150, 50, "Submit", GREEN, (100, 255, 100))
    
    # 定義字體路徑
    font_path = "path/to/your/font.ttf"  # 替換為實際字體檔案路徑
    
    # 初始化字體
    try:
        title_font = pygame.font.Font(font_path, 72)
        button_font = pygame.font.Font(font_path, 48)
        normal_font = pygame.font.Font(font_path, 36)
    except FileNotFoundError:
        print(f"Cannot find font file: {font_path}, using system default font.")
        title_font = pygame.font.Font(None, 72)
        button_font = pygame.font.Font(None, 48)
        normal_font = pygame.font.Font(None, 36)

# 創建主選單按鈕
def create_main_menu_buttons():
    """創建主選單的按鈕"""
    button_width, button_height = 300, 70
    x_center = width // 2 - button_width // 2
    start_button = Button(x_center, 300, button_width, button_height, "Start Game", GREEN, (100, 255, 100))
    settings_button = Button(x_center, 390, button_width, button_height, "Settings", BLUE, (100, 100, 255))
    leaderboard_button = Button(x_center, 480, button_width, button_height, "Leaderboard", YELLOW, (255, 255, 100))
    exit_button = Button(x_center, 570, button_width, button_height, "Exit Game", RED, (255, 100, 100))
    
    return [start_button, settings_button, leaderboard_button, exit_button]

# 創建返回按鈕
def create_back_button():
    """創建返回主選單按鈕"""
    return Button(width//2 - 200, height - 200, 400, 70, "Back to Main Menu", LIGHT_GRAY, (180, 180, 180))

# 創建遊戲中的返回按鈕
def create_game_back_button():
    """創建遊戲中的返回選單按鈕"""
    return Button(width - 150, 20, 130, 50, "Menu", LIGHT_GRAY, (180, 180, 180))

# 繪製主選單
def draw_main_menu(buttons):
    """繪製主選單界面"""
    screen.fill(WHITE)
    
    # 加載並繪製 IEM 圖片
    iem_image = pygame.image.load("src/iem.png")
    iem_image = pygame.transform.scale(iem_image, (200, 200))  # 調整圖片大小
    screen.blit(iem_image, (width - 280, height // 2 - 100))  # 繪製在右邊偏中間的位置

    # 繪製標題
    title_text = title_font.render("Train Acceleration Simulator", True, BLACK)
    title_rect = title_text.get_rect(center=(width//2, 150))
    screen.blit(title_text, title_rect)
    
    # 繪製所有按鈕
    for button in buttons:
        button.draw(screen)
    
    # 繪製版權資訊
    copyright_text = normal_font.render("© 2025 IEM Driving Simulation Team", True, DARK_GRAY)
    copyright_rect = copyright_text.get_rect(center=(width//2, height-50))
    screen.blit(copyright_text, copyright_rect)

# 處理設定頁面事件
def handle_settings_event(event, mouse_pos, mouse_click):
    """處理設定頁面的事件（如輸入框、按鈕點擊等）"""
    global active_input, distance_input_text, time_input_text, settings_changed
    
    # 定義輸入框
    distance_input_rect = pygame.Rect(width//2 + 50, 200, 200, 40)
    time_input_rect = pygame.Rect(width//2 + 50, 300, 200, 40)
    
    # 處理點擊事件
    if mouse_click:
        # 檢查點擊輸入框
        if distance_input_rect.collidepoint(mouse_pos):
            active_input = "distance"
        elif time_input_rect.collidepoint(mouse_pos):
            active_input = "time"
        else:
            active_input = None
            
        # 檢查提交按鈕點擊
        if submit_button.is_clicked(mouse_pos, mouse_click):
            try:
                # 轉換為浮點數並更新 optimizer
                new_distance = float(distance_input_text)
                new_time = float(time_input_text)
                
                if (new_distance != optimizer.distance_m or new_time != optimizer.time_s):
                    optimizer.distance_m = new_distance
                    optimizer.time_s = new_time
                    settings_changed = True
                    print(f"Settings updated: Distance={new_distance}m, Time={new_time}s")
            except ValueError:
                print("Invalid input. Please enter valid numbers.")
    
    # 處理鍵盤輸入
    if event.type == pygame.KEYDOWN:
        if active_input == "distance":
            if event.key == pygame.K_BACKSPACE:
                distance_input_text = distance_input_text[:-1]
            elif event.key == pygame.K_RETURN:
                active_input = None
            elif event.unicode.isdigit() or event.unicode == '.':
                distance_input_text += event.unicode
        elif active_input == "time":
            if event.key == pygame.K_BACKSPACE:
                time_input_text = time_input_text[:-1]
            elif event.key == pygame.K_RETURN:
                active_input = None
            elif event.unicode.isdigit() or event.unicode == '.':
                time_input_text += event.unicode

# 繪製設定頁面
def draw_settings(back_btn):
    """繪製設定頁面"""
    screen.fill(WHITE)
    
    # 設定頁面的內容
    title_text = title_font.render("Game Settings", True, BLACK)
    title_rect = title_text.get_rect(center=(width//2, 100))
    screen.blit(title_text, title_rect)
    
    # 添加輸入框標籤
    distance_label = normal_font.render("Set Distance (m):", True, BLACK)
    distance_label_rect = distance_label.get_rect(topleft=(width//2 - 250, 200))
    screen.blit(distance_label, distance_label_rect)
    
    time_label = normal_font.render("Set Time (s):", True, BLACK)
    time_label_rect = time_label.get_rect(topleft=(width//2 - 250, 300))
    screen.blit(time_label, time_label_rect)
    
    # 創建輸入框
    distance_input_rect = pygame.Rect(width//2 + 50, 200, 200, 40)
    time_input_rect = pygame.Rect(width//2 + 50, 300, 200, 40)
    
    # 依據活動狀態設定輸入框顏色
    distance_box_color = (200, 200, 255) if active_input == "distance" else LIGHT_GRAY
    time_box_color = (200, 200, 255) if active_input == "time" else LIGHT_GRAY
    
    pygame.draw.rect(screen, distance_box_color, distance_input_rect, 0, 5)
    pygame.draw.rect(screen, BLACK, distance_input_rect, 2, 5)
    
    pygame.draw.rect(screen, time_box_color, time_input_rect, 0, 5)
    pygame.draw.rect(screen, BLACK, time_input_rect, 2, 5)
    
    # 繪製輸入框中的文字
    distance_text_surface = normal_font.render(distance_input_text, True, BLACK)
    time_text_surface = normal_font.render(time_input_text, True, BLACK)
    
    screen.blit(distance_text_surface, (distance_input_rect.x + 5, distance_input_rect.y + 5))
    screen.blit(time_text_surface, (time_input_rect.x + 5, time_input_rect.y + 5))
    
    # 提交按鈕
    submit_button.check_hover(pygame.mouse.get_pos())
    submit_button.draw(screen)
    
    # 返回按鈕
    back_btn.draw(screen)

# 檢查設定是否已更改
def is_settings_changed():
    """檢查設定是否已被更改"""
    global settings_changed
    changed = settings_changed
    settings_changed = False  # 重置狀態
    return changed

# 繪製排行榜頁面
def draw_leaderboard(back_btn):
    """繪製排行榜頁面"""
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
    back_btn.draw(screen)

# 繪製遊戲儀表板
def draw_dashboard(vehicle, game_back_btn, distance_optimal, time_optimal, speed_optimal_time):
    """繪製遊戲儀表板，顯示車輛狀態和控制資訊"""
    screen.fill(WHITE)
    
    # 繪製路線基準線
    pygame.draw.line(screen, BLACK, (50, height - 100), (width - 50, height - 100), 5)
    
    # 繪製速度限制區域
    for i, (start_pos, limit) in enumerate(optimizer.speed_limits):
        # 計算結束位置
        end_pos = optimizer.speed_limits[i+1][0] if i < len(optimizer.speed_limits) - 1 else optimizer.distance_m
        
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
    car_x = 50 + (width - 100) * (vehicle.position / optimizer.distance_m)
    pygame.draw.rect(screen, BLUE, (car_x - 20, height - 130, 40, 20))
    
    # 獲取當前位置速度限制
    current_speed_limit = optimizer.get_speed_limit_at_position(vehicle.position) * 3.6  # 轉換為 km/h
    
    # 計算剩餘距離
    remain_distance = optimizer.distance_m - vehicle.position
    
    # 顯示剩餘距離
    remain_distance_text = f"Remaining Distance: {remain_distance:.1f} m"
    remain_distance_surf = normal_font.render(remain_distance_text, True, BLACK)
    screen.blit(remain_distance_surf, (50, 20))

    # 使用get_dynamic_speed_recommendation獲取建議
    recommendation, recommended_speed = optimizer.get_dynamic_speed_recommendation(
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
        color = BLACK
        screen.blit(font.render(text, True, color), (50, 50 + i * 40))
    
    # 顯示箭頭指示器
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
    
    # 繪製比較圖表
    # draw_comparison_graphs(screen, width, vehicle, time_optimal, speed_optimal_time, distance_optimal)
    
    # 添加返回主選單按鈕
    game_back_btn.draw(screen)
"""
火車加速優化模擬器 - UI 模組
負責所有使用者介面繪製和元素定義
"""
import pygame
import numpy as np
from enum import Enum
import csv
import os
import math

# 定義顏色常量
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)  
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
YELLOW = (255, 255, 0)
LIGHT_GRAY = (220, 220, 220)
DARK_GRAY = (100, 100, 100)
GRAY = (128, 128, 128)
LIGHT_YELLOW = (255, 255, 200)

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

# Leaderboard相關變數
ranking_file = 'ranking.csv'
ranking = []
last_game_time = 0
last_game_energy = 0  
last_game_score = 0
last_game_distance = 0

# 定義遊戲狀態
class GameState(Enum):
    MAIN_MENU = 0
    SETTINGS = 1
    LEADERBOARD = 2
    GAME = 3
    RESULT = 4
    EXIT = 5

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
    global distance_input_text, time_input_text, ranking
    
    screen = screen_surface
    width = screen_width
    height = screen_height
    optimizer = opt
    
    # 初始化設定頁面輸入值
    distance_input_text = str(optimizer.distance_m)
    time_input_text = str(optimizer.time_s)
    
    # 創建提交按鈕
    submit_button = Button(width//2 - 75, 400, 150, 50, "Submit", GREEN, (100, 255, 100))
    
    # 初始化字體
    try:
        title_font = pygame.font.Font(None, 72)
        button_font = pygame.font.Font(None, 48)
        normal_font = pygame.font.Font(None, 36)
    except:
        title_font = pygame.font.Font(None, 72)
        button_font = pygame.font.Font(None, 48)
        normal_font = pygame.font.Font(None, 36)
        
    # 載入排行榜資料
    if os.path.exists(ranking_file):
        with open(ranking_file, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                if len(row) >= 3:
                    total_score = float(row[0])
                    total_time = float(row[1])
                    energy = float(row[2])
                    ranking.append([total_score, total_time, energy])

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
    return Button(width//2 - 175, height - 150, 350, 70, "Back to Main Menu", LIGHT_GRAY, (180, 180, 180))

# 創建遊戲中的返回按鈕
def create_game_back_button():
    """創建遊戲中的返回選單按鈕"""
    return Button(width - 150, 20, 130, 50, "Menu", LIGHT_GRAY, (180, 180, 180))

# 創建結果畫面按鈕
def create_result_buttons():
    """創建結果畫面的按鈕"""
    retry_button = Button(width//2 + 230, 650, 180, 50, "Try Again", RED, (255, 100, 100))
    result_back_button = Button(width//2 - 130, 650, 350, 50, "Back to Main Menu", GREEN, (100, 255, 100))
    return retry_button, result_back_button

# 繪製主選單
def draw_main_menu(buttons):
    """繪製主選單界面"""
    screen.fill(WHITE)
    
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

# 皇冠繪製函數
def draw_crown(surface, x, y, scale=1.0):
    """繪製皇冠圖示"""
    # 底座
    base_width = int(80 * scale)
    base_height = int(20 * scale)
    pygame.draw.rect(surface, (240, 230, 0), (x, y + 40 * scale, base_width, base_height))
    
    # 三個尖角
    peak_color = (240, 230, 0)
    pygame.draw.polygon(surface, peak_color, [
        (x, y + 40 * scale),
        (x + 20 * scale, y),
        (x + 40 * scale, y + 40 * scale)
    ])
    pygame.draw.polygon(surface, peak_color, [
        (x + 20 * scale, y + 40 * scale),
        (x + 40 * scale, y),
        (x + 60 * scale, y + 40 * scale)
    ])
    pygame.draw.polygon(surface, peak_color, [
        (x + 40 * scale, y + 40 * scale),
        (x + 60 * scale, y),
        (x + 80 * scale, y + 40 * scale)
    ])
    
    # 王冠珠寶
    pygame.draw.circle(surface, (255, 0, 0), (int(x + 20 * scale), int(y)), int(4 * scale))
    pygame.draw.circle(surface, (0, 255, 0), (int(x + 40 * scale), int(y)), int(4 * scale))
    pygame.draw.circle(surface, (0, 0, 255), (int(x + 60 * scale), int(y)), int(4 * scale))

# 繪製排行榜頁面
def draw_leaderboard(back_btn):
    """繪製排行榜頁面"""
    screen.fill(WHITE)
    
    # 標題
    title_text = title_font.render("Leaderboard", True, BLACK)
    title_rect = title_text.get_rect(center=(width//2, 100))
    screen.blit(title_text, title_rect)
    
    # 背景
    pygame.draw.rect(screen, BLACK, (width//2-380, height//2-130, 760, 270), border_radius=20)
    pygame.draw.rect(screen, LIGHT_GRAY, (width//2-375, height//2-125, 750, 260), border_radius=20)
    pygame.draw.rect(screen, WHITE, (width//2-350, height//2-112, 700, 66), border_radius=10)
    pygame.draw.rect(screen, WHITE, (width//2-350, height//2-32, 700, 66), border_radius=10)
    pygame.draw.rect(screen, WHITE, (width//2-350, height//2+50, 700, 66), border_radius=10)
    
    # 皇冠圖示
    draw_crown(screen, width//2 - 40, 152, scale=1.0)
    
    # 排序分數
    sorted_ranking = sorted(ranking, key=lambda x: (-x[0], x[2], x[1]))
    
    # 顯示前三名
    y_offset = height//2-75
    for i, (total_score, time, energy) in enumerate(sorted_ranking[:3], start=1):
        rank_text = normal_font.render(
            f"{i}. Score: {total_score:.0f} pts, Time: {time:.1f} s, Energy: {energy:.1f} kWh", 
            True, BLACK
        )
        rank_rect = rank_text.get_rect(center=(width//2, y_offset))
        screen.blit(rank_text, rank_rect)
        y_offset += 80
    
    # 返回按鈕
    back_btn.draw(screen)

# 繪製結果畫面
def draw_result(back_btn, retry_btn):
    """繪製結果畫面"""
    screen.fill(WHITE)
    
    # 標題
    if last_game_time <= 60:
        title_text = title_font.render("Success!", True, GREEN)
    else:
        title_text = title_font.render("Fail...", True, RED)
    title_rect = title_text.get_rect(center=(width//2, 100))
    screen.blit(title_text, title_rect)
    
    # 遊戲結果資訊
    if last_game_time == 60:
        status_text = "On Time!"
        status_color = GREEN
    elif last_game_time < 60:
        status_text = "Early Arrived!"
        status_color = GREEN
    else:
        status_text = "You're Late!"
        status_color = RED
    
    # 顯示結果資訊
    y_offset = height//2 - 80
    info_texts = [
        (status_text, status_color),
        (f"Score: {last_game_score:.0f} pts", BLACK),
        (f"Finished Time: {last_game_time:.1f} s", BLACK),
        (f"Distance: {last_game_distance:.1f} km", BLACK),
        (f"Total Energy: {last_game_energy:.1f} kWh", BLACK)
    ]
    
    for text, color in info_texts:
        text_surf = normal_font.render(text, True, color)
        screen.blit(text_surf, (120, y_offset))
        y_offset += 40
    
    # 按鈕
    back_btn.draw(screen)
    retry_btn.draw(screen)

# 儀表板繪製函數
def draw_dashboard(vehicle, game_back_btn, distance_optimal, time_optimal, speed_optimal_time):
    """繪製遊戲儀表板"""
    screen.fill(WHITE)
    
    # 左上角 - 地圖與剩餘距離
    map_radius = 125
    map_center = (map_radius, map_radius)
    pygame.draw.circle(screen, (245, 240, 230), map_center, map_radius)
    
    # 繪製路線
    line_width = 5
    pygame.draw.line(screen, GRAY, 
                    (map_center[0], map_center[1] - map_radius + 20),
                    (map_center[0], map_center[1] + map_radius - 20),
                    line_width)
    
    # 終點標記
    end_point_y = map_center[1] - map_radius + 20
    pygame.draw.circle(screen, RED, (map_center[0], end_point_y), 8)
    
    # 車輛位置
    car_pos_ratio = vehicle.position / distance_optimal[-1]
    path_length = (2 * map_radius - 40)
    car_y = map_center[1] + map_radius - 20 - path_length * car_pos_ratio
    pygame.draw.circle(screen, BLUE, (map_center[0], car_y), 8)
    
    # 剩餘距離
    remain_distance = distance_optimal[-1] - vehicle.position
    distance_font = pygame.font.Font(None, 30)
    if remain_distance <= 0:
        distance_text = "Arrived at destination!"
    else:
        distance_text = f"Remaining: {remain_distance:.1f} m"
    
    distance_surf = distance_font.render(distance_text, True, BLACK)
    text_rect = distance_surf.get_rect(center=(map_center[0], map_center[1] + map_radius + 25))
    screen.blit(distance_surf, text_rect)
    
    # 右上角 - 時鐘倒數計時
    clock_radius = 70
    clock_center = (width - clock_radius - 30, 100 + clock_radius + 10)
    pygame.draw.circle(screen, (0, 0, 40), clock_center, clock_radius)
    
    # 計算時間
    expected_arrival_time = 60  # 預設60秒
    time_diff = expected_arrival_time - vehicle.time
    is_delayed = time_diff < 0
    abs_time = abs(time_diff)
    minutes = int(abs_time // 60)
    seconds = int(abs_time % 60)
    
    # 顯示時間
    time_label_font = pygame.font.Font(None, 36)
    time_label = time_label_font.render("TIME", True, WHITE)
    label_rect = time_label.get_rect(center=(clock_center[0], clock_center[1] - 20))
    screen.blit(time_label, label_rect)
    
    time_font = pygame.font.Font(None, 48)
    time_text = f"{minutes:02d}:{seconds:02d}"
    time_surf = time_font.render(time_text, True, WHITE)
    text_rect = time_surf.get_rect(center=(clock_center[0], clock_center[1] + 10))
    screen.blit(time_surf, text_rect)
    
    # 進度圓弧
    if is_delayed:
        arc_color = RED
        progress_percentage = 1.0
    else:
        arc_color = GREEN
        progress_percentage = time_diff / expected_arrival_time if expected_arrival_time > 0 else 0
    
    start_angle = -math.pi / 2
    end_angle = start_angle + 2 * math.pi * progress_percentage
    arc_rect = (clock_center[0] - clock_radius, clock_center[1] - clock_radius, 
               2 * clock_radius, 2 * clock_radius)
    pygame.draw.arc(screen, arc_color, arc_rect, start_angle, end_angle, 5)
    
    # 中間 - 路線與火車
    route_y = height // 2
    route_start_x = 100
    route_end_x = width - 100
    total_distance = distance_optimal[-1]
    
    # 繪製路線
    pygame.draw.line(screen, GRAY, (route_start_x, route_y), (route_end_x, route_y), 8)
    
    # 刻度標記
    mark_count = 5
    for i in range(mark_count + 1):
        mark_distance = (total_distance / mark_count) * i
        mark_x = route_start_x + (route_end_x - route_start_x) * (mark_distance / total_distance)
        pygame.draw.line(screen, BLACK, (mark_x, route_y - 10), (mark_x, route_y + 10), 2)
        
        mark_font = pygame.font.Font(None, 24)
        mark_text = f"{int(mark_distance)}m"
        mark_surf = mark_font.render(mark_text, True, BLACK)
        mark_rect = mark_surf.get_rect(center=(mark_x, route_y + 25))
        screen.blit(mark_surf, mark_rect)
    
    # 終點旗幟
    flag_width, flag_height = 30, 40
    pygame.draw.line(screen, BLACK, (route_end_x, route_y), (route_end_x, route_y - flag_height), 3)
    pygame.draw.polygon(screen, RED, [
        (route_end_x, route_y - flag_height),
        (route_end_x + flag_width, route_y - flag_height + flag_height // 2),
        (route_end_x, route_y - flag_height + flag_height)
    ])
    
    # 火車
    train_pos_ratio = min(1.0, vehicle.position / total_distance)
    train_x = route_start_x + (route_end_x - route_start_x) * train_pos_ratio
    train_width, train_height = 50, 30
    train_y = route_y - train_height // 2 - 5
    
    # 火車車身
    pygame.draw.rect(screen, BLUE, (train_x - train_width // 2, train_y, train_width, train_height), border_radius=5)
    
    # 火車車頭
    head_radius = train_height // 2
    head_color = (0, 0, 150)
    head_x = train_x + train_width // 2 - head_radius // 2
    pygame.draw.circle(screen, head_color, (head_x, train_y + head_radius), head_radius)
    
    # 車窗
    window_width, window_height = 8, 10
    window_spacing = 12
    for i in range(3):
        window_x = train_x - train_width // 2 + 10 + i * window_spacing
        window_y = train_y + 5
        pygame.draw.rect(screen, WHITE, (window_x, window_y, window_width, window_height))
    
    # 車輪
    wheel_radius = 5
    wheel_spacing = train_width // 3
    for i in range(2):
        wheel_x = train_x - train_width // 4 + i * wheel_spacing
        wheel_y = train_y + train_height
        pygame.draw.circle(screen, BLACK, (wheel_x, wheel_y), wheel_radius)
    
    # 底部儀表板
    dashboard_width, dashboard_height = 600, 180
    dashboard_x = (width - dashboard_width) // 2
    dashboard_y = height - dashboard_height - 30
    
    # 儀表板背景
    pygame.draw.rect(screen, GRAY, 
                    (dashboard_x, dashboard_y, dashboard_width, dashboard_height),
                    border_radius=20)
    
    # 中間顯示區域
    display_width = dashboard_width - 180
    display_height = dashboard_height - 40
    display_x = dashboard_x + 60
    display_y = dashboard_y + 30
    
    pygame.draw.rect(screen, BLACK, (display_x, display_y, display_width, display_height))
    
    # 速度資訊
    current_speed = vehicle.speed
    recommended_speed = optimizer.get_dynamic_speed_recommendation(
        current_speed=vehicle.speed,
        remain_distance=distance_optimal[-1] - vehicle.position,
        current_time=vehicle.time
    )[1]
    
    # 顯示當前速度
    speed_font = pygame.font.Font(None, 45)
    speed_text = f"Speed: {int(current_speed)}(km/h)"
    speed_surf = speed_font.render(speed_text, True, WHITE)
    speed_x = display_x + (display_width * 0.6) - speed_surf.get_width() // 2
    speed_y = display_y + display_height // 2 - speed_surf.get_height() // 2
    screen.blit(speed_surf, (speed_x, speed_y))
    
    # 返回按鈕
    game_back_btn.draw(screen)

# 保存遊戲成績
def save_game_score(score, time, energy, distance):
    """保存遊戲成績到排行榜"""
    global ranking, last_game_score, last_game_time, last_game_energy, last_game_distance
    
    last_game_score = score
    last_game_time = time
    last_game_energy = energy
    last_game_distance = distance / 1000  # 轉換為公里
    
    ranking.append([score, time, energy])
    ranking.sort(key=lambda x: (-x[0], x[2], x[1]))
    
    # 寫入CSV檔
    with open(ranking_file, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerows(ranking)
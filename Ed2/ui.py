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
    DETAIL = 6

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
    detail_page_button = Button(120, 500, 210 , 50 , "See More...", LIGHT_GRAY , (100, 255, 100))
    return retry_button, result_back_button, detail_page_button

def create_detail_buttons():
    """創建查看更多畫面的按鈕"""
    detail_back_button = Button(width//2 - 60, 655, 350 , 50 , "Back to Result Page", GREEN , (100, 255, 100))
    speed_target_button = Button(width//2 - 430, 655, 350 , 50 , "Show Target Speed", LIGHT_GRAY , (100, 255, 100))
    return speed_target_button, detail_back_button

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

# 繪製查看更多頁面
def draw_detail_page(back_btn, speed_btn):

    screen.fill(DARK_GRAY)
        
    # 主面板
    panel_rect = pygame.Rect(20, 20, width - 40, height - 40)
    pygame.draw.rect(screen, LIGHT_GRAY, panel_rect)

    titlebg_rect = pygame.Rect(80, 75, 360, 75)
    pygame.draw.rect(screen, WHITE, titlebg_rect, border_radius=10)

    bodybg_rect = pygame.Rect(80, height//2-135, 360, 270)
    pygame.draw.rect(screen, WHITE, bodybg_rect, border_radius=10)

    if last_game_time == 60:
        status_text = "On Time! "
        status_color = GREEN
    elif last_game_time < 60:
        status_text = "Early Arrived! "
        status_color = YELLOW
    elif last_game_time > 60:
        status_text = "You're Late! "
        status_color = RED

    # 標題
    bgline_rect = pygame.Rect(80, 75, 360, 75)
    pygame.draw.rect(screen, BLACK, bgline_rect, width=3, border_radius=10)
    title_surface = title_font.render("Game Record", True, BLACK)
    screen.blit(title_surface, (95, 85))

    # 狀態（綠燈＋文字）
    pygame.draw.circle(screen, status_color, (270, height//2-85), 10)   
    screen.blit(normal_font.render(status_text, True, BLACK), (95, height//2-95))

    # 分數、完成時間等文字
    screen.blit(normal_font.render(f"Score: {last_game_score:.0f} pts", True, BLACK), (95, height//2-55))
    screen.blit(normal_font.render(f"Finished Time: {last_game_time:.1f}", True, BLACK), (95, height//2-15))
    screen.blit(normal_font.render(f"Distance: {last_game_distance:.1f} km", True, BLACK), (95, height//2+25))
    # screen.blit(normal_font.render(f"Average Speed: {avg_speed} km/h", True, BLACK), (95, info_y + 3 * info_spacing))
    screen.blit(normal_font.render(f"Total Energy: {last_game_energy:.1f} kWh", True, BLACK), (95, height//2+65))
    
    # 按鈕
    back_btn.draw(screen)
    speed_btn.draw(screen)

    pygame.display.flip()

# 繪製結果畫面
def draw_result(back_btn, retry_btn, detail_btn):
    """繪製結果畫面"""
    screen.fill(WHITE)
    
    # 標題
    # 成功標題與圖示
    if last_game_time <= 60:
        success_text = title_font.render("Success......", True, BLACK)
    else:
        success_text = title_font.render("Fail......", True, BLACK)
    success_rect = success_text.get_rect(center=(width//2, 100))
    screen.blit(success_text, success_rect)
    
    # --- 取得最佳值（從 optimizer）---
    optimal_energy = optimizer.optimal_result['optimal_energy']
    optimal_energy_val = float(optimal_energy[0]) if isinstance(optimal_energy, (list, np.ndarray)) else float(optimal_energy)

    total_score = max(0, (50 - abs(last_game_time - 60)) + 50*(optimal_energy_val / last_game_energy if last_game_energy != 0 else 0))

    # 遊戲結果資訊
    if last_game_time == 60:
        status_text = "On Time!"
        status_color = GREEN
        light_color = GREEN
    elif last_game_time < 60:
        status_text = "Early Arrived!"
        status_color = GREEN
        light_color = YELLOW
    else:
        status_text = "You're Late!"
        status_color = RED
        light_color = RED
    
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
    
    # 燈號
    pygame.draw.circle(screen, light_color, (295, height//2-70), 10)

    # 排行榜背景
    pygame.draw.rect(screen,BLACK,(width//2-30,height//2-145,490,310), border_radius=20)
    pygame.draw.rect(screen,LIGHT_GRAY,(width//2-25,height//2-140,480,300), border_radius=20)
    pygame.draw.rect(screen,WHITE,(width//2-10,height//2-83,450,71), border_radius=10)
    pygame.draw.rect(screen,WHITE,(width//2-10,height//2-3,450,71), border_radius=10)
    pygame.draw.rect(screen,WHITE,(width//2-10,height//2+77,450,71), border_radius=10)

    # 排行榜（只顯示前3名）
    y_offset = height//2-80
    light_y_offset = height//2-68
    line_spacing = 40  # 每行間距
    sorted_ranking = sorted(ranking, key=lambda x: (-x[0], x[2], x[1]))

    subtitle_text = normal_font.render("Leaderboard", True, BLACK)
    screen.blit(subtitle_text, (width//2+145, y_offset - line_spacing))

    for i, (total_score, time, energy) in enumerate(sorted_ranking[:3], start=1):
        # 資料
        if time == 60:
            record_color = GREEN
        elif time <60:
            record_color = YELLOW
        elif time >60:
            record_color = RED
        score_line = f"{i}. Score: {total_score:.0f} pts"
        time_line = f"Total time: {time:.1f} s, Energy: {energy:.1f} kWh"

        # 分別 render 每一行
        score_text = normal_font.render(score_line, True, BLACK)
        time_text = normal_font.render(time_line, True, BLACK)

        # 分別 blit 到畫面上
        screen.blit(score_text, (width//2, y_offset))
        screen.blit(time_text, (width//2, y_offset + line_spacing))
        y_offset += (line_spacing*2)
        pygame.draw.circle(screen, record_color, (width//2+210, light_y_offset), 10)
        light_y_offset += (line_spacing*2)

    # 按鈕
    back_btn.draw(screen)
    retry_btn.draw(screen)
    detail_btn.draw(screen)

# 儀表板繪製函數
def draw_dashboard(vehicle, game_back_btn, distance_optimal, time_optimal, speed_optimal_time):
    """繪製遊戲儀表板"""
    # Import math module
    import math
    
    # Define colors for dashboard
    BLACK = (0, 0, 0)
    WHITE = (255, 255, 255)
    RED = (255, 0, 0)
    GREEN = (0, 255, 0)
    BLUE = (0, 0, 255)
    YELLOW = (255, 255, 0)
    LIGHT_YELLOW = (255, 255, 200)
    GRAY = (128, 128, 128)
    
    screen.fill(WHITE)
    
    # 左上角 - 地圖與剩餘距離
    # -------------------------------
    # 圓形背景
    map_radius = 125
    map_center = (map_radius, map_radius)
    pygame.draw.circle(screen, (245, 240, 230), map_center, map_radius)
    
    # 繪製路線 - 垂直線
    line_width = 5
    pygame.draw.line(screen, GRAY, 
                    (map_center[0], map_center[1] - map_radius + 20),  # 上方位置（終點）
                    (map_center[0], map_center[1] + map_radius - 20),  # 下方位置（起點）
                    line_width)
    
    # 繪製終點 - 紅色點，位於上方
    end_point_y = map_center[1] - map_radius + 20  # 終點在上方
    pygame.draw.circle(screen, RED, (map_center[0], end_point_y), 8)
    
    # 車輛位置指示器 - 藍色點
    # 計算車輛位置比例（0表示起點，1表示終點）
    car_pos_ratio = vehicle.position / distance_optimal[-1]
    # 計算車輛在垂直線上的位置 - 從下方往上方移動
    path_length = (2 * map_radius - 40)  # 路徑總長度
    car_y = map_center[1] + map_radius - 20 - path_length * car_pos_ratio  # 從下往上移動
    pygame.draw.circle(screen, BLUE, (map_center[0], car_y), 8)
    
    
    # 計算剩餘距離
    remain_distance = distance_optimal[-1] - vehicle.position

    # 顯示剩餘距離或已到達
    distance_font = pygame.font.Font(None, 30)

    if remain_distance <= 0:
        # 如果剩餘距離小於或等於0，顯示"已到達"（英文）
        distance_text = "Arrived at destination!"
    else:
        # 否則顯示剩餘距離
        distance_text = f"Remaining: {remain_distance:.1f} m"

    distance_surf = distance_font.render(distance_text, True, BLACK)
    text_rect = distance_surf.get_rect(center=(map_center[0], map_center[1] + map_radius + 25))
    screen.blit(distance_surf, text_rect)
    
    # -------------------------------
    # 右上角 - 時鐘與剩餘/延遲時間
    # -------------------------------
    # 選單按鈕 - 使用game_back_btn替代
    game_back_btn.draw(screen)

    # 時鐘
    clock_radius = 70
    clock_center = (width - clock_radius - 30, 30 + 50 + clock_radius + 10)

    # 繪製時鐘外圈（深藍色）
    pygame.draw.circle(screen, (0, 0, 40), clock_center, clock_radius)

    # 計算時間數據
    expected_arrival_time = optimizer.expected_arrival_time if hasattr(optimizer, 'expected_arrival_time') else 60  # 預設1分鐘
    time_diff = expected_arrival_time - vehicle.time  # 正值表示剩餘時間，負值表示延遲

    # 確定是剩餘時間還是延遲
    is_delayed = time_diff < 0
    abs_time = abs(time_diff)  # 取絕對值用於顯示
    minutes = int(abs_time // 60)
    seconds = int(abs_time % 60)

    # 顯示TIME文字 - 白色
    time_label_font = pygame.font.Font(None, 36)
    time_label = time_label_font.render("TIME", True, WHITE)
    label_rect = time_label.get_rect(center=(clock_center[0], clock_center[1] - 20))
    screen.blit(time_label, label_rect)

    # 顯示時間 - 白色大字體
    time_font = pygame.font.Font(None, 48)
    time_text = f"{minutes:02d}:{seconds:02d}"
    time_surf = time_font.render(time_text, True, WHITE)
    text_rect = time_surf.get_rect(center=(clock_center[0], clock_center[1] + 10))
    screen.blit(time_surf, text_rect)

    # 確定進度圓弧的顏色和文本
    if is_delayed:
        # 延遲 - 使用紅色
        arc_color = RED
        # 在時鐘下方顯示"Delay"文字
        delay_font = pygame.font.Font(None, 30)
        delay_text = f"Delay: {minutes:02d}:{seconds:02d}"
        delay_surf = delay_font.render(delay_text, True, RED)
        delay_rect = delay_surf.get_rect(center=(clock_center[0], clock_center[1] + clock_radius + 20))
        screen.blit(delay_surf, delay_rect)
        # 延遲時進度圓是完整的
        progress_percentage = 1.0
    else:
        # 正常倒計時 - 使用綠色
        arc_color = GREEN
        # 計算進度比例 - 從1減少到0
        progress_percentage = time_diff / expected_arrival_time if expected_arrival_time > 0 else 0

    # 繪製進度圈 - 從頂部開始，順時針方向
    start_angle = -math.pi / 2
    end_angle = start_angle + 2 * math.pi * progress_percentage

    # 計算圓弧的矩形區域
    arc_rect = (clock_center[0] - clock_radius, clock_center[1] - clock_radius, 
               2 * clock_radius, 2 * clock_radius)

    # 繪製進度圓弧（顏色根據是否延遲決定）
    pygame.draw.arc(screen, arc_color, arc_rect, start_angle, end_angle, 5)
    
    # -------------------------------
    # 畫面中間 - 路線與火車
    # -------------------------------
    # 設定路線參數
    route_y = height // 2  # 路線在畫面中間高度
    route_start_x = 100  # 左側起點
    route_end_x = width - 100  # 右側終點
    total_distance = distance_optimal[-1]  # 使用實際的總距離

    # 繪製路線(灰色)
    pygame.draw.line(screen, GRAY, (route_start_x, route_y), (route_end_x, route_y), 8)

    # 在路線上標記刻度（使用固定數量的刻度點，而不是用range和間隔）
    mark_count = 5  # 希望顯示的刻度數量
    for i in range(mark_count + 1):
        # 計算刻度位置和對應的距離值
        mark_distance = (total_distance / mark_count) * i
        mark_x = route_start_x + (route_end_x - route_start_x) * (mark_distance / total_distance)
    
        # 繪製刻度線
        pygame.draw.line(screen, BLACK, (mark_x, route_y - 10), (mark_x, route_y + 10), 2)
    
        # 標記距離文字
        mark_font = pygame.font.Font(None, 24)
        mark_text = f"{int(mark_distance)}m"
        mark_surf = mark_font.render(mark_text, True, BLACK)
        mark_rect = mark_surf.get_rect(center=(mark_x, route_y + 25))
        screen.blit(mark_surf, mark_rect)
    
    # 繪製終點標誌(紅色旗幟)
    flag_width, flag_height = 30, 40
    flag_x = route_end_x - flag_width // 2
    flag_y = route_y - flag_height - 5

    # 旗幟桿
    pygame.draw.line(screen, BLACK, (route_end_x, route_y), (route_end_x, route_y - flag_height), 3)
    # 旗幟(紅色三角形)
    pygame.draw.polygon(screen, RED, [
        (route_end_x, route_y - flag_height),
        (route_end_x + flag_width, route_y - flag_height + flag_height // 2),
        (route_end_x, route_y - flag_height + flag_height)
    ])

    # 計算火車位置
    train_pos_ratio = min(1.0, vehicle.position / total_distance)  # 確保不超過終點
    train_x = route_start_x + (route_end_x - route_start_x) * train_pos_ratio

    # 繪製火車(藍色)
    train_width, train_height = 50, 30
    train_y = route_y - train_height // 2 - 5  # 將火車放在軌道上方

    # 火車車身(藍色矩形)
    pygame.draw.rect(screen, BLUE, (train_x - train_width // 2, train_y, train_width, train_height), border_radius=5)

    # 火車車頭(深藍色半圓)
    head_radius = train_height // 2
    head_color = (0, 0, 150)  # 深藍色
    head_x = train_x + train_width // 2 - head_radius // 2
    pygame.draw.circle(screen, head_color, (head_x, train_y + head_radius), head_radius)

    # 車窗(白色小矩形)
    window_width, window_height = 8, 10
    window_spacing = 12
    for i in range(3):
        window_x = train_x - train_width // 2 + 10 + i * window_spacing
        window_y = train_y + 5
        pygame.draw.rect(screen, WHITE, (window_x, window_y, window_width, window_height))

    # 車輪(黑色圓圈)
    wheel_radius = 5
    wheel_spacing = train_width // 3
    for i in range(2):
        wheel_x = train_x - train_width // 4 + i * wheel_spacing
        wheel_y = train_y + train_height
        pygame.draw.circle(screen, BLACK, (wheel_x, wheel_y), wheel_radius)

    # 顯示當前位置
    position_font = pygame.font.Font(None, 30)
    position_text = f"Position: {vehicle.position:.1f}m / {total_distance:.1f}m"
    position_surf = position_font.render(position_text, True, BLACK)
    position_rect = position_surf.get_rect(center=(width // 2, route_y - 40))
    screen.blit(position_surf, position_rect)
    
    # -------------------------------
    # 底部儀表板
    # -------------------------------
    # 儀表板背景
    dashboard_width, dashboard_height = 600, 180
    dashboard_x = (width - dashboard_width) // 2
    dashboard_y = height - dashboard_height - 30

    # 繪製儀表板背景（圓角灰色矩形）
    pygame.draw.rect(screen, GRAY, 
                (dashboard_x, dashboard_y, dashboard_width, dashboard_height),
                border_radius=20)

    # 狀態指示燈（上方一排）
    light_radius = 10
    light_spacing = 30
    light_y = dashboard_y + 15
    light_start_x = dashboard_x + 30

    # 紅、黃、黃、紅燈配置
    light_colors = [RED, YELLOW, YELLOW, RED]
    for i, color in enumerate(light_colors):
        pygame.draw.circle(screen, color, (light_start_x + i * light_spacing, light_y), light_radius)

    # 左側燈號（綠、紅、黑、黃）
    side_light_colors = [GREEN, RED, BLACK, YELLOW]
    side_light_spacing = 40
    for i, color in enumerate(side_light_colors):
        pygame.draw.circle(screen, color, (dashboard_x + 15, dashboard_y + 40 + i * side_light_spacing), light_radius)

    # 中間顯示區（黑色矩形）
    display_width = dashboard_width - 180
    display_height = dashboard_height - 40
    display_x = dashboard_x + 60
    display_y = dashboard_y + 30

    pygame.draw.rect(screen, BLACK, (display_x, display_y, display_width, display_height))

    # 獲取當前速度和建議速度
    current_speed = vehicle.speed
    recommended_speed = optimizer.get_dynamic_speed_recommendation(
        current_speed=vehicle.speed,
        remain_distance=distance_optimal[-1] - vehicle.position,
        current_time=vehicle.time
    )[1]  # 獲取建議速度值

    # 左側圓形儀表 - 速度表
    gauge_radius = 50
    gauge_center = (dashboard_x + 130, dashboard_y + dashboard_height//2 + 10)

    # 速度表背景（白色圓）
    pygame.draw.circle(screen, WHITE, gauge_center, gauge_radius)

    # 繪製刻度線（可選）
    for i in range(0, 121, 20):  # 假設最大速度120km/h，每20km/h一個刻度
        angle = -math.pi / 2 + (i / 120) * 2 * math.pi  # 從-90度開始，順時針旋轉
        start_x = gauge_center[0] + (gauge_radius - 10) * math.cos(angle)
        start_y = gauge_center[1] + (gauge_radius - 10) * math.sin(angle)
        end_x = gauge_center[0] + gauge_radius * math.cos(angle)
        end_y = gauge_center[1] + gauge_radius * math.sin(angle)
        pygame.draw.line(screen, BLACK, (start_x, start_y), (end_x, end_y), 2)
    
    # 繪製指針（紅色）
    current_angle = -math.pi / 2 + (current_speed / 120) * 2 * math.pi
    needle_length = gauge_radius - 10
    needle_end_x = gauge_center[0] + needle_length * math.cos(current_angle)
    needle_end_y = gauge_center[1] + needle_length * math.sin(current_angle)
    pygame.draw.line(screen, RED, gauge_center, (needle_end_x, needle_end_y), 3)
    
    # 速度值在圓形儀表中間
    speed_value_font = pygame.font.Font(None, 46)
    speed_value_text = f"{int(current_speed)}"
    speed_value_surf = speed_value_font.render(speed_value_text, True, BLACK)
    speed_value_rect = speed_value_surf.get_rect(center=gauge_center)
    screen.blit(speed_value_surf, speed_value_rect)

    # 在黑色區域中間偏右顯示當前速度
    speed_font = pygame.font.Font(None, 45)
    speed_text = f"Speed: {int(current_speed)}(km/h)"
    speed_surf = speed_font.render(speed_text, True, WHITE)
    # 放置在偏右位置
    speed_x = display_x + (display_width * 0.6) - speed_surf.get_width() // 2
    speed_y = display_y + display_height // 2 - speed_surf.get_height() // 2
    screen.blit(speed_surf, (speed_x, speed_y))
    
    # ========== 能耗與目標速度指示燈 ==========
    # 取得最佳能耗曲線與距離
    cumulative_energy, distance_m = optimizer.calculate_cumulative_energy_based_on_distance(optimizer.optimal_result)
    min_len = min(len(distance_m), len(cumulative_energy))
    distance_m = distance_m[:min_len]
    cumulative_energy = cumulative_energy[:min_len]

    # 使用目前距離位置做線性插值，找出對應最佳能耗
    optimal_energy_at_current_position = np.interp(
        vehicle.position,      # x: 目前位置
        distance_m,            # x 座標：最佳距離資料
        cumulative_energy      # y 座標：最佳能耗曲線
    )

    # 比較目前能耗與該點最佳能耗，決定燈號
    energy_light_color = GREEN if vehicle.energy_consumption <= optimal_energy_at_current_position else RED

    # 能耗指示燈
    energy_light_radius = 10
    energy_light_x = display_x + display_width - 150  # 根據畫面向左調整，避免太靠邊
    energy_light_y = display_y + 24

    # 繪製圓形燈
    pygame.draw.circle(screen, energy_light_color, (energy_light_x, energy_light_y), energy_light_radius)

    # "Energy" 標籤文字（在圓圈左邊）
    energy_label_font = pygame.font.Font(None, 24)
    energy_label_text = "Energy"
    energy_label_surf = energy_label_font.render(energy_label_text, True, WHITE)
    energy_label_x = energy_light_x - energy_label_surf.get_width() -15
    energy_label_y = energy_light_y - energy_label_surf.get_height() // 2
    screen.blit(energy_label_surf, (energy_label_x, energy_label_y))
    
    # Target Speed 標籤（在圓圈右邊）
    target_speed_font = pygame.font.Font(None, 24)
    target_speed_text = "Target speed"
    target_speed_surf = target_speed_font.render(target_speed_text, True, WHITE)
    target_speed_x = energy_light_x + energy_light_radius + 10
    target_speed_y = energy_light_y - target_speed_surf.get_height() // 2
    screen.blit(target_speed_surf, (target_speed_x, target_speed_y))

    # 箭頭（在 target speed 右邊）
    arrow_base_x = target_speed_x + target_speed_surf.get_width() + 10
    arrow_y_top = energy_light_y - 5
    arrow_y_bottom = energy_light_y + 5

    if recommended_speed > current_speed:
        # 加速 → 紅色向上箭頭
        arrow_color = RED
        arrow_points = [
            (arrow_base_x, arrow_y_top),
            (arrow_base_x - 5, arrow_y_bottom),
            (arrow_base_x + 5, arrow_y_bottom)
        ]
    else:
        # 減速 → 綠色向下箭頭
        arrow_color = GREEN
        arrow_points = [
            (arrow_base_x, arrow_y_bottom),
            (arrow_base_x - 5, arrow_y_top),
            (arrow_base_x + 5, arrow_y_top)
        ]

    # 繪製箭頭
    pygame.draw.polygon(screen, arrow_color, arrow_points)
    
    # -------------------------------
    # 前方速限提示 - 隨距離倒數
    # -------------------------------
    # 獲取當前速度和速限
    current_speed = vehicle.speed  # 轉換為 km/h
    current_speed_limit = optimizer.get_speed_limit_at_position(vehicle.position) * 3.6  # 轉換為 km/h

    # 檢查是否超速
    is_speeding = current_speed > current_speed_limit

    # 遍歷所有速限變化點
    next_speed_limit_pos = None
    next_speed_limit_value = None

    # 查找下一個速限變化點
    sorted_limits = sorted(optimizer.speed_limits)
    for limit_pos, limit_speed in sorted_limits:
        if limit_pos > vehicle.position:
            next_speed_limit_pos = limit_pos
            next_speed_limit_value = limit_speed * 3.6  # 轉換為 km/h
            break

    # 繪製提示（超速提示或前方速限提示）
    warning_x = width // 2 - 180
    warning_y = height // 4 - 40

    # 如果超速，顯示超速警告
    if is_speeding:
        # 繪製警告三角形（紅色表示超速）
        warning_size = 20
        warning_points = [
            (warning_x - warning_size, warning_y + warning_size),  # 左下
            (warning_x, warning_y - warning_size),                # 頂部
            (warning_x + warning_size, warning_y + warning_size)   # 右下
        ]
        pygame.draw.polygon(screen, RED, warning_points)  # 紅色表示超速警告
        pygame.draw.polygon(screen, BLACK, warning_points, 2)  # 黑色邊框
        
        # 在三角形內部繪製驚嘆號
        exclamation_font = pygame.font.Font(None, 24)
        exclamation_surf = exclamation_font.render("!", True, BLACK)
        exclamation_rect = exclamation_surf.get_rect(center=(warning_x, warning_y))
        screen.blit(exclamation_surf, exclamation_rect)
        
        # 超速警告文字
        info_font = pygame.font.Font(None, 28)
        speeding_text = f"Speeding! Current: {int(current_speed)} km/h, Limit: {int(current_speed_limit)} km/h"
        speeding_surf = info_font.render(speeding_text, True, RED)  # 紅色文字
        speeding_rect = speeding_surf.get_rect(midleft=(warning_x + warning_size + 10, warning_y))
        screen.blit(speeding_surf, speeding_rect)

    # 如果找到了下一個速限變化點
    elif next_speed_limit_pos is not None:
        # 計算到下一個速限點的距離
        distance_to_next = next_speed_limit_pos - vehicle.position
        
        # 如果距離在200米內，顯示提示
        if 0 < distance_to_next <= 200:
            # 繪製警告三角形（黃色表示前方速限變化）
            warning_size = 20
            warning_points = [
                (warning_x - warning_size, warning_y + warning_size),  # 左下
                (warning_x, warning_y - warning_size),                # 頂部
                (warning_x + warning_size, warning_y + warning_size)   # 右下
            ]
            pygame.draw.polygon(screen, YELLOW, warning_points)
            pygame.draw.polygon(screen, BLACK, warning_points, 2)  # 黑色邊框
            
            # 在三角形內部繪製驚嘆號
            exclamation_font = pygame.font.Font(None, 24)
            exclamation_surf = exclamation_font.render("!", True, BLACK)
            exclamation_rect = exclamation_surf.get_rect(center=(warning_x, warning_y))
            screen.blit(exclamation_surf, exclamation_rect)
            
            # 前方速限提示文字
            info_font = pygame.font.Font(None, 28)
            ahead_text = f"Ahead {int(distance_to_next)}m, Speed Limit: {int(next_speed_limit_value)} (km/h)"
            ahead_surf = info_font.render(ahead_text, True, BLACK)
            ahead_rect = ahead_surf.get_rect(midleft=(warning_x + warning_size + 10, warning_y))
            screen.blit(ahead_surf, ahead_rect)
# 保存遊戲成績
def save_game_score(score, time, energy, distance):
    """保存遊戲成績到排行榜"""
    global ranking, last_game_score, last_game_time,last_game_energy, last_game_distance
    
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
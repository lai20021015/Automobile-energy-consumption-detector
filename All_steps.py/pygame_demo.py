import pygame
import numpy as np
import matplotlib.pyplot as plt
from train_energy_optimizer import TrainEnergyOptimizer

plt.rcParams['font.sans-serif'] = ['Microsoft JhengHei'] 
plt.rcParams['axes.unicode_minus'] = False

class TrainSimulator:
    def __init__(self):
        pygame.init()
        self.width = 1200
        self.height = 800
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption("列車模擬器")
        
        # 時間和物理參數
        self.clock = pygame.time.Clock()
        self.dt = 0.1  # 時間步長
        
        # 列車參數
        self.train = {
            'position': 0,     # 位置(m)
            'speed': 0,        # 速度(m/s)
            'acceleration': 0,  # 加速度(m/s^2)
            'max_speed': 30.0, # 最大速度(m/s)
            'max_accel': 1.1,  # 最大加速度(m/s^2)
            'energy': 0        # 累積能耗(kWh)
        }
        
        # 創建優化器和獲取最佳速度曲線
        self.optimizer = TrainEnergyOptimizer(
            distance_m=1000.0,   # 1公里
            time_s=60.0,         # 60秒
            max_speed_mps=self.train['max_speed'],
            max_accel=self.train['max_accel'],
            control_points=5,
            veh_id=43
        )
        
        # 獲取最佳曲線
        self.optimal_results = self.optimizer.optimize()
        
        # 初始化實際運行數據記錄
        self.actual_profile = {
            'time': [0],
            'speed': [0],
            'position': [0],
            'energy': [0]
        }
        
        # 字體初始化
        self.font = pygame.font.Font(None, 36)

    def handle_input(self):
        """處理輸入控制"""
        keys = pygame.key.get_pressed()
        
        if keys[pygame.K_UP]:  # 加速
            self.train['acceleration'] = min(self.train['max_accel'],
                                          self.train['acceleration'] + 0.1)
        elif keys[pygame.K_DOWN]:  # 減速/煞車
            self.train['acceleration'] = max(-self.train['max_accel'],
                                          self.train['acceleration'] - 0.1)
        else:  # 慣性/阻力減速
            self.train['acceleration'] = -0.1 * (self.train['speed'] / self.train['max_speed'])

    def calculate_energy(self, speed, acceleration):
        """計算瞬時能耗（簡化模型）"""
        mass = 1000  # 假設質量1000kg
        # 計算動能變化和阻力功
        kinetic_energy = 0.5 * mass * acceleration * speed
        resistance = 0.01 * mass * 9.81 * speed  # 簡化的阻力模型
        return max(0, kinetic_energy + resistance) * self.dt / 3600000  # 轉換為kWh

    def update_physics(self):
        """更新物理狀態"""
        # 更新速度和位置
        self.train['speed'] += self.train['acceleration'] * self.dt
        self.train['speed'] = np.clip(self.train['speed'], 0, self.train['max_speed'])
        self.train['position'] += self.train['speed'] * self.dt
        
        # 計算能耗
        energy = self.calculate_energy(self.train['speed'], self.train['acceleration'])
        self.train['energy'] += energy
        
        # 記錄實際運行數據
        self.actual_profile['time'].append(self.actual_profile['time'][-1] + self.dt)
        self.actual_profile['speed'].append(self.train['speed'])
        self.actual_profile['position'].append(self.train['position'])
        self.actual_profile['energy'].append(self.train['energy'])

    def draw_speed_comparison_chart(self):
        """繪製速度比較圖表"""
        chart_x = 50
        chart_y = 50
        chart_width = 500
        chart_height = 200
        
        # 繪製圖表背景和框架
        pygame.draw.rect(self.screen, (240, 240, 240), 
                        (chart_x, chart_y, chart_width, chart_height))
        pygame.draw.rect(self.screen, (0, 0, 0), 
                        (chart_x, chart_y, chart_width, chart_height), 1)
        
        # 繪製最佳速度曲線
        optimal_time = self.optimal_results['optimal_time']
        optimal_speed = self.optimal_results['optimal_speed']
        
        for i in range(len(optimal_time)-1):
            x1 = chart_x + (optimal_time[i]/60.0) * chart_width
            y1 = chart_y + chart_height - (optimal_speed[i]/self.train['max_speed']) * chart_height
            x2 = chart_x + (optimal_time[i+1]/60.0) * chart_width
            y2 = chart_y + chart_height - (optimal_speed[i+1]/self.train['max_speed']) * chart_height
            pygame.draw.line(self.screen, (0, 255, 0), (x1, y1), (x2, y2), 2)
            
        # 繪製實際速度曲線
        if len(self.actual_profile['time']) > 1:
            for i in range(len(self.actual_profile['time'])-1):
                x1 = chart_x + (self.actual_profile['time'][i]/60.0) * chart_width
                y1 = chart_y + chart_height - (self.actual_profile['speed'][i]/self.train['max_speed']) * chart_height
                x2 = chart_x + (self.actual_profile['time'][i+1]/60.0) * chart_width
                y2 = chart_y + chart_height - (self.actual_profile['speed'][i+1]/self.train['max_speed']) * chart_height
                pygame.draw.line(self.screen, (255, 0, 0), (x1, y1), (x2, y2), 2)

    def draw_energy_comparison(self):
        """繪製能耗比較"""
        energy_x = 50
        energy_y = 300
        
        # 顯示實際能耗
        actual_text = self.font.render(f"實際能耗: {self.train['energy']:.3f} kWh", 
                                     True, (255, 0, 0))
        self.screen.blit(actual_text, (energy_x, energy_y))
        
        # 顯示最佳能耗
        optimal_text = self.font.render(f"最佳能耗: {self.optimal_results['optimal_energy']:.3f} kWh", 
                                      True, (0, 255, 0))
        self.screen.blit(optimal_text, (energy_x, energy_y + 40))

    def draw(self):
        """繪製畫面"""
        self.screen.fill((255, 255, 255))
        
        # 繪製軌道
        pygame.draw.line(self.screen, (0, 0, 0),
                        (50, self.height-100),
                        (self.width-50, self.height-100), 5)
        
        # 繪製列車
        train_pos_x = 50 + (self.width-100) * (self.train['position']/1000.0)
        pygame.draw.rect(self.screen, (0, 0, 255),
                        (train_pos_x, self.height-120, 100, 40))
        
        # 顯示累積行駛距離
        distance_text = self.font.render(f"累積行駛: {self.train['position']:.1f} m", 
                                        True, (0, 0, 0))
        self.screen.blit(distance_text, (50, self.height-80))  # 位置可以調整
        # 繪製速度比較圖表
        self.draw_speed_comparison_chart()
        
        # 繪製能耗比較
        self.draw_energy_comparison()
        
        # 顯示當前速度
        speed_text = self.font.render(f"當前速度: {self.train['speed']*3.6:.1f} km/h", 
                                    True, (0, 0, 0))
        self.screen.blit(speed_text, (50, self.height-50))
        
        pygame.display.flip()

    def run(self):
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                    
            self.handle_input()
            self.update_physics()
            self.draw()
            
            self.clock.tick(60)
            
        pygame.quit()

if __name__ == "__main__":
    sim = TrainSimulator()
    sim.run()
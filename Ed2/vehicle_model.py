import numpy as np

class Vehicle:
    #定義汽車駕駛參數
    def __init__(self):
        self.position = 0.0  # 米
        self.speed = 0.0     # km/h
        self.acceleration = 0.0  # km/h/s
        self.time = 0.0      # 秒
        self.energy_consumption = 0.0  # kWh

    def update(self, dt, acceleration):
        """更新車輛狀態"""
        self.acceleration = acceleration
        self.speed += self.acceleration * dt
        self.speed = max(0, self.speed)  # 確保速度不為負
        self.position += self.speed * dt / 3.6  # 轉換為米/秒
        self.time += dt
        
        # 簡單的能耗模型
        # 耗能 = 0.01 * 速度^2 + 0.2 * 加速度^2
        self.energy_consumption += (0.01 * self.speed**2 + 0.2 * self.acceleration**2) * dt

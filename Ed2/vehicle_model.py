import numpy as np
import fastsim as fsim
class Vehicle:
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
         # 使用RF模型預測能耗而不是簡化公式
        if hasattr(self, 'rf_model'):
            # 準備模型輸入特徵
            # 特徵1: 平均速度 (m/s)
            # 特徵2: 時間步長 (s)
            # 特徵3: 坡度 (假設為平坦)
            grade = 0.0
            
            # 組織特徵數組
            features = np.array([[self.speed, dt, grade]])
            
            # 預測區間能耗
            energy_increment = self.rf_model.predict(features)[0]
            
            # 更新總能耗
            self.energy_consumption += energy_increment
        else:
            # 如果模型不可用，嘗試載入它
        
            import joblib
            import os
            
            # 定義可能的模型路徑
            model_paths = [
                "models/RFmodel_v1.joblib",
                "src/models/RFmodel_v1.joblib",
                os.path.join(os.path.dirname(__file__), "models/RFmodel_v1.joblib")
            ]
            
            # 嘗試加載模型
            for path in model_paths:
                if os.path.exists(path):
                    self.rf_model = joblib.load(path)
                    print(f"成功載入能耗預測模型：{path}")
                    break
            
            # 如果無法載入模型，使用原始計算方式
            if not hasattr(self, 'rf_model'):
                raise FileNotFoundError("找不到RF模型文件")
                
            # 使用剛載入的模型重新計算這一步的能耗
            features = np.array([[self.speed, dt, 0.0]])
            energy_increment = self.rf_model.predict(features)[0]
            self.energy_consumption += energy_increment
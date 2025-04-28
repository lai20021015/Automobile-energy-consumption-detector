import pandas as pd
import numpy as np
from sklearn.ensemble import RandomForestRegressor
from sklearn.model_selection import train_test_split
from sklearn.metrics import mean_absolute_error, r2_score, mean_squared_error
import joblib
import os
import time

def train_energy_surrogate(data_file="energy_dataset.csv", model_dir="models", model_name="RFmodel_v1.joblib"):
    """
    訓練輕量級隨機森林能源消耗代理模型
    
    Args:
        data_file: 數據文件路徑
        model_dir: 模型保存目錄
        model_name: 模型檔名
    """
    # 創建模型保存目錄
    os.makedirs(model_dir, exist_ok=True)
    
    # 讀取數據
    print(f"讀取數據: {data_file}")
    df = pd.read_csv(data_file)
    
    # 數據概覽
    print("\n=== 資料概況 ===")
    print(f"總樣本數: {len(df)}")
    print(f"特徵: {list(df.columns)}")
    
    # 準備特徵和標籤
    X = df[['ConstantSpeed_mps', 'SimulationDuration_s', 'RoadGradeFactor']].values
    y = df['EnergyConsumption'].values
    
    # 分割訓練集和測試集
    X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)
    print(f"\n訓練集大小: {len(X_train)} 筆")
    print(f"測試集大小: {len(X_test)} 筆")
    
    # 定義輕量級隨機森林模型
    print("\n使用輕量級隨機森林 (專注於預測速度)")
    model = RandomForestRegressor(
        n_estimators=20,       # 減少樹的數量
        max_depth=8,           # 減少樹的深度
        min_samples_split=10,  # 增加分割所需的最小樣本數
        min_samples_leaf=5,    # 增加葉節點所需的最小樣本數
        random_state=42,
        n_jobs=-1              # 使用所有CPU核心
    )
    
    # 訓練模型
    print("\n開始訓練輕量級隨機森林模型...")
    start_time = time.time()
    model.fit(X_train, y_train)
    training_time = time.time() - start_time
    print(f"訓練完成! 耗時: {training_time:.2f} 秒")
    
    # 測試預測速度
    print("\n測試預測速度...")
    n_repeats = 1000
    start_time = time.time()
    for _ in range(n_repeats):
        model.predict(X_test[:1])  # 只預測一個樣本
    inference_time = (time.time() - start_time) / n_repeats
    print(f"單次預測平均耗時: {inference_time*1000:.4f} 毫秒")
    
    # 評估模型
    print("\n=== 模型評估 ===")
    
    # 測試集評估
    y_pred_test = model.predict(X_test)
    mae_test = mean_absolute_error(y_test, y_pred_test)
    rmse_test = np.sqrt(mean_squared_error(y_test, y_pred_test))
    r2_test = r2_score(y_test, y_pred_test)
    
    print("\n測試集性能:")
    print(f"平均絕對誤差 (MAE): {mae_test:.4f} kWh")
    print(f"均方根誤差 (RMSE): {rmse_test:.4f} kWh")
    print(f"決定係數 (R²): {r2_test:.4f}")
    
    # 特徵重要性
    print("\n=== 特徵重要性 ===")
    feature_importance = model.feature_importances_
    features = ['Speed', 'Duration', 'Road Grade']
    for i, feature in enumerate(features):
        print(f"{feature:12s}: {feature_importance[i]:.4f}")
    
    # 保存模型
    model_path = os.path.join(model_dir, model_name)
    joblib.dump(model, model_path)
    print(f"\n模型已保存至: {model_path}")
    
    # 保存模型元數據
    metadata = {
        "model_type": "lightweight_random_forest",
        "model_file": model_name,
        "sample_count": len(df),
        "mae_test": float(mae_test),
        "rmse_test": float(rmse_test),
        "r2_test": float(r2_test),
        "training_time_seconds": training_time,
        "inference_time_ms": float(inference_time*1000),
        "feature_importance": {features[i]: float(feature_importance[i]) for i in range(len(features))},
        "model_params": {
            "n_estimators": 20,
            "max_depth": 8,
            "min_samples_split": 10,
            "min_samples_leaf": 5
        }
    }
    
    import json
    metadata_path = os.path.join(model_dir, "RFmodel_v1_metadata.json")
    with open(metadata_path, "w") as f:
        json.dump(metadata, f, indent=4)
    print(f"模型元資料已保存至: {metadata_path}")
    
    # 輸出預測樣本
    print("\n=== 預測樣本展示 ===")
    sample_indices = np.random.choice(len(X_test), min(5, len(X_test)), replace=False)
    
    print("\n速度(m/s)  時間(s)  坡度      實際能耗   預測能耗   誤差(kWh)  誤差(%)")
    print("---------------------------------------------------------------------")
    for idx in sample_indices:
        speed, duration, grade = X_test[idx]
        actual = y_test[idx]
        predicted = y_pred_test[idx]
        error = predicted - actual
        error_pct = (error / actual) * 100 if actual != 0 else float('inf')
        
        print(f"{speed:8.2f}  {duration:7.1f}  {grade:8.4f}  {actual:9.4f}  {predicted:9.4f}  {error:10.4f}  {error_pct:7.2f}%")
    
    print("\n訓練完成! RFmodel_v1已準備就緒。")
    
    return model, metadata

if __name__ == "__main__":
    train_energy_surrogate()
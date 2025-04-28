# train_surrogate.py
import pandas as pd
import numpy as np
from sklearn.ensemble import RandomForestRegressor
from sklearn.model_selection import train_test_split
from sklearn.metrics import mean_absolute_error, r2_score, mean_squared_error
import joblib
import os
import time

def train_energy_surrogate(data_file="energy_dataset.csv", model_dir="models"):
    # 創建模型保存目錄
    os.makedirs(model_dir, exist_ok=True)
    
    # 讀取數據
    print(f"Reading data from {data_file}")
    df = pd.read_csv(data_file)
    
    # 數據概覽
    print("\n=== 資料概況 ===")
    print(f"總樣本數: {len(df)}")
    print(f"特徵: {list(df.columns)}")
    print("\n資料統計摘要:")
    print(df.describe())
    
    # 準備特徵和標籤
    X = df[['ConstantSpeed_mps', 'SimulationDuration_s', 'RoadGradeFactor']].values
    y = df['EnergyConsumption'].values
    
    # 分割訓練集和測試集
    X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)
    print(f"\n訓練集大小: {len(X_train)} 筆")
    print(f"測試集大小: {len(X_test)} 筆")
    
    # 訓練隨機森林模型
    print("\n開始訓練隨機森林模型...")
    start_time = time.time()
    
    model = RandomForestRegressor(
        n_estimators=100, 
        max_depth=15,
        min_samples_split=5,
        min_samples_leaf=2,
        random_state=42,
        n_jobs=-1  # 使用所有CPU核心
    )
    
    model.fit(X_train, y_train)
    
    training_time = time.time() - start_time
    print(f"訓練完成! 耗時: {training_time:.2f} 秒")
    
    # 評估模型
    print("\n=== 模型評估 ===")
    start_time = time.time()
    
    # 訓練集評估
    y_pred_train = model.predict(X_train)
    mae_train = mean_absolute_error(y_train, y_pred_train)
    rmse_train = np.sqrt(mean_squared_error(y_train, y_pred_train))
    r2_train = r2_score(y_train, y_pred_train)
    
    # 測試集評估
    y_pred_test = model.predict(X_test)
    mae_test = mean_absolute_error(y_test, y_pred_test)
    rmse_test = np.sqrt(mean_squared_error(y_test, y_pred_test))
    r2_test = r2_score(y_test, y_pred_test)
    
    eval_time = time.time() - start_time
    
    print("\n訓練集性能:")
    print(f"平均絕對誤差 (MAE): {mae_train:.4f} kWh")
    print(f"均方根誤差 (RMSE): {rmse_train:.4f} kWh")
    print(f"決定係數 (R²): {r2_train:.4f}")
    
    print("\n測試集性能:")
    print(f"平均絕對誤差 (MAE): {mae_test:.4f} kWh")
    print(f"均方根誤差 (RMSE): {rmse_test:.4f} kWh")
    print(f"決定係數 (R²): {r2_test:.4f}")
    
    print(f"\n評估耗時: {eval_time:.2f} 秒")
    
    # 特徵重要性
    print("\n=== 特徵重要性 ===")
    feature_importance = model.feature_importances_
    features = ['Speed', 'Duration', 'Road Grade']
    for i, feature in enumerate(features):
        print(f"{feature:12s}: {feature_importance[i]:.4f}")
    
    # 保存模型
    model_path = os.path.join(model_dir, "energy_surrogate_model.joblib")
    joblib.dump(model, model_path)
    print(f"\n模型已保存至: {model_path}")
    
    # 保存模型元數據
    metadata = {
        "sample_count": len(df),
        "training_samples": len(X_train),
        "testing_samples": len(X_test),
        "mae_train": float(mae_train),
        "mae_test": float(mae_test),
        "rmse_train": float(rmse_train),
        "rmse_test": float(rmse_test),
        "r2_train": float(r2_train),
        "r2_test": float(r2_test),
        "training_time_seconds": training_time,
        "evaluation_time_seconds": eval_time,
        "feature_importance": {features[i]: float(feature_importance[i]) for i in range(len(features))}
    }
    
    import json
    metadata_path = os.path.join(model_dir, "model_metadata.json")
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
    
    print("\n訓練完成! 您的代理模型已準備就緒。")
    
    return model, metadata

if __name__ == "__main__":
    train_energy_surrogate()
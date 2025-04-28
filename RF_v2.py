import pandas as pd
import numpy as np
from sklearn.ensemble import RandomForestRegressor
from sklearn.tree import DecisionTreeRegressor
from sklearn.model_selection import train_test_split
from sklearn.metrics import mean_absolute_error, r2_score, mean_squared_error
import joblib
import os
import time

def train_minimal_model(data_file="energy_dataset.csv", model_dir="models", model_name="RFmodel_v2.joblib"):
    """
    訓練極度輕量化的隨機森林能源消耗代理模型(V2版本)
    
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
    
    # 特徵分析 - 檢查哪些特徵最重要
    print("\n=== 特徵相關性分析 ===")
    corr = df.corr()["EnergyConsumption"].abs().sort_values(ascending=False)
    print(corr)
    
    # 根據相關性選擇重要特徵
    # 如果只有一個特徵相關性很高，我們可以考慮僅使用該特徵
    top_feature = corr.index[1]  # 排除能耗本身
    print(f"\n最相關特徵: {top_feature}")
    
    # 準備特徵和標籤 - 提供兩種選項
    # 選項1: 使用所有特徵
    X_all = df[['ConstantSpeed_mps', 'SimulationDuration_s', 'RoadGradeFactor']].values
    # 選項2: 僅使用最相關特徵
    X_minimal = df[[top_feature]].values
    
    y = df['EnergyConsumption'].values
    
    # 分割訓練集和測試集
    X_all_train, X_all_test, X_minimal_train, X_minimal_test, y_train, y_test = train_test_split(
        X_all, X_minimal, y, test_size=0.2, random_state=42)
    
    print(f"\n訓練集大小: {len(y_train)} 筆")
    print(f"測試集大小: {len(y_test)} 筆")
    
    # 訓練兩種模型並比較結果
    models = {}
    
    # 1. 極簡隨機森林（只有5棵樹，淺深度）
    print("\n訓練極簡隨機森林模型...")
    rf_mini = RandomForestRegressor(
        n_estimators=5,        # 極少數的樹
        max_depth=5,           # 很淺的深度
        min_samples_split=10,  
        min_samples_leaf=5,    
        random_state=42,
        n_jobs=-1
    )
    
    start_time = time.time()
    rf_mini.fit(X_all_train, y_train)
    rf_mini_time = time.time() - start_time
    
    # 2. 單一決策樹
    print("\n訓練單一決策樹模型...")
    dt = DecisionTreeRegressor(
        max_depth=8,
        min_samples_split=10,
        min_samples_leaf=5,
        random_state=42
    )
    
    start_time = time.time()
    dt.fit(X_all_train, y_train)
    dt_time = time.time() - start_time
    
    # 3. 使用單一最重要特徵的隨機森林
    print(f"\n訓練僅使用 {top_feature} 的隨機森林...")
    rf_single_feature = RandomForestRegressor(
        n_estimators=10,
        max_depth=5,
        min_samples_split=5,
        min_samples_leaf=2,
        random_state=42,
        n_jobs=-1
    )
    
    start_time = time.time()
    rf_single_feature.fit(X_minimal_train, y_train)
    rf_single_time = time.time() - start_time
    
    # 收集模型結果
    models = {
        "rf_mini": {"model": rf_mini, "train_time": rf_mini_time, "features": "all"},
        "dt": {"model": dt, "train_time": dt_time, "features": "all"},
        "rf_single": {"model": rf_single_feature, "train_time": rf_single_time, "features": "single"}
    }
    
    # 評估所有模型
    print("\n=== 模型評估 ===")
    for name, model_info in models.items():
        model = model_info["model"]
        features = model_info["features"]
        
        # 選擇適當的測試集
        X_test = X_all_test if features == "all" else X_minimal_test
        
        # 測量推理時間
        n_repeats = 1000
        start_time = time.time()
        for _ in range(n_repeats):
            model.predict(X_test[:1])
        inference_time = (time.time() - start_time) / n_repeats
        
        # 評估準確度
        y_pred = model.predict(X_test)
        mae = mean_absolute_error(y_test, y_pred)
        rmse = np.sqrt(mean_squared_error(y_test, y_pred))
        r2 = r2_score(y_test, y_pred)
        
        # 更新模型信息
        model_info["mae"] = mae
        model_info["rmse"] = rmse
        model_info["r2"] = r2
        model_info["inference_time"] = inference_time
        
        print(f"\n{name} 模型評估:")
        print(f"  MAE: {mae:.6f} kWh")
        print(f"  RMSE: {rmse:.6f} kWh")
        print(f"  R²: {r2:.6f}")
        print(f"  單次推理時間: {inference_time*1000:.4f} 毫秒")
    
    # 選擇最佳的V2模型 - 基於速度和準確度的平衡
    # 基本策略: 如果單特徵模型的R²不是太低，選擇它作為最快的選項
    # 否則選擇極簡隨機森林
    
    rf_single_r2 = models["rf_single"]["r2"]
    threshold = 0.90  # 可接受的最低R²值
    
    if rf_single_r2 >= threshold:
        print(f"\n選擇單特徵隨機森林作為V2模型 (R²={rf_single_r2:.4f})")
        best_model = "rf_single"
    else:
        print(f"\n單特徵模型R²過低 ({rf_single_r2:.4f})，選擇極簡隨機森林作為V2模型")
        best_model = "rf_mini"
    
    # 保存選定的模型
    selected_model = models[best_model]["model"]
    selected_features = models[best_model]["features"]
    
    model_path = os.path.join(model_dir, model_name)
    joblib.dump(selected_model, model_path)
    print(f"模型已保存至: {model_path}")
    
    # 生成使用說明
    feature_instruction = ""
    if selected_features == "single":
        feature_instruction = f"""
注意: 此V2模型僅使用了單一特徵 '{top_feature}'。
使用方法示例:
```python
import joblib
model = joblib.load('{model_path}')
# 只用{top_feature}特徵預測
value = [your_{top_feature}_value]  # 單一特徵，如 [120.0]
prediction = model.predict([value])
```
"""
    else:
        feature_instruction = f"""
使用方法示例:
```python
import joblib
model = joblib.load('{model_path}')
# 使用全部三個特徵
features = [speed, duration, grade]  # 例如 [2.0, 120.0, 0.0]
prediction = model.predict([features])
```
"""
    
    # 保存模型元數據
    metadata = {
        "model_type": best_model,
        "features_used": selected_features,
        "important_feature": top_feature if selected_features == "single" else "all",
        "sample_count": len(df),
        "mae_test": float(models[best_model]["mae"]),
        "rmse_test": float(models[best_model]["rmse"]),
        "r2_test": float(models[best_model]["r2"]),
        "training_time_seconds": float(models[best_model]["train_time"]),
        "inference_time_ms": float(models[best_model]["inference_time"]*1000),
        "model_params": {
            "type": "RandomForestRegressor" if best_model != "dt" else "DecisionTreeRegressor",
            "n_estimators": 5 if best_model == "rf_mini" else (10 if best_model == "rf_single" else "N/A"),
            "max_depth": 5 if best_model in ["rf_mini", "rf_single"] else 8,
            "min_samples_split": 10 if best_model == "rf_mini" else 5,
            "min_samples_leaf": 5 if best_model == "rf_mini" else 2,
        }
    }
    
    import json
    metadata_path = os.path.join(model_dir, "RFmodel_v2_metadata.json")
    with open(metadata_path, "w") as f:
        json.dump(metadata, f, indent=4)
    print(f"模型元資料已保存至: {metadata_path}")
    
    # 創建使用指南
    guide_path = os.path.join(model_dir, "RFmodel_v2_usage_guide.md")
    with open(guide_path, "w") as f:
        f.write(f"""# RFmodel_v2 使用指南

    ## 模型資訊
    - 類型: {metadata['model_type']}
    - 特徵: {metadata['features_used']}
    - R² 分數: {metadata['r2_test']:.6f}
    - 平均推理時間: {metadata['inference_time_ms']:.4f} 毫秒

    ## 特性
    這是高速版能源消耗預測模型，為了達到最快的推理速度，犧牲了少量精確度。
    {feature_instruction}

    ## 使用方法
    {feature_instruction}

    ## 比較
    相較於V1模型:
    - 推理速度提升約 {models["rf_mini"]["inference_time"]*1000:.2f}/{models["dt"]["inference_time"]*1000:.2f}/{models["rf_single"]["inference_time"]*1000:.2f} 毫秒
    - 準確度 (R²): V1 vs V2 = ? vs {metadata['r2_test']:.4f}
    """)
    print(f"使用指南已保存至: {guide_path}")
    
    # 輸出測試樣本
    print("\n=== 預測樣本展示 ===")
    y_pred = selected_model.predict(X_test if selected_features == "all" else X_minimal_test)
    sample_indices = np.random.choice(len(X_test), min(5, len(X_test)), replace=False)
    
    print("\n測試樣本比較 (V2模型):")
    total_error_pct = 0
    for i, idx in enumerate(sample_indices):
        if selected_features == "all":
            speed, duration, grade = X_test[idx]
            feature_str = f"速度={speed:.2f} m/s, 時間={duration:.1f} s, 坡度={grade:.4f}"
        else:
            feature_value = X_minimal_test[idx][0]
            feature_str = f"{top_feature}={feature_value:.2f}"
            
        actual = y_test[idx]
        predicted = y_pred[idx]
        error = predicted - actual
        error_pct = (error / actual) * 100 if actual != 0 else float('inf')
        total_error_pct += abs(error_pct)
        
        print(f"樣本 {i+1}: {feature_str}")
        print(f"  實際值: {actual:.6f} kWh")
        print(f"  預測值: {predicted:.6f} kWh")
        print(f"  誤差: {error:.6f} kWh ({error_pct:.2f}%)\n")
    
    avg_error_pct = total_error_pct / len(sample_indices)
    print(f"平均誤差百分比: {avg_error_pct:.2f}%")
    
    print("\nV2模型訓練完成! 這是專為最高速度優化的極輕量版本。")
    
    return selected_model, metadata

if __name__ == "__main__":
    train_minimal_model()
import pandas as pd
import numpy as np
import time
import joblib
import os
from sklearn.model_selection import train_test_split
from sklearn.metrics import mean_absolute_error, mean_squared_error, r2_score
from sklearn.linear_model import LinearRegression
import xgboost as xgb
import tensorflow as tf
from tensorflow.keras.models import Sequential, save_model, load_model
from tensorflow.keras.layers import Dense, Conv1D, Flatten, Reshape
from tensorflow.keras.optimizers import Adam

def create_model_directory():
    """創建模型保存目錄"""
    model_dir = "models/comparison"
    os.makedirs(model_dir, exist_ok=True)
    return model_dir

def load_data(data_file="energy_dataset.csv"):
    """載入數據並分割訓練測試集"""
    print(f"載入數據: {data_file}")
    df = pd.read_csv(data_file)
    
    # 準備特徵和標籤
    X = df[['ConstantSpeed_mps', 'SimulationDuration_s', 'RoadGradeFactor']].values
    y = df['EnergyConsumption'].values
    
    # 分割訓練集和測試集
    X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)
    
    print(f"數據集大小: {len(df)} 筆")
    print(f"訓練集: {len(X_train)} 筆, 測試集: {len(X_test)} 筆")
    
    return X_train, X_test, y_train, y_test, df.columns[:3].tolist()

def train_linear_regression(X_train, y_train):
    """訓練線性迴歸模型"""
    print("\n===== 訓練線性迴歸模型 =====")
    start_time = time.time()
    
    model = LinearRegression()
    model.fit(X_train, y_train)
    
    training_time = time.time() - start_time
    print(f"訓練完成，耗時: {training_time:.4f} 秒")
    
    return model, training_time

def train_xgboost(X_train, y_train):
    """訓練XGBoost模型"""
    print("\n===== 訓練XGBoost模型 =====")
    start_time = time.time()
    
    model = xgb.XGBRegressor(
        n_estimators=100,
        max_depth=6,
        learning_rate=0.1,
        subsample=0.8,
        colsample_bytree=0.8,
        random_state=42,
        n_jobs=-1
    )
    model.fit(X_train, y_train)
    
    training_time = time.time() - start_time
    print(f"訓練完成，耗時: {training_time:.4f} 秒")
    
    return model, training_time

def train_cnn(X_train, y_train, epochs=50):
    """訓練簡單的CNN模型"""
    print("\n===== 訓練CNN模型 =====")
    start_time = time.time()
    
    # 重塑輸入為CNN需要的形狀: [samples, time_steps, features]
    X_train_reshaped = X_train.reshape(X_train.shape[0], X_train.shape[1], 1)
    
    model = Sequential([
        Conv1D(16, kernel_size=2, activation='relu', input_shape=(X_train.shape[1], 1)),
        Conv1D(32, kernel_size=2, activation='relu'),
        Flatten(),
        Dense(20, activation='relu'),
        Dense(1)
    ])
    
    model.compile(optimizer=Adam(learning_rate=0.001), loss='mse')
    
    # 靜默訓練輸出
    model.fit(
        X_train_reshaped, 
        y_train, 
        epochs=epochs, 
        batch_size=32, 
        verbose=0
    )
    
    training_time = time.time() - start_time
    print(f"訓練完成，耗時: {training_time:.4f} 秒")
    
    return model, training_time

def evaluate_model(model, X_test, y_test, model_name, is_cnn=False):
    """評估模型性能"""
    start_time = time.time()
    
    if is_cnn:
        X_test_reshaped = X_test.reshape(X_test.shape[0], X_test.shape[1], 1)
        y_pred = model.predict(X_test_reshaped, verbose=0).flatten()
    else:
        y_pred = model.predict(X_test)
    
    inference_time = time.time() - start_time
    
    # 計算多次預測的平均時間
    n_repeats = 100
    start_time = time.time()
    
    for _ in range(n_repeats):
        if is_cnn:
            # 只預測一個樣本
            sample = X_test_reshaped[0:1]
            _ = model.predict(sample, verbose=0)
        else:
            _ = model.predict(X_test[0:1])
    
    single_inference_time = (time.time() - start_time) / n_repeats
    
    # 計算評估指標
    mae = mean_absolute_error(y_test, y_pred)
    rmse = np.sqrt(mean_squared_error(y_test, y_pred))
    r2 = r2_score(y_test, y_pred)
    
    print(f"\n----- {model_name} 模型評估 -----")
    print(f"平均絕對誤差 (MAE): {mae:.6f}")
    print(f"均方根誤差 (RMSE): {rmse:.6f}")
    print(f"決定係數 (R²): {r2:.6f}")
    print(f"批量推理時間 ({len(X_test)} 筆): {inference_time:.6f} 秒")
    print(f"單次推理時間: {single_inference_time*1000:.4f} 毫秒")
    
    return {
        "model_name": model_name,
        "mae": mae,
        "rmse": rmse,
        "r2": r2,
        "batch_inference_time": inference_time,
        "single_inference_time": single_inference_time
    }

def save_models(models, model_dir):
    """保存所有模型"""
    print("\n===== 保存模型 =====")
    
    # 保存線性迴歸模型
    lr_path = os.path.join(model_dir, "linear_regression_model.joblib")
    joblib.dump(models["linear_regression"]["model"], lr_path)
    print(f"線性迴歸模型已保存至: {lr_path}")
    
    # 保存XGBoost模型
    xgb_path = os.path.join(model_dir, "xgboost_model.joblib")
    joblib.dump(models["xgboost"]["model"], xgb_path)
    print(f"XGBoost模型已保存至: {xgb_path}")
    
    # 保存CNN模型
    cnn_path = os.path.join(model_dir, "cnn_model")
    models["cnn"]["model"].save(cnn_path)
    print(f"CNN模型已保存至: {cnn_path}")
    
    # 保存評估結果
    results = {
        "linear_regression": models["linear_regression"]["evaluation"],
        "xgboost": models["xgboost"]["evaluation"],
        "cnn": models["cnn"]["evaluation"]
    }
    
    # 添加訓練時間
    for model_name in results:
        results[model_name]["training_time"] = models[model_name]["training_time"]
    
    import json
    with open(os.path.join(model_dir, "model_comparison_results.json"), "w") as f:
        json.dump(results, f, indent=4)

def compare_inference_speed(models, X_test, n_samples=1000):
    """比較不同模型的推理速度"""
    print("\n===== 推理速度比較 (模擬實際使用) =====")
    
    # 確保有足夠的樣本進行測試
    if len(X_test) < n_samples:
        # 通過複製現有樣本來擴充測試集
        multiplier = int(np.ceil(n_samples / len(X_test)))
        X_test_expanded = np.tile(X_test, (multiplier, 1))
        X_test_speed = X_test_expanded[:n_samples]
    else:
        X_test_speed = X_test[:n_samples]
    
    # 準備CNN輸入
    X_test_cnn = X_test_speed.reshape(X_test_speed.shape[0], X_test_speed.shape[1], 1)
    
    # 測試線性迴歸
    lr_model = models["linear_regression"]["model"]
    start_time = time.time()
    _ = lr_model.predict(X_test_speed)
    lr_time = time.time() - start_time
    
    # 測試XGBoost
    xgb_model = models["xgboost"]["model"]
    start_time = time.time()
    _ = xgb_model.predict(X_test_speed)
    xgb_time = time.time() - start_time
    
    # 測試CNN
    cnn_model = models["cnn"]["model"]
    start_time = time.time()
    _ = cnn_model.predict(X_test_cnn, verbose=0)
    cnn_time = time.time() - start_time
    
    print(f"測試樣本數: {n_samples}")
    print(f"線性迴歸推理時間: {lr_time:.4f} 秒, 每樣本 {lr_time/n_samples*1000:.4f} 毫秒")
    print(f"XGBoost推理時間: {xgb_time:.4f} 秒, 每樣本 {xgb_time/n_samples*1000:.4f} 毫秒")
    print(f"CNN推理時間: {cnn_time:.4f} 秒, 每樣本 {cnn_time/n_samples*1000:.4f} 毫秒")
    
    # 計算相對速度
    fastest_time = min(lr_time, xgb_time, cnn_time)
    print("\n相對速度比較 (越低越好):")
    print(f"線性迴歸: {lr_time/fastest_time:.2f}x")
    print(f"XGBoost: {xgb_time/fastest_time:.2f}x")
    print(f"CNN: {cnn_time/fastest_time:.2f}x")
    
    return {
        "linear_regression": lr_time,
        "xgboost": xgb_time,
        "cnn": cnn_time,
        "samples": n_samples
    }

def main(data_file="energy_dataset.csv"):
    """主函數，執行完整的比較流程"""
    print("===== 開始模型比較 =====")
    
    # 創建模型目錄
    model_dir = create_model_directory()
    
    # 載入數據
    X_train, X_test, y_train, y_test, feature_names = load_data(data_file)
    
    # 訓練模型
    lr_model, lr_time = train_linear_regression(X_train, y_train)
    xgb_model, xgb_time = train_xgboost(X_train, y_train)
    cnn_model, cnn_time = train_cnn(X_train, y_train)
    
    # 評估模型
    lr_eval = evaluate_model(lr_model, X_test, y_test, "線性迴歸")
    xgb_eval = evaluate_model(xgb_model, X_test, y_test, "XGBoost")
    cnn_eval = evaluate_model(cnn_model, X_test, y_test, "CNN", is_cnn=True)
    
    # 收集模型信息
    models = {
        "linear_regression": {
            "model": lr_model,
            "evaluation": lr_eval,
            "training_time": lr_time
        },
        "xgboost": {
            "model": xgb_model,
            "evaluation": xgb_eval,
            "training_time": xgb_time
        },
        "cnn": {
            "model": cnn_model,
            "evaluation": cnn_eval,
            "training_time": cnn_time
        }
    }
    
    # 比較推理速度
    speed_comparison = compare_inference_speed(models, X_test)
    
    # 保存模型
    save_models(models, model_dir)
    
    # 打印總結
    print("\n===== 模型比較總結 =====")
    print("\n訓練時間比較:")
    print(f"線性迴歸: {lr_time:.4f} 秒")
    print(f"XGBoost: {xgb_time:.4f} 秒")
    print(f"CNN: {cnn_time:.4f} 秒")
    
    print("\n精度比較 (R² 分數, 越高越好):")
    print(f"線性迴歸: {lr_eval['r2']:.6f}")
    print(f"XGBoost: {xgb_eval['r2']:.6f}")
    print(f"CNN: {cnn_eval['r2']:.6f}")
    
    print("\n推理速度比較 (單次, 毫秒):")
    print(f"線性迴歸: {lr_eval['single_inference_time']*1000:.4f} 毫秒")
    print(f"XGBoost: {xgb_eval['single_inference_time']*1000:.4f} 毫秒")
    print(f"CNN: {cnn_eval['single_inference_time']*1000:.4f} 毫秒")
    
    print(f"\n所有結果已保存至: {model_dir}")
    print("===== 比較完成 =====")

if __name__ == "__main__":
    main()
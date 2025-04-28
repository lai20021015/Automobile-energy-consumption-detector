import os
import joblib
import numpy as np

def test_energy_model(speed, duration, grade, model_path="models/RFmodel_v1.joblib"):
    """
    使用訓練好的模型預測能源消耗
    
    Args:
        speed: 速度 (m/s)
        duration: 時間 (s)
        grade: 坡度因子
        model_path: 模型路徑
    
    Returns:
        預測的能源消耗 (kWh)
    """
    # 檢查模型是否存在
    if not os.path.exists(model_path):
        return f"錯誤: 找不到模型 {model_path}"
    
    try:
        # 加載模型
        model = joblib.load(model_path)
        
        # 準備輸入特徵
        features = np.array([[float(speed), float(duration), float(grade)]])
        
        # 預測能源消耗
        energy = model.predict(features)[0]
        
        return energy
    
    except Exception as e:
        return f"預測過程中出現錯誤: {e}"

def get_valid_float_input(prompt):
    """確保獲取有效的浮點數輸入"""
    while True:
        try:
            value = input(prompt)
            # 檢查是否使用了逗號而非小數點
            if ',' in value and '.' not in value:
                value = value.replace(',', '.')
            return float(value)
        except ValueError:
            print(f"無效輸入! 請輸入有效的數字 (例如: 1.5)")

if __name__ == "__main__":
    print("能源消耗代理模型測試")
    print("=====================")
    
    try:
        # 檢查模型是否存在
        model_path = "models/RFmodel_v2.joblib"
        if not os.path.exists(model_path):
            print(f"警告: 找不到默認模型路徑 {model_path}")
            model_path = input("請輸入正確的模型路徑: ")
            if not os.path.exists(model_path):
                print(f"錯誤: 無法找到模型 {model_path}")
                exit(1)
        
        # 讀取用戶輸入
        print("\n請輸入以下參數:")
        speed = get_valid_float_input("速度 (m/s): ")
        duration = get_valid_float_input("時間 (s): ")
        grade = get_valid_float_input("坡度因子: ")
        
        # 獲取預測結果
        energy = test_energy_model(speed, duration, grade, model_path)
        
        # 檢查結果是否為錯誤消息
        if isinstance(energy, str):
            print(f"\n錯誤: {energy}")
        else:
            # 顯示結果
            print("\n===== 預測結果 =====")
            print(f"速度: {speed} m/s")
            print(f"時間: {duration} s")
            print(f"坡度: {grade}")
            print(f"預測能耗: {energy:.4f} kWh")
        
    except KeyboardInterrupt:
        print("\n程序已被用戶中斷。")
    except Exception as e:
        print(f"\n程序執行出現意外錯誤: {e}")
    
    # 等待用戶按下任意鍵退出
    input("\n按下Enter鍵退出...")
# NYCU IEM Project
# 通過自動化模擬優化列車能耗和準時性
![alt text](image.png)
## 一、研究動機
- 觀察到由於缺乏自動駕駛輔助系統，司機駕駛行為差異對列車準時性和運行效率產生顯著影響
- 旨在開發一套自動化模擬系統，優化駕駛行為，提高車輛準時性和效率
- 解決傳統操作中不當加速或剎車造成的能源浪費問題
- 設計一種加減速控制策略，在確保準時性的同時優化列車能耗

## 二、研究架構
1. 數據收集和基本設計（FASTSim，OpenModelica）
2. 系統建模和模擬測試
3. 參數調整和功能擴展
4. 模擬結果可視化
5. 設計模擬器（pygame）

## 三、研究方法

1. 數據收集和基本設計（FASTSim，OpenModelica）
   - 模型選擇：FASTSim 和 OpenModelica
   - 台灣鐵路行車模式（速度）

2. 系統建模和模擬測試
   - 設置行駛週期
   - 使用二次函數最小平方法獲得該環境下的最小能耗
   - 距離、時間、最大速度限制、最大加速度限制

3. 參數調整和功能擴展
   - 增加可調參數：空氣阻力、重力（重量、坡度）、慣性
   - 全面考慮空氣阻力、滾動阻力和加速能耗

4. 模擬結果可視化（Streamlit）

5. 設計模擬器

## 四、研究結果（成果展示）

- Streamlit 界面
- `create_speed_profile` 函數
   - 靈活性
   - 平滑插值
   - 距離校正
- `simulate_energy` 函數
   - 目標：計算給定速度剖面的總能耗，並加入懲罰項
   - 優點：
     1. 多層次懲罰
     2. 全面模擬
     3. 穩健性
- `optimize` 函數
   - 執行優化：重複運行 `simulate` 直到找到最佳解
   - 目標函數是 `self.simulate_energy`，即模擬能耗
   - 使用 SLSQP 算法
- `plot_result` 函數
   - 根據優化結果生成時間和速度剖面
- `TrainEnergyOptimizer` 類
   - 驗證方法：
     1. `calculate_theoretical_minimum`：計算理論最小能耗
     2. `sensitivity_analysis`：測試不同參數組合的敏感度
     3. `validate_solution`：全面驗證優化解的合理性
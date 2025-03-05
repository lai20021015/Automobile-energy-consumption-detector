import streamlit as st
import fastsim as fsim
import numpy as np
import matplotlib.pyplot as plt
from train_energy_optimizer import TrainEnergyOptimizer

def main():
    # 設定中文字體
    plt.rcParams['font.sans-serif'] = ['Microsoft JhengHei'] 
    plt.rcParams['axes.unicode_minus'] = False
    st.title('列車能耗優化系統')

    # 側邊欄參數設置
    st.sidebar.header('參數設置')
    distance_m = st.sidebar.number_input('行駛距離(公尺)', min_value=100.0, max_value=5000.0, value=1000.0)
    time_s = st.sidebar.number_input('行駛時間(秒)', min_value=10.0, max_value=300.0, value=60.0)
    max_speed = st.sidebar.number_input('最高速度(公尺/秒)', min_value=5.0, max_value=50.0, value=30.0)
    max_accel = st.sidebar.number_input('最大加速度(公尺/平方秒)', min_value=0.5, max_value=2.0, value=1.1)
    control_points = st.sidebar.slider('控制點數量', min_value=3, max_value=9, value=5)
    
    # 車輛選擇
    veh_options = {
        "EMU-500車型": 43,
        "EMU-3000": 44,
        "EMU-900": 45
    }
    selected_veh = st.sidebar.selectbox("選擇車輛類型", list(veh_options.keys()))
    veh_id = veh_options[selected_veh]

    # 創建優化器實例
    optimizer = TrainEnergyOptimizer(
        distance_m=distance_m,
        time_s=time_s, 
        max_speed_mps=max_speed,
        # max_speed_mps=np.full(61, max_speed),
        max_accel=max_accel,
        control_points=control_points,
        veh_id=veh_id
    )

    # 優化按鈕
    if st.button('開始優化'):
        with st.spinner('正在執行優化計算...'):
            # 執行優化
            results = optimizer.optimize()
            
            # 顯示優化結果
            st.success('優化完成!')
            
            # 分成兩列顯示
            col1, col2 = st.columns(2)
            
            with col1:
                st.header('速度剖面')
                fig, ax = plt.subplots()
                ax.plot(results['optimal_time'], results['optimal_speed'] * 3.6)
                ax.set_xlabel('時間 (秒)')
                ax.set_ylabel('速度 (公里/小時)')
                ax.grid(True)
                st.pyplot(fig)
            
            with col2:
                st.header('能耗分析')
                # 計算累積能耗
                energy_consumption = results['simulation'].ess_cur_kwh[0] - results['simulation'].ess_cur_kwh[:-1]
                fig, ax = plt.subplots()
                ax.plot(results['optimal_time'][:-1], energy_consumption)
                ax.set_xlabel('時間 (秒)')
                ax.set_ylabel('累積能耗 (kWh)')
                ax.grid(True)
                st.pyplot(fig)
            
            # 顯示詳細能耗分析
            st.header('詳細能耗分析')
            sim = results['simulation']
            time_s = results['optimal_time']
            
            # 計算各項能耗
            drag_energy = np.trapz(sim.drag_kw[1:], time_s[:-1])/3600
            rr_energy = np.trapz(sim.rr_kw[1:], time_s[:-1])/3600
            accel_energy = np.trapz(sim.accel_kw[1:], time_s[:-1])/3600
            total_energy = abs(results['optimal_energy'])
            
            # 使用表格顯示結果
            energy_data = {
                '項目': ['總能耗', '每公里能耗', '空氣阻力損耗', '滾動阻力損耗', '加速能耗'],
                '數值': [
                    f"{total_energy:.3f} kWh",
                    f"{(total_energy/(distance_m/1000)):.3f} kWh/km",
                    f"{drag_energy:.3f} kWh ({drag_energy/total_energy*100:.1f}%)",
                    f"{rr_energy:.3f} kWh ({rr_energy/total_energy*100:.1f}%)",
                    f"{accel_energy:.3f} kWh ({accel_energy/total_energy*100:.1f}%)"
                ]
            }
            st.table(energy_data)
            
            # 添加驗證按鈕
            if st.button('執行解決方案驗證'):
                with st.spinner('正在執行驗證分析...'):
                    theoretical_min, sensitivity_df = optimizer.validate_solution()
                    st.write(f"理論最小能耗: {theoretical_min:.3f} kWh")
                    st.write("敏感度分析結果:")
                    st.dataframe(sensitivity_df)

if __name__ == '__main__':
    main()
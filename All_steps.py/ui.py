import streamlit as st
import fastsim as fsim
import numpy as np
import matplotlib.pyplot as plt

# 保留原有的函數定義
def create_custom_cycle(profile_type, params):
    """
    創建自定義行駛循環
    profile_type: 'constant', 'accelerate', 'pulse'
    params: 相關參數
    """
    dt_s = 1.0
    time_s = np.arange(0, params['duration_s'], dt_s)
    
    if profile_type == 'constant':
        # 恆定速度
        mps = np.ones_like(time_s) * params['speed_mps']
        
    elif profile_type == 'accelerate':
        # 加速-恆速-減速
        accel_time = params['accel_time']
        decel_time = params['decel_time']
        cruise_time = params['duration_s'] - accel_time - decel_time
        
        accel_profile = np.linspace(0, params['speed_mps'], int(accel_time))
        cruise_profile = np.ones(int(cruise_time)) * params['speed_mps']
        decel_profile = np.linspace(params['speed_mps'], 0, int(decel_time))
        
        mps = np.concatenate([accel_profile, cruise_profile, decel_profile])
        
    elif profile_type == 'pulse':
        # 脈衝式速度變化
        period = params['period']
        mps = np.zeros_like(time_s)
        for i in range(len(time_s)):
            if (i % period) < (period/2):
                mps[i] = params['speed_mps']
            else:
                mps[i] = params['speed_mps'] * 0.5
    
    cycle_dict = {
        'time_s': time_s,
        'mps': mps,
        'grade': np.zeros_like(time_s),
        'road_type': np.zeros_like(time_s)
    }
    
    return fsim.cycle.Cycle.from_dict(cycle_dict)

def analyze_driving_pattern(sim, title="Driving Analysis"):
    """分析駕駛模式的各項指標"""
    # 計算關鍵指標
    total_energy = np.trapz(sim.fc_kw_out_ach, sim.cyc.time_s)
    avg_power = np.mean(sim.fc_kw_out_ach)
    peak_power = np.max(sim.fc_kw_out_ach)
    
    # 計算能源效率
    distance = np.sum(sim.dist_mi) * 1.60934  # 轉換為公里
    energy_efficiency = total_energy / distance  # 每公里能耗
    
    # 計算加速度
    accel = np.diff(sim.mps_ach) / np.diff(sim.cyc.time_s)
    max_accel = np.max(accel)
    
    return {
        "總能耗(kJ)": total_energy,
        "平均功率(kW)": avg_power,
        "最大功率(kW)": peak_power,
        "能源效率(kJ/km)": energy_efficiency,
        "最大加速度(m/s²)": max_accel,
    }

def plot_analysis(sim, title="分析結果"):
    """繪製詳細分析圖表"""
    fig, axes = plt.subplots(4, 1, figsize=(12, 15))
    fig.suptitle(title, fontsize=16)
    
    # 速度曲線
    axes[0].plot(sim.cyc.time_s, sim.mps_ach * 3.6, 'b-', label='車速')
    axes[0].set_ylabel('速度 (km/h)')
    axes[0].grid(True)
    axes[0].legend()
    
    # 功率分配
    axes[1].plot(sim.cyc.time_s, sim.drag_kw, 'r-', label='空氣阻力損耗')
    axes[1].plot(sim.cyc.time_s, sim.fc_kw_out_ach, 'g-', label='引擎輸出功率')
    axes[1].set_ylabel('功率 (kW)')
    axes[1].grid(True)
    axes[1].legend()
    
    # 能耗累積
    energy_cumulative = np.cumsum(sim.fc_kw_out_ach * np.diff(sim.cyc.time_s, prepend=0))
    axes[2].plot(sim.cyc.time_s, energy_cumulative, 'b-', label='累積能耗')
    axes[2].set_ylabel('累積能耗 (kJ)')
    axes[2].grid(True)
    axes[2].legend()
    
    # 加速度
    accel = np.diff(sim.mps_ach) / np.diff(sim.cyc.time_s)
    axes[3].plot(sim.cyc.time_s[1:], accel, 'g-', label='加速度')
    axes[3].set_ylabel('加速度 (m/s²)')
    axes[3].set_xlabel('時間 (s)')
    axes[3].grid(True)
    axes[3].legend()
    
    plt.tight_layout()
    return fig

# Streamlit 介面
def main():
    st.title("車輛駕駛模式分析儀表板")
    st.write("正在載入設定...")
    # 設定中文字體
    plt.rcParams['font.sans-serif'] = ['Microsoft JhengHei']
    plt.rcParams['axes.unicode_minus'] = False
    
    # 側邊欄：選擇車輛和駕駛模式
    st.sidebar.header("設定參數")
    
    # 車輛選擇（這裡先使用固定車輛，您可以添加更多選項）
   # 修改車輛載入部分
    try:
        veh = fsim.vehicle.Vehicle.from_file('2012_Ford_Fusion.csv')
    except Exception as e:
        st.error(f"載入車輛數據時出錯: {str(e)}")
        st.info("請確保 2012_Ford_Fusion.csv 文件在正確的路徑下")
        veh = fsim.vehicle.Vehicle()  # 創建默認車輛作為備選
    
    # 駕駛模式選擇
    driving_mode = st.sidebar.selectbox(
        "選擇駕駛模式",
        ["恆速巡航", "漸進加速", "脈衝行駛"]
    )
    
    # 根據不同模式設置參數
    st.sidebar.subheader("模式參數設定")
    
    base_params = {
        "speed_mps": st.sidebar.slider("目標速度 (m/s)", 5, 35, 25),
        "duration_s": st.sidebar.slider("模擬時間 (s)", 50, 200, 100)
    }
    
    if driving_mode == "漸進加速":
        base_params.update({
            "accel_time": st.sidebar.slider("加速時間 (s)", 10, 50, 30),
            "decel_time": st.sidebar.slider("減速時間 (s)", 10, 50, 30)
        })
    elif driving_mode == "脈衝行駛":
        base_params.update({
            "period": st.sidebar.slider("脈衝週期 (s)", 10, 40, 20)
        })
    
    # 運行模擬按鈕
    if st.sidebar.button("運行模擬"):
        # 創建模擬循環
        pattern_type_map = {
            "恆速巡航": "constant",
            "漸進加速": "accelerate",
            "脈衝行駛": "pulse"
        }
        
        cyc = create_custom_cycle(pattern_type_map[driving_mode], base_params)
        sim = fsim.simdrive.SimDrive(cyc, veh)
        sim.sim_drive()
        
        # 顯示分析結果
        col1, col2 = st.columns([2, 1])
        
        with col1:
            st.subheader("模擬圖表")
            fig = plot_analysis(sim, f"{driving_mode}模式分析")
            st.pyplot(fig)
        
        with col2:
            st.subheader("分析指標")
            metrics = analyze_driving_pattern(sim, driving_mode)
            for metric, value in metrics.items():
                st.metric(metric, f"{value:.2f}")
        
        # 顯示車輛參數
        st.subheader("車輛參數")
        st.write(f"空氣阻力係數: {veh.drag_coef}")
        st.write(f"迎風面積: {veh.frontal_area_m2} m²")
        st.write(f"車輛質量: {veh.veh_kg} kg")

if __name__ == "__main__":
    main()
import fastsim as fsim
import numpy as np
import matplotlib.pyplot as plt

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

def compare_driving_patterns(veh):
    """比較不同駕駛模式"""
    # 定義不同的駕駛模式
    patterns = {
        '恆速巡航': {
            'type': 'constant',
            'params': {'speed_mps': 25, 'duration_s': 100}
        },
        '漸進加速': {
            'type': 'accelerate',
            'params': {
                'speed_mps': 25,
                'duration_s': 100,
                'accel_time': 30,
                'decel_time': 30
            }
        },
        '脈衝行駛': {
            'type': 'pulse',
            'params': {
                'speed_mps': 25,
                'duration_s': 100,
                'period': 20
            }
        }
    }
    
    results = {}
    for name, config in patterns.items():
        # 創建循環並執行模擬
        cyc = create_custom_cycle(config['type'], config['params'])
        sim = fsim.simdrive.SimDrive(cyc, veh)
        sim.sim_drive()
        
        # 分析結果
        results[name] = analyze_driving_pattern(sim, name)
        
        # 繪製圖表
        fig = plot_analysis(sim, f"{name}模式分析")
        plt.show()
    
    # 比較結果
    print("\n不同駕駛模式比較:")
    metrics = list(next(iter(results.values())).keys())
    for metric in metrics:
        print(f"\n{metric}:")
        for pattern_name, result in results.items():
            print(f"{pattern_name}: {result[metric]:.2f}")

def main():
    # 設定中文字體
    plt.rcParams['font.sans-serif'] = ['Microsoft JhengHei']
    plt.rcParams['axes.unicode_minus'] = False
    
    # 載入車輛
    veh = fsim.vehicle.Vehicle.from_file('2012_Ford_Fusion.csv')
    
    # 比較不同駕駛模式
    compare_driving_patterns(veh)
    
    # 顯示車輛參數
    print(f"\n車輛參數:")
    print(f"空氣阻力係數: {veh.drag_coef}")
    print(f"迎風面積: {veh.frontal_area_m2} m²")
    print(f"車輛質量: {veh.veh_kg} kg")

if __name__ == "__main__":
    main()
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

fig, ax = plt.subplots(figsize=(10, 6))

#中文
plt.rcParams['font.sans-serif'] = ['Microsoft JhengHei'] 
plt.rcParams['axes.unicode_minus'] = False


t = np.linspace(0, 66, 500)
v = np.where(t < 10, 7*t, 
             np.where(t < 50, 70, 
                     np.where(t < 60, 70 - 7*(t-50), 0)))

ax.set_xlim(0, 60)
ax.set_ylim(0, 80)
ax.set_xlabel('時間 (秒)')
ax.set_ylabel('速度 (公里/小時)')
ax.set_title('最佳化速度-時間圖')
ax.grid(True)

line, = ax.plot([], [], 'b-', label='最佳化速度')
ax.legend()

def init():
    line.set_data([], [])
    return line,

def animate(i):
    x = t[:i]
    y = v[:i]
    line.set_data(x, y)
    return line,

anim = FuncAnimation(fig, animate, init_func=init,
                    frames=len(t), interval=10,  # 改
                    blit=True, repeat=False)

plt.show()

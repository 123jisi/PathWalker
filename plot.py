import matplotlib.pyplot as plt
import matplotlib.animation as animation
import os
import time

# log路径
THEORETICAL_LOG = os.path.expanduser("~/ros2_ws/theoretical_log.txt")
ACTUAL_LOG = os.path.expanduser("~/ros2_ws/log.txt")
# timestamp = time.strftime("%Y%m%d-%H%M%S")
gif_path = f"trajectory_comparison_square_IBP.gif"


def parse_log(file_path, skip_header=True):
    xs, ys = [], []
    try:
        with open(file_path, "r") as f:
            lines = f.readlines()[1:] if skip_header else f.readlines()
            for line in lines:
                parts = line.strip().split(",")
                if len(parts) >= 2:
                    try:
                        xs.append(float(parts[0]))
                        ys.append(float(parts[1]))
                    except ValueError:
                        continue
    except FileNotFoundError:
        pass
    return xs, ys

# 初始化画布
fig, ax = plt.subplots()
line_theory, = ax.plot([], [], 'b--', label='Theoretical Path')  # 蓝色虚线
line_actual, = ax.plot([], [], 'r-', label='Actual Path')        # 红色实线

ax.set_title("Theoretical vs Actual Trajectory")
ax.set_xlabel("X Position (m)")
ax.set_ylabel("Y Position (m)")
ax.grid(True)
ax.set_aspect('equal')
ax.legend()

# 初始化动画
def init():
    line_theory.set_data([], [])
    line_actual.set_data([], [])
    return line_theory, line_actual

# 动画更新
def update(frame):
    xs_t, ys_t = parse_log(THEORETICAL_LOG)
    xs_a, ys_a = parse_log(ACTUAL_LOG)

    line_theory.set_data(xs_t, ys_t)
    line_actual.set_data(xs_a, ys_a)

    all_x = xs_t + xs_a
    all_y = ys_t + ys_a
    if all_x and all_y:
        ax.set_xlim(min(all_x) - 0.1, max(all_x) + 0.1)
        ax.set_ylim(min(all_y) - 0.1, max(all_y) + 0.1)
    return line_theory, line_actual

# 启动动画
ani = animation.FuncAnimation(fig, update, init_func=init,
                              interval=300, blit=True, save_count=200)

# 导出为GIF
ani.save(gif_path, writer='pillow')
print(f"Trajectory comparison saved to {gif_path}")

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
import math
import csv

class TrajectoryPlotter:
    def __init__(self, topic_name='/cmd_vel'):
        self.topic_name = topic_name
        
        # 数据存储
        self.times = []
        self.linear_x = []
        self.angular_z = []
        
        # 轨迹推算数据 (初始位置设为 0,0,0)
        self.traj_x = [0.0]
        self.traj_y = [0.0]
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        
        self.last_time = None
        self.start_time = None
        self.is_running = True

        # 初始化节点
        rospy.init_node('cmd_vel_trajectory_plotter', anonymous=True)
        
        # 订阅话题
        rospy.Subscriber(self.topic_name, Twist, self.callback)
        print(f"\n>>> 开始监听话题: {self.topic_name}")
        print(">>> 正在记录指令并计算理想轨迹...")
        print(">>> 按 Ctrl+C 结束并绘图")

    def callback(self, msg):
        if not self.is_running:
            return

        current_ros_time = rospy.Time.now()
        current_sec = current_ros_time.to_sec()
        
        # 初始化时间戳
        if self.start_time is None:
            self.start_time = current_sec
            self.last_time = current_sec
            return # 第一帧只记录时间，无法计算 dt

        # 计算时间差 dt
        dt = current_sec - self.last_time
        self.last_time = current_sec

        # 记录原始速度数据
        rel_time = current_sec - self.start_time
        self.times.append(rel_time)
        self.linear_x.append(msg.linear.x)
        self.angular_z.append(msg.angular.z)

        # === 核心逻辑：航迹推算 (Dead Reckoning) ===
        # 使用简单的欧拉积分计算位置
        # x += v * cos(theta) * dt
        # y += v * sin(theta) * dt
        # theta += w * dt
        
        v = msg.linear.x
        w = msg.angular.z

        # 更新位置
        self.current_x += v * math.cos(self.current_theta) * dt
        self.current_y += v * math.sin(self.current_theta) * dt
        
        # 更新朝向
        self.current_theta += w * dt

        # 保存轨迹点
        self.traj_x.append(self.current_x)
        self.traj_y.append(self.current_y)

    def save_and_plot(self):
        self.is_running = False
        if not self.times:
            print("\n[警告] 未接收到有效数据（数据量不足）！")
            return

        print(f"\n\n>>> 停止记录，共收集 {len(self.times)} 帧数据。")
        print(">>> 正在绘制曲线...")

        # 创建一个较大的画布，包含两个部分
        fig = plt.figure(figsize=(12, 8))

        # --- 子图 1: 速度曲线 ---
        ax1 = plt.subplot(2, 2, 1) # 左上
        ax1.plot(self.times, self.linear_x, 'b-', label='Linear X', linewidth=1.5)
        ax1.set_title('Linear Velocity (m/s)')
        ax1.grid(True)
        ax1.legend()

        ax2 = plt.subplot(2, 2, 3) # 左下
        ax2.plot(self.times, self.angular_z, 'r-', label='Angular Z', linewidth=1.5)
        ax2.set_title('Angular Velocity (rad/s)')
        ax2.set_xlabel('Time (s)')
        ax2.grid(True)
        ax2.legend()

        # --- 子图 2: 理想轨迹图 (XY平面) ---
        ax3 = plt.subplot(1, 2, 2) # 右侧占满
        ax3.plot(self.traj_x, self.traj_y, 'g-', linewidth=2, label='Ideal Trajectory')
        
        # 标记起点和终点
        ax3.plot(self.traj_x[0], self.traj_y[0], 'ko', label='Start')
        ax3.plot(self.traj_x[-1], self.traj_y[-1], 'ro', label='End')
        
        ax3.set_title(f'Reconstructed Path from {self.topic_name}')
        ax3.set_xlabel('X Position (m)')
        ax3.set_ylabel('Y Position (m)')
        ax3.axis('equal') # 保证比例尺一致，这点对看轨迹很重要
        ax3.grid(True)
        ax3.legend()
        
        # 添加一些文字辅助
        info_text = f"Total Time: {self.times[-1]:.1f}s\nFinal X: {self.traj_x[-1]:.2f}m\nFinal Y: {self.traj_y[-1]:.2f}m"
        ax3.text(0.05, 0.95, info_text, transform=ax3.transAxes, verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

        plt.tight_layout()

        # 保存图片
        img_filename = 'trajectory_analysis.png'
        try:
            plt.savefig(img_filename)
            print(f"[OK] 图片已保存: {img_filename}")
        except Exception as e:
            print(f"[Error] 图片保存失败: {e}")

        # 保存 CSV (包含推算的轨迹XY)
        csv_filename = 'trajectory_data.csv'
        try:
            with open(csv_filename, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['Time', 'Linear_X', 'Angular_Z', 'Pos_X', 'Pos_Y'])
                # 注意：traj_x 比 times 多一个初始点，这里对齐数据长度
                data_len = len(self.times)
                # 取 traj_x 的后 data_len 个点来对应时间
                # (或者你可以简单地把 times 前面补个0，这里我们只存对应指令时刻的位置)
                for i in range(data_len):
                    writer.writerow([
                        self.times[i], 
                        self.linear_x[i], 
                        self.angular_z[i],
                        self.traj_x[i+1], # +1 是因为 traj_x[0] 是初始0点
                        self.traj_y[i+1]
                    ])
            print(f"[OK] 数据已保存: {csv_filename}")
        except Exception as e:
            print(f"[Error] CSV保存失败: {e}")

        print(">>> 显示图表...")
        plt.show()

if __name__ == '__main__':
    plotter = TrajectoryPlotter(topic_name='/cmd_vel')
    
    try:
        rate = rospy.Rate(50) # 采样频率稍微高一点，计算积分更准
        while not rospy.is_shutdown():
            rate.sleep()
            
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        plotter.save_and_plot()

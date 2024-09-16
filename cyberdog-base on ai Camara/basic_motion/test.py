'''
This demo show the communication interface of MR813 motion control board based on Lcm.
Dependency: 
- robot_control_cmd_lcmt.py
- robot_control_response_lcmt.py
'''
import lcm
import sys
import os
import time
from threading import Thread, Lock

from robot_control_cmd_lcmt import robot_control_cmd_lcmt
from robot_control_response_lcmt import robot_control_response_lcmt

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from statistics import mean

import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import UnivariateSpline 
import scipy.interpolate as interp
from statsmodels.nonparametric.smoothers_lowess import lowess



def main(args=None):
    rclpy.init(args=args)
    laser_scan_listener = LaserScanListener()
    Ctrl = Robot_Ctrl(laser_scan_listener)
    Ctrl.run()

    laser_thread = Thread(target=rclpy.spin, args=(laser_scan_listener,)) # 新开一个线程，跑激光雷达订阅节点
    laser_thread.start()

    msg = robot_control_cmd_lcmt()
    try:
        msg.mode = 12 # Recovery stand
        msg.gait_id = 0
        msg.life_count += 1 # Command will take effect when life_count update
        Ctrl.Send_cmd(msg)
        Ctrl.Wait_finish(12, 0)

        Ctrl.fixAngle(msg) # 回正
        Ctrl.navigate(msg) # 回中
        
        # msg.mode = 16  
        # msg.gait_id = 1
        # msg.duration= 800
        # msg.life_count += 1
        # Ctrl.Send_cmd(msg)
        # Ctrl.Wait_finish(16, 1)

    
    except KeyboardInterrupt:
        pass
    Ctrl.quit()
    laser_scan_listener.destroy_node()
    rclpy.shutdown()
    laser_thread.join()
    sys.exit()


class LaserScanListener(Node):

    def __init__(self):
        super().__init__('laser_scan_listener')
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT  # 设置为RELIABILITY_BEST_EFFORT
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            qos_profile)
        self.subscription  # prevent unused variable warning
        self.left_dist = None
        self.right_dist = None
        self.list = []

    def listener_callback(self, msg):        
        # 有时收集到的某些雷达点数据小于0.1（0.2）属于无效数据，用样条插值法补全
        ranges = np.array(msg.ranges)
        ranges = self.interpolate_invalid_values(ranges)
        self.list = ranges.tolist()
        self.left_dist = mean(msg.ranges[:len(self.list)//18])  # 左侧
        self.right_dist = mean(msg.ranges[-len(self.list)//18:])  # 右侧
        

    def interpolate_invalid_values(self, ranges):
        # 将无效值（接近0的值）设置为np.nan
        ranges[ranges < 0.15] = np.nan
        # 获取非nan值的索引
        valid_indices = np.where(~np.isnan(ranges))[0]
        if valid_indices.size == 0:
            return ranges  # 如果没有有效值，直接返回原数组
        # 使用样条插值填充nan值
        cs = interp.CubicSpline(valid_indices, ranges[valid_indices])
        interpolated_ranges = cs(np.arange(len(ranges)))
        return interpolated_ranges


class Robot_Ctrl(object):
    def __init__(self, laser_scanner):
        self.laser_scanner = laser_scanner
        self.rec_thread = Thread(target=self.rec_responce)
        self.send_thread = Thread(target=self.send_publish)
        self.lc_r = lcm.LCM("udpm://239.255.76.67:7670?ttl=255")
        self.lc_s = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
        self.cmd_msg = robot_control_cmd_lcmt()
        self.rec_msg = robot_control_response_lcmt()
        self.send_lock = Lock()
        self.delay_cnt = 0
        self.mode_ok = 0
        self.gait_ok = 0
        self.runing = 1
        # self.msg = robot_control_cmd_lcmt()

    def run(self):
        self.lc_r.subscribe("robot_control_response", self.msg_handler)
        self.send_thread.start()
        self.rec_thread.start()

    def msg_handler(self, channel, data):
        self.rec_msg = robot_control_response_lcmt().decode(data)
        if(self.rec_msg.order_process_bar >= 95):
            self.mode_ok = self.rec_msg.mode
        else:
            self.mode_ok = 0

    def rec_responce(self):
        while self.runing:
            self.lc_r.handle()
            time.sleep( 0.002 )

    def Wait_finish(self, mode, gait_id):
        count = 0
        while self.runing and count < 1000: #5s
            if self.mode_ok == mode and self.gait_ok == gait_id:
                return True
            else:
                time.sleep(0.005)
                count += 1

    def Wait_finish_short(self, mode, gait_id):
        count = 0
        while self.runing and count < 500: #2.5s
            if self.mode_ok == mode and self.gait_ok == gait_id:
                return True
            else:
                time.sleep(0.005)
                count += 1


    def send_publish(self):
        while self.runing:
            self.send_lock.acquire()
            if self.delay_cnt > 20: # Heartbeat signal 10HZ, It is used to maintain the heartbeat when life count is not updated
                self.lc_s.publish("robot_control_cmd",self.cmd_msg.encode())
                self.delay_cnt = 0
            self.delay_cnt += 1
            self.send_lock.release()
            time.sleep( 0.005 )

    def Send_cmd(self, msg):
        self.send_lock.acquire()
        self.delay_cnt = 50
        self.cmd_msg = msg
        self.send_lock.release()

    def quit(self):
        self.runing = 0
        self.rec_thread.join()
        self.send_thread.join()
    
    def navigate(self, msg):
        print(self.laser_scanner.left_dist)
        print(self.laser_scanner.right_dist)
        print("\n")
        if self.laser_scanner.left_dist is not None and self.laser_scanner.right_dist is not None:
            error = self.laser_scanner.right_dist - self.laser_scanner.left_dist
            # 说明识别的至少有一边不是墙壁，算法失效，直接return
            if error>1 or error<-1:
                return
            # msg = robot_control_cmd_lcmt()
            # 设置步态和目标速度
            msg.mode = 11
            msg.gait_id = 27
            if error>0:
                msg.vel_des = [0, 0.15, 0]
            else:
                error = -error
                msg.vel_des = [0, -0.15, 0]
            msg.duration = int(2400*error)
            msg.step_height = [0.06, 0.06]            
            msg.life_count += 1
            self.Send_cmd(msg)
            self.Wait_finish(11, 27)


    def fixAngle(self, msg):
        print('回正')
        leftAngle = self.getAngle('left')
        rightAngle = self.getAngle('right')
        print(leftAngle)
        print(rightAngle)
        msg.mode = 11
        msg.gait_id = 27
        if leftAngle == 0 and rightAngle == 0:
            return
        # 大概率大角度的那一边是错误的识别，过滤
        if leftAngle != 0 and rightAngle != 0:
            if leftAngle>rightAngle:
                leftAngle = 0
            else:
                rightAngle = 0
        if(leftAngle == 0):
            msg.vel_des = [0, 0, 0.8]  # 调整角速度, 左转
            msg.duration = int((rightAngle/90)*2400)
        else:
            msg.vel_des = [0, 0, -0.8]  # 调整角速度，右转
            msg.duration = int((leftAngle/90)*2300)
        msg.step_height = [0.06, 0.06]
        msg.life_count += 1
        self.Send_cmd(msg)
        self.Wait_finish(11, 27)
    
    
    def getAngle(self, select):
        x_axis = []
        y_axis = []
        if select == 'right':
            for i in range(75):
                x_axis.append(i)
                y_axis.append(self.laser_scanner.list[i])
        else:
            for i in range(179, 104, -1):
                x_axis.append(180-i)
                y_axis.append(self.laser_scanner.list[i])
        
        # 创建x坐标，假设数据点是等间距的
        x = np.arange(len(y_axis))
        y = np.array(y_axis)

        # 使用局部加权回归（LOESS）进行平滑
        smoothed = lowess(y, x, frac=0.3)  # frac参数控制平滑度，值越大曲线越平滑

        # 提取平滑曲线的x和y值
        xs, ys = smoothed[:, 0], smoothed[:, 1]

        # 计算平滑曲线的最小值
        min_index = np.argmin(ys)
        min_x = xs[min_index] #这个是偏转角度值
        min_y = ys[min_index]
               
        '''
        # 可视化
        #把x轴的主刻度设置为5的倍数
        x_major_locator=MultipleLocator(5)
        ax=plt.gca()
        ax.xaxis.set_major_locator(x_major_locator)

        plt.plot(x_axis, y_axis, 'o', label='原始数据')
        plt.plot(xs, ys, label='平滑曲线')
        plt.plot(min_x, min_y, 'ro', label=f'最小值: ({min_x:.2f}, {min_y:.2f})')
        plt.legend()
        plt.show()
        '''
        
        return min_x
        

# Main function
if __name__ == '__main__':
    main()
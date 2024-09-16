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
import scipy.interpolate as interp
from statsmodels.nonparametric.smoothers_lowess import lowess

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def change_direction(direction, msg, Ctrl):
    if direction == 'left':
        msg.gait_id = 0
    elif direction == 'right':
        msg.gait_id = 3
    msg.mode = 16
    msg.life_count += 1
    Ctrl.Send_cmd(msg)
    Ctrl.Wait_finish(16, msg.gait_id)

def move_change(direction, msg, Ctrl, duration, speed, step_height = 0.03):
    if direction == 'left':
        msg.vel_des = [0, speed, 0]
    elif direction == 'right':
        speed = 0 - speed
        msg.vel_des = [0, speed, 0]
    elif direction == 'forward':
        msg.vel_des = [speed, 0, 0]
    elif direction == 'backward':
        speed = 0 - speed
        msg.del_des = [speed, 0, 0]
    msg.mode = 11
    msg.gait_id = 26
    msg.duration = duration
    msg.life_count += 1
    msg.step_height = [step_height, step_height]
    Ctrl.Send_cmd(msg)
    Ctrl.Wait_finish(11, 26)

def move(direction, msg, Ctrl, duration, speed):
    if direction == 'left':
        msg.vel_des = [0, speed, 0]
    elif direction == 'right':
        speed = 0 - speed
        msg.vel_des = [0, speed, 0]
    elif direction == 'forward':
        msg.vel_des = [speed, 0, 0]
    elif direction == 'backward':
        speed = 0 - speed
        msg.del_des = [speed, 0, 0]
    msg.mode = 11
    msg.gait_id = 27
    msg.duration = duration
    msg.life_count += 1
    # msg.step_height = [step_height, step_height]
    Ctrl.Send_cmd(msg)
    Ctrl.Wait_finish(11, 27)

def stand(msg, Ctrl):
    msg.mode = 12
    msg.gait_id = 0
    msg.duration = 1000
    msg.life_count += 1
    Ctrl.Send_cmd(msg)
    Ctrl.Wait_finish(12, 0)

def main(args=None):
    rclpy.init(args=args)

    # laser_scan_listener = LaserScanListener()
    # image_subscriber = ImageSubscriber()
    sensor_subscriber = SensorSubscriber()
    spin_thread = Thread(target=rclpy.spin, args=(sensor_subscriber,)) # 新开一个线程，跑激光雷达订阅节点
    spin_thread.start()

    
    # image_thread = Thread(target=rclpy.spin, args=(image_subscriber,))
    # image_thread.start()

    

    Ctrl = Robot_Ctrl(sensor_subscriber)
    Ctrl.run()
    msg = robot_control_cmd_lcmt()

    try:
        msg.mode = 12 # Recovery stand
        msg.gait_id = 0
        msg.life_count += 1 # Command will take effect when life_count update
        Ctrl.Send_cmd(msg)
        Ctrl.Wait_finish(12, 0)
        # move('left', msg, Ctrl, 700)

        move_change('forward', msg, Ctrl, 1900, 0.9)
        time.sleep(1)
        Ctrl.fixAngle(msg)  # 居正
        Ctrl.navigate(msg)  # 回中
        move('left', msg, Ctrl, 250, 0.3)
        move_change('forward', msg, Ctrl, 9600, 0.455, 0.12)
        stand(msg, Ctrl)
        # move('forward', msg, Ctrl, 3000, 0.4)


        
        # time.sleep(1)
        stand(msg ,Ctrl)
        # move('forward', msg, Ctrl, 1000)
        # time.sleep(60)
        direction = 'unknown'
        while rclpy.ok():
            print(direction)
            if sensor_subscriber.green_cx is not None and sensor_subscriber.red_cx is not None:
                print("green_cx is {}".format(sensor_subscriber.green_cx))
                print("red_cx is {}".format(sensor_subscriber.red_cx))
                print("redcount is {}".format(sensor_subscriber.red_count))
                print("greencount is {}".format(sensor_subscriber.green_count))
                if sensor_subscriber.green_count > 2000 and sensor_subscriber.red_count > 2000:  # 确保看到的是两块布而不是其它干扰因素
                    if sensor_subscriber.green_cx < sensor_subscriber.red_cx and direction != 'left':
                        Ctrl.fixAngle(msg)  # 居正
                        Ctrl.navigate(msg)  # 回中
                        sensor_subscriber.get_logger().info('Moving left towards green object.')
                        move('left', msg, Ctrl, 900, 0.3)
                        direction = 'left'
                    elif sensor_subscriber.green_cx > sensor_subscriber.red_cx and direction !='right':
                        Ctrl.fixAngle(msg)  # 居正
                        Ctrl.navigate(msg)  # 回中
                        sensor_subscriber.get_logger().info('Moving right towards green object.')
                        move('right', msg, Ctrl, 900, 0.3)
                        direction = 'right'
                elif sensor_subscriber.green_count > 2000 and sensor_subscriber.red_count < 1000:
                    pass
                elif sensor_subscriber.green_count < 1000 and sensor_subscriber.red_count > 2000:
                    Ctrl.fixAngle(msg)  # 居正
                    Ctrl.navigate(msg)  # 回中
                    if direction == 'left':
                        sensor_subscriber.get_logger().info('Moving right towards green object.')
                        move('right', msg, Ctrl, 900, 0.3)
                        direction = 'right'
                    elif direction == 'right':
                        sensor_subscriber.get_logger().info('Moving left towards green object.')
                        move('left', msg, Ctrl, 900, 0.3)
                        direction = 'left'
            elif sensor_subscriber.green_cx is None and sensor_subscriber.red_cx is not None:
                print("全红\n")
                Ctrl.fixAngle(msg)  # 居正
                Ctrl.navigate(msg)  # 回中
                if direction == 'left':
                    sensor_subscriber.get_logger().info('Moving right towards green object.')
                    move('right', msg, Ctrl, 900, 0.3)
                    direction = 'right'
                elif direction == 'right':
                    sensor_subscriber.get_logger().info('Moving left towards green object.')
                    move('left', msg, Ctrl, 900, 0.3)
                    direction = 'left'
            else:
                print("全绿\n")
                pass
              
            move('forward', msg, Ctrl, 1300, 1.0)
        move('forward', msg, Ctrl)
        
    
    except KeyboardInterrupt:
        pass
    
    # laser_scan_listener.destroy_node()
    # image_subscriber.destroy_node()
    # laser_thread.join()
    # image_thread.join()
    sensor_subscriber.destroy_node()
    spin_thread.join()
    Ctrl.quit()
    rclpy.shutdown()
    sys.exit()

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.green_cx = 0
        self.red_cx = 0
        self.green_count = 0
        self.red_count = 0
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT
        self.subscription = self.create_subscription(
            Image,
            '/rgb_camera/image_raw',
            self.image_callback,
            qos_profile,
        )
        self.subscription  # 防止未使用变量警告
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # 将ROS消息转换为OpenCV图像格式
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 定义红色和绿色的颜色范围（BGR格式）
        lower_red = np.array([0, 20, 40])
        upper_red = np.array([200, 30, 55])
        lower_green = np.array([0, 60, 0])
        upper_green = np.array([200, 80, 200])

        # 创建红色和绿色的掩膜
        red_mask = cv2.inRange(cv_image, lower_red, upper_red)
        green_mask = cv2.inRange(cv_image, lower_green, upper_green)

        # 计算红色和绿色掩膜的像素数量
        self.red_count = np.sum(red_mask > 0)
        self.green_count = np.sum(green_mask > 0)
        
        # 计算红色掩膜的质心
        red_moments = cv2.moments(red_mask)
        if red_moments["m00"] != 0:
            self.red_cx = int(red_moments["m10"] / red_moments["m00"])
            self.red_cy = int(red_moments["m01"] / red_moments["m00"])
        else:
            self.red_cx, self.red_cy = None, None  # 无法找到红色质心

        # 计算绿色掩膜的质心
        green_moments = cv2.moments(green_mask)
        if green_moments["m00"] != 0:
            self.green_cx = int(green_moments["m10"] / green_moments["m00"])
            self.green_cy = int(green_moments["m01"] / green_moments["m00"])
        else:
            self.green_cx, self.green_cy = None, None  # 无法找到绿色质心

        # 在原始图像上绘制质心
        if self.red_cx is not None and self.red_cy is not None:
            cv2.circle(cv_image, (self.red_cx, self.red_cy), 5, (0, 0, 255), -1)  # 红色质心用蓝色表示
        if self.green_cx is not None and self.green_cy is not None:
            cv2.circle(cv_image, (self.green_cx, self.green_cy), 5, (0, 255, 0), -1)  # 绿色质心用绿色表示

        # 显示原始图像和掩膜图像
        cv2.imshow('Original Image', cv_image)
        cv2.imshow('Red Mask', red_mask)
        cv2.imshow('Green Mask', green_mask)

        cv2.waitKey(1)  # 等待1毫秒，允许OpenCV处理窗口事件

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
    
class SensorSubscriber(Node):
    def __init__(self):
        super().__init__('sensor_listener')
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT  # 设置为RELIABILITY_BEST_EFFORT
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            qos_profile)
        self.left_dist = None
        self.right_dist = None
        self.list = []

        self.green_cx = 0
        self.red_cx = 0
        self.green_count = 0
        self.red_count = 0
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT
        self.subscription = self.create_subscription(
            Image,
            '/rgb_camera/image_raw',
            self.image_callback,
            qos_profile,
        )
        self.subscription  # 防止未使用变量警告
        self.bridge = CvBridge()

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
        # 使用np.isfinite()检查哪些元素是有限值（非NaN）
        finite_values = np.isfinite(ranges)
        # 使用np.where()获取非NaN值的索引
        valid_indices = np.where(finite_values)[0]
        # valid_indices = np.where(~np.isnan(ranges))[0]
        if valid_indices.size == 0:
            return ranges  # 如果没有有效值，直接返回原数组
        # 使用样条插值填充nan值
        try:
            cs = interp.CubicSpline(valid_indices, ranges[valid_indices])
        except Exception:
            pass
        interpolated_ranges = cs(np.arange(len(ranges)))
        return interpolated_ranges
    
    def image_callback(self, msg):
        # 将ROS消息转换为OpenCV图像格式
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 定义红色和绿色的颜色范围（BGR格式）
        lower_red = np.array([0, 20, 40])
        upper_red = np.array([200, 30, 55])
        lower_green = np.array([0, 60, 0])
        upper_green = np.array([200, 80, 200])

        # 创建红色和绿色的掩膜
        red_mask = cv2.inRange(cv_image, lower_red, upper_red)
        green_mask = cv2.inRange(cv_image, lower_green, upper_green)

        # 计算红色和绿色掩膜的像素数量
        self.red_count = np.sum(red_mask > 0)
        self.green_count = np.sum(green_mask > 0)
        
        # 计算红色掩膜的质心
        red_moments = cv2.moments(red_mask)
        if red_moments["m00"] != 0:
            self.red_cx = int(red_moments["m10"] / red_moments["m00"])
            self.red_cy = int(red_moments["m01"] / red_moments["m00"])
        else:
            self.red_cx, self.red_cy = None, None  # 无法找到红色质心

        # 计算绿色掩膜的质心
        green_moments = cv2.moments(green_mask)
        if green_moments["m00"] != 0:
            self.green_cx = int(green_moments["m10"] / green_moments["m00"])
            self.green_cy = int(green_moments["m01"] / green_moments["m00"])
        else:
            self.green_cx, self.green_cy = None, None  # 无法找到绿色质心

        # 在原始图像上绘制质心
        if self.red_cx is not None and self.red_cy is not None:
            cv2.circle(cv_image, (self.red_cx, self.red_cy), 5, (0, 0, 255), -1)  # 红色质心用蓝色表示
        if self.green_cx is not None and self.green_cy is not None:
            cv2.circle(cv_image, (self.green_cx, self.green_cy), 5, (0, 255, 0), -1)  # 绿色质心用绿色表示

        # 显示原始图像和掩膜图像
        cv2.imshow('Original Image', cv_image)
        cv2.imshow('Red Mask', red_mask)
        cv2.imshow('Green Mask', green_mask)

        cv2.waitKey(1)  # 等待1毫秒，允许OpenCV处理窗口事件


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
        while self.runing and count < 2000: #10s
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
            for i in range(178, 104, -1):
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
'''
This demo show the communication interface of MR813 motion control board based on Lcm.
Dependency: 
- robot_control_cmd_lcmt.py
- robot_control_response_lcmt.py
'''
import lcm
import sys
import os
import threading
import time
from threading import Thread, Lock
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

from rclpy.qos import QoSProfile, ReliabilityPolicy
from robot_control_cmd_lcmt import robot_control_cmd_lcmt
from robot_control_response_lcmt import robot_control_response_lcmt

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
        


def change_direction(direction, msg, Ctrl):
    if direction == 'left':
        msg.gait_id = 0
    elif direction == 'right':
        msg.gait_id = 3
    msg.mode = 16
    msg.life_count += 1
    Ctrl.Send_cmd(msg)
    Ctrl.Wait_finish(16, msg.gait_id)


def move(direction, msg, Ctrl, duration):
    if direction == 'left':
        msg.vel_des = [0, 0.3, 0]
    elif direction == 'right':
        msg.vel_des = [0, -0.3, 0]
    elif direction == 'forward':
        msg.vel_des = [1.0, 0, 0]
    elif direction == 'backward':
        msg.del_des = [-1.0, 0, 0]
    msg.mode = 11
    msg.gait_id = 3
    msg.duration = duration
    msg.life_count += 1
    Ctrl.Send_cmd(msg)
    Ctrl.Wait_finish(11, 3)

def stand(msg, Ctrl):
    msg.mode = 12
    msg.gait_id = 0
    msg.duration = 1000
    msg.life_count += 1
    Ctrl.Send_cmd(msg)
    Ctrl.Wait_finish(12, 0)



def main(args=None):
    Ctrl = Robot_Ctrl()
    Ctrl.run()
    msg = robot_control_cmd_lcmt()
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    # 创建一个线程来运行 rclpy.spin
    spin_thread = threading.Thread(target=rclpy.spin, args=(image_subscriber,))
    spin_thread.start()
    try:
        
        msg.mode = 12 # Recovery stand
        msg.gait_id = 0
        msg.life_count += 1 # Command will take effect when life_count update
        Ctrl.Send_cmd(msg)
        Ctrl.Wait_finish(12, 0)

        # msg.mode = 16
        # msg.gait_id = 0
        # msg.life_count += 1
        # Ctrl.Send_cmd(msg)
        # Ctrl.Wait_finish(16, 0)
        # change_direction('left', msg, Ctrl)
        # time.sleep(1)
        # stand(msg ,Ctrl)
        # move('forward', msg, Ctrl, 1000)
        direction = 'unknown'
        while rclpy.ok():
            if image_subscriber.green_cx is not None and image_subscriber.red_cx is not None:
                print("green_cx{}".format(image_subscriber.green_cx))
                print("red_cx{}".format(image_subscriber.red_cx))
                print("redcount is{}".format(image_subscriber.red_count))
                print("greencount is{}".format(image_subscriber.green_count))
                if image_subscriber.green_count > 2000 and image_subscriber.red_count > 2000:  # 确保看到的是两块布而不是其它干扰因素
                    if image_subscriber.green_cx < image_subscriber.red_cx and direction != 'left':
                        image_subscriber.get_logger().info('Moving left towards green object.')
                        move('left', msg, Ctrl, 1100)
                        direction = 'left'
                    elif image_subscriber.green_cx > image_subscriber.red_cx and direction !='right':
                        image_subscriber.get_logger().info('Moving right towards green object.')
                        move('right', msg, Ctrl, 1100)
                        direction = 'right'
                elif image_subscriber.green_count > 2000 and image_subscriber.red_count < 1000:
                    pass
                elif image_subscriber.green_count < 1000 and image_subscriber.red_count > 2000:
                    if direction == 'left':
                        image_subscriber.get_logger().info('Moving right towards green object.')
                        move('right', msg, Ctrl, 1100)
                        direction = 'right'
                    elif direction == 'right':
                        image_subscriber.get_logger().info('Moving left towards green object.')
                        move('left', msg, Ctrl, 1100)
                        direction = 'left'
            move('forward', msg, Ctrl, 1200)

        move('forward', msg, Ctrl)

        msg.mode = 11   # 测试快跑
        msg.gait_id = 3
        msg.vel_des=[0,0.3,0]
        msg.duration=1000
        #msg.rpy_des=0
        #msg.pos_des=0.25
        #msg.step_height=0.04
        msg.life_count += 1
        Ctrl.Send_cmd(msg)
        Ctrl.Wait_finish(11, 3)
        '''
        msg.mode = 62 # Shake hand, based on position interpolation control
        msg.gait_id = 2
        msg.life_count += 1
        Ctrl.Send_cmd(msg)
        Ctrl.Wait_finish(62, 2)
         
        msg.mode = 64 # Twoleg Stand
        msg.gait_id = 0
        msg.life_count += 1
        Ctrl.Send_cmd(msg)
        Ctrl.Wait_finish(64, 0)

        msg.mode = 21 # Position interpolation control
        msg.gait_id = 0
        msg.rpy_des = [0, 0.3, 0] # Head up
        msg.duration = 500 # Expected execution time, 0.5s 
        msg.life_count += 1
        Ctrl.Send_cmd(msg)
        time.sleep( 0.5 )

        msg.mode = 21 # Position interpolation control
        msg.gait_id = 0
        msg.rpy_des = [0, -0.3, 0] # Head down
        msg.duration = 300 
        msg.life_count += 1
        Ctrl.Send_cmd(msg)
        time.sleep( 0.3 )

        msg.mode = 21 # Position interpolation control
        msg.gait_id = 5
        msg.rpy_des = [0, 0, 0]
        msg.pos_des = [0, 0, 0.22] # Set body height
        msg.duration = 400 
        msg.life_count += 1
        Ctrl.Send_cmd(msg)
        time.sleep( 1 )

        msg.mode = 11 # Locomotion
        msg.gait_id = 26 # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
        msg.vel_des = [0, 0, 0.5] #转向
        msg.duration = 0 # Zero duration means continuous motion until a new command is used.
                         # Continuous motion can interrupt non-zero duration interpolation motion
        msg.step_height = [0.06, 0.06] # ground clearness of swing leg
        msg.life_count += 1
        Ctrl.Send_cmd(msg)
        time.sleep( 5 )

        msg.mode = 7    # PureDamper
        msg.gait_id = 0
        msg.life_count += 1
        Ctrl.Send_cmd(msg)
        Ctrl.Wait_finish(7, 0)
        
        '''
        msg.mode = 11   # 测试快跑
        msg.gait_id = 7
        msg.vel_des=[0.1,0.1,1.5]
        msg.duration=100000
        msg.life_count += 1
        Ctrl.Send_cmd(msg)
        image_subscriber.destroy_node()
        rclpy.shutdown()
        

    except KeyboardInterrupt:
        pass
    Ctrl.quit()
    sys.exit()
    image_subscriber.destroy_node()
    rclpy.shutdown()


class Robot_Ctrl(object):
    def __init__(self):
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

# Main function
if __name__ == '__main__':
    main()

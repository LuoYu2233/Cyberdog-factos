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
import time

from rclpy.qos import QoSProfile, ReliabilityPolicy
from robot_control_cmd_lcmt import robot_control_cmd_lcmt
from robot_control_response_lcmt import robot_control_response_lcmt

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.green_cx = 0
        self.green_cy = 0
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
        # self.txt_file = open('green_channel_data.txt', 'w')  # 打开文件准备写入数据

    def image_callback(self, msg):
        # 将ROS消息转换为OpenCV图像格式
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 定义绿色的颜色范围（BGR格式）
        lower_green = np.array([70, 110, 40])
        upper_green = np.array([80, 120, 50])

        # 创建绿色的掩膜
        green_mask = cv2.inRange(cv_image, lower_green, upper_green)

        # 计算绿色掩膜的像素数量
        self.green_count = np.sum(green_mask > 0)

        # 保存绿色通道的数据到txt文件
        green_channel_data = cv_image[:,:,1]  # 绿色通道是第2个通道 (0为蓝色, 1为绿色, 2为红色)
        # np.savetxt(self.txt_file, green_channel_data, fmt='%d', delimiter=' ')
        # cv2.imwrite('original_image.png', cv_image)
        # cv2.imwrite('green_mask.png', green_mask)

        # 计算绿色掩膜的质心
        green_moments = cv2.moments(green_mask)
        if green_moments["m00"] != 0:
            self.green_cx = int(green_moments["m10"] / green_moments["m00"])
            self.green_cy = int(green_moments["m01"] / green_moments["m00"])
        else:
            self.green_cx, self.green_cy = None, None  # 无法找到绿色质心

        # 在原始图像上绘制质心
        if self.green_cx is not None and self.green_cy is not None:
            cv2.circle(cv_image, (self.green_cx, self.green_cy), 5, (0, 255, 0), -1)  # 绿色质心用绿色表示

        # 显示原始图像和掩膜图像
        cv2.imshow('Original Image', cv_image)
        cv2.imshow('Green Mask', green_mask)

        cv2.waitKey(1)  # 等待1毫秒，允许OpenCV处理窗口事件

    def destroy_node(self):
        super().destroy_node()


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
    msg.duration = 10000
    msg.life_count += 1
    Ctrl.Send_cmd(msg)
    Ctrl.Wait_finish(12, 0)
def main(args=None):
    Ctrl = Robot_Ctrl()
    Ctrl.run()
    msg = robot_control_cmd_lcmt()
    rclpy.init(args=args)
    # image_subscriber = ImageSubscriber()
    # # 创建一个线程来运行 rclpy.spin
    # spin_thread = threading.Thread(target=rclpy.spin, args=(image_subscriber,))
    # spin_thread.start()
    try:
        
        msg.mode = 12 # Recovery stand
        msg.gait_id = 0
        msg.life_count += 1 # Command will take effect when life_count update
        Ctrl.Send_cmd(msg)
        Ctrl.Wait_finish(12, 0)

        msg.mode = 3 # Recovery stand
        msg.gait_id = 0
        msg.life_count += 1 # Command will take effect when life_count update
        msg.rpy_des = [0 ,0.2, 0]
        msg.pos_des = [0.5, 0, 0.3]
        msg.contact = 0b1110
        msg.duration = 1000
        Ctrl.Send_cmd(msg)
        Ctrl.Wait_finish(3, 0)
        print('2')

        while True:
            time.sleep(1)
            stand(msg, Ctrl)
        while image_subscriber.green_count> 1000:
            stand(msg, Ctrl)

        # move('forward', msg, Ctrl)

        # msg.mode = 11   # 测试快跑
        # msg.gait_id = 3
        # msg.vel_des=[0,0.3,0]
        # msg.duration=1000
        # #msg.rpy_des=0
        # #msg.pos_des=0.25
        # #msg.step_height=0.04
        # msg.life_count += 1
        # Ctrl.Send_cmd(msg)
        # Ctrl.Wait_finish(11, 3)

        # msg.mode = 11   # 测试快跑
        # msg.gait_id = 7
        # msg.vel_des=[0.1,0.1,1.5]
        # msg.duration=100000
        # msg.life_count += 1
        # Ctrl.Send_cmd(msg)


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

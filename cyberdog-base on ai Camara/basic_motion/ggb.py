import numpy as np  
from scipy.signal import medfilt  
from skimage.transform import hough_line, hough_line_peaks  




def getAngle(self):  
    # 获取激光扫描数据  
    ranges = np.array(self.laser_scanner.list)  
    angles = np.linspace(0, np.pi, len(ranges))  

    # 数据预处理  
    # 1. 中值滤波去除噪声  
    filtered_ranges = medfilt(ranges, kernel_size=5)  

    # 2. 将距离数据转换为笛卡尔坐标  
    x = filtered_ranges * np.cos(angles)  
    y = filtered_ranges * np.sin(angles)  

    # 3. 移除无效点（太远或太近的点）  
    valid = (filtered_ranges > 0.1) & (filtered_ranges < 10)  
    x = x[valid]  
    y = y[valid]  

    # 创建二值图像  
    resolution = 0.05  # 5cm分辨率  
    x_max, x_min = x.max(), x.min()  
    y_max, y_min = y.max(), y.min()  
    width = int((x_max - x_min) / resolution)  
    height = int((y_max - y_min) / resolution)  
    image = np.zeros((height, width), dtype=np.uint8)  

    # 将点云数据映射到图像上  
    ix = ((x - x_min) / resolution).astype(int)  
    iy = ((y - y_min) / resolution).astype(int)  
    image[iy, ix] = 1  

    # 霍夫变换  
    h, theta, d = hough_line(image)  

    # 提取最显著的直线  
    _, angles, _ = hough_line_peaks(h, theta, d, num_peaks=2)  

    # 计算墙壁的平均角度  
    wall_angle = np.mean(angles)  

    # 计算机器人相对于墙壁的角度  
    robot_angle = np.pi/2 - wall_angle  

    # 转换为度数并返回  
    return np.degrees(robot_angle)  



def fixAngle(self, msg):  
    print("将调整角度")  
    
    # 使用新的 getAngle 函数获取当前角度  
    current_angle = self.getAngle()  
    
    print("当前角度： " + str(current_angle))  
    
    # 定义角度阈值，小于此值认为已经对齐  
    ANGLE_THRESHOLD = 2.0  # 度  
    
    if abs(current_angle) < ANGLE_THRESHOLD:  
        print("已经对齐...")  
        return  
    
    msg.mode = 11  # 设置运动模式为11（通常是步态运动模式）  
    msg.gait_id = 27  # 设置步态ID为27  
    
    # 根据角度的正负决定转向方向  
    if current_angle > 0:  
        print("向右转...")  
        msg.vel_des = [0, 0, -0.8]  # 设置角速度，向右转  
        msg.duration = int((current_angle / 90) * 2400)  # 右转：90度对应2400ms  
    else:  
        print("向左转...")  
        msg.vel_des = [0, 0, 0.8]  # 设置角速度，向左转  
        msg.duration = int((abs(current_angle) / 90) * 2300)  # 左转：90度对应2300ms  
    
    msg.step_height = [0.06, 0.06]  # 设置步高  
    msg.life_count += 1  # 增加生命计数  
    
    print("调整时长：" + str(msg.duration))  
    
    self.Send_cmd(msg)  # 发送控制命令  
    self.Wait_finish_short(11, 27)  # 等待命令执行完成  
    
    # 再次检查角度，如果还没有对齐，可以考虑递归调用  
    new_angle = self.getAngle()  
    if abs(new_angle) >= ANGLE_THRESHOLD:  
        print("角度还未完全对齐，进行微调")  
        self.fixAngle(msg)
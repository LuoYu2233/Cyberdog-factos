class SensorSubscriber(Node):
    def __init__(self):
        super().__init__('sensor_listener')
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT  # 设置为RELIABILITY_BEST_EFFORT
        self.subscription = self.create_subscription(
            LaserScan,
            '/mi_desktop_48_b0_2d_7b_06_b6/scan',
            self.listener_callback,
            qos_profile)
        self.left_dist = None
        self.right_dist = None
        self.left_list = []
        self.right_list = []
        self.list = []


    def listener_callback(self, msg):        
        # 有时收集到的某些雷达点数据小于0.1（0.2）属于无效数据，用样条插值法补全
        ranges = np.array(msg.ranges)
        ranges = self.interpolate_invalid_values(ranges)
        self.list = ranges.tolist()
        self.left_list = msg.ranges[:len(self.list)//12]  # 左侧
        self.right_list = msg.ranges[-len(self.list)//12:]  # 右侧



    def interpolate_invalid_values(self, ranges):
        # 将无效值（接近0的值）设置为np.nan
        ranges[ranges < 0.15] = np.nan
        # 获取非nan值的索引
        # 使用np.isfinite()检查哪些元素是有限值（非NaN）
        finite_values = np.isfinite(ranges)
        # 使用np.where()获取非NaN值的索引
        valid_indices = np.where(finite_values)[0]
        # valid_indices = np.where(~np.isnan(ranges))[0]
        if valid_indices.size <= 2:
            return ranges  # 如果有效值少于2，直接返回原数组
        # 使用样条插值填充nan值
        try:
            cs = interp.CubicSpline(valid_indices, ranges[valid_indices])
            interpolated_ranges = cs(np.arange(len(ranges)))
        except Exception:
            return ranges
        return interpolated_ranges
    
    def navigate(self, msg, error=0):  
        # 定义导航函数，接受消息对象和可选的error参数  

        if(error == 0): # 如果没传入error,则需要通过传感器获取error  
            print("没传入erro值将通过传感器获得")    
            error = self.getSide()  # 调用getSide()函数获取两侧距离差  

        if(error != 0):  # 如果error不为0，说明需要进行调整  
            # 设置步态和目标速度  
            print("需要调整")  
            msg.mode = 11  # 设置运动模式为11（可能是某种特定的运动模式）  
            msg.gait_id = 27  # 设置步态ID为27  

            if error > 0:  # 如果error大于0，说明右侧距离大于左侧
                print("右侧大于左侧")  
                msg.vel_des = [0, 0.15, 0]  # 设置向右移动的速度  
            else:  # 如果error小于0，说明左侧距离大于右侧  
                print("左侧大于右侧")  
                error = -error  # 将error取绝对值  
                msg.vel_des = [0, -0.15, 0]  # 设置向左移动的速度  

            # 根据error大小设置运动持续时间，error越大，时间越长  
            msg.duration = int(2700*error)  
            print("调整时长" + str(msg.duration))  

            msg.step_height = [0.04, 0.04]  # 设置步高  
            
            msg.life_count += 1  # 增加生命计数  

            self.Send_cmd(msg)  # 发送控制命令  
            self.Wait_finish(11, 27)  # 等待命令执行完成

    def getDistance(self, angle):
        
        return self.laser_scanner.list[angle]

    def getSide(self):  
        # 定义获取两侧距离差的函数  

        # 获取左侧距离，使用左侧列表的第一个元素  
        self.laser_scanner.left_dist = self.laser_scanner.left_list[0]  
        
        # 获取右侧距离，使用右侧列表的
        self.laser_scanner.right_dist = self.laser_scanner.right_list[-1]  
        
        # 打印左侧距离  
        print(self.laser_scanner.left_dist)  
        # 打印右侧距离  
        print(self.laser_scanner.right_dist)  
        # 打印一个空行，用于分隔输出  
        print("\n")  

        # 检查左右两侧的距离是否都有有效值  
        if self.laser_scanner.left_dist is not None and self.laser_scanner.right_dist is not None:  
            # 计算右侧距离减去左侧距离的差值  
            error = self.laser_scanner.right_dist - self.laser_scanner.left_dist  
            
            # 如果差值大于1或小于-1，说明至少有一边可能不是墙壁，算法失效  
            if error > 1 or error < -1:  
                return 0  # 返回0，表示无法确定有效的距离差  
            else:  
                return error  # 返回计算出的距离差  
        else:  
            return 0  # 如果任一侧距离为None，返回0  


    def fixAngle(self, msg, leftAngle = -1, rightAngle = -1):  
        # 定义修正角度的函数，接受消息对象和可选的左右角度参数  
        print("将调整角度")  
        if leftAngle == -1: # 如果没传入左右角度值,则需要通过传感器获取角度值  
            leftAngle = self.getAngle('left')  # 获取左侧角度  
            rightAngle = self.getAngle('right')  # 获取右侧角度  
        
        print("左侧角度： " + str(leftAngle))  # 打印左侧角度  
        print("右侧角度： " + str(rightAngle))  # 打印右侧角度  
        
        msg.mode = 11  # 设置运动模式为11（通常是步态运动模式）  
        msg.gait_id = 27  # 设置步态ID为27  
        
        if leftAngle == 0 and rightAngle == 0:  
            print("已经对齐...")    
            return  # 如果左右角度都为0，说明已经对齐，直接返回  
        
        # 大概率大角度的那一边是错误的识别，过滤  
        if leftAngle != 0 and rightAngle != 0:  
            if leftAngle > rightAngle:  
                print("左边角度大")    
                leftAngle = 0  # 如果左角度大，认为左侧识别错误，置为0  
            else:  
                print("右边角度大...")    
                rightAngle = 0  # 否则认为右侧识别错误，置为0  
        
        if(leftAngle == 0):  
            print("向左转...")    
            msg.vel_des = [0, 0, 0.8]  # 设置角速度，向左转  
            msg.duration = int((rightAngle/90)*2400)  # 根据右侧角度计算转动持续时间  
        else:  
            print("向右转...")    
            msg.vel_des = [0, 0, -0.8]  # 设置角速度，向右转  
            msg.duration = int((leftAngle/90)*2300)  # 根据左侧角度计算转动持续时间  
        
        msg.step_height = [0.06, 0.06]  # 设置步高  
        msg.life_count += 1  # 增加生命计数  
        
        self.Send_cmd(msg)  # 发送控制命令  
        self.Wait_finish_short(11, 27)  # 等待命令执行完成
        
        
    def getAngle(self, select):
        total_points = len(self.laser_scanner.list)  
        angle_range = 45  # 假设我们仍然想要覆盖45度范围  
        points_per_degree = total_points / 180  # 假设180度对应全部数据点  
        side_points = int(angle_range * points_per_degree)  

        x_axis = []  
        y_axis = []  

        if select == 'right':  
            for i in range(side_points):  
                x_axis.append(i)  
                y_axis.append(self.laser_scanner.list[i])  
        else:  # left side  
            for i in range(total_points - 1, total_points - side_points - 1, -1):  
                x_axis.append(total_points - 1 - i)  
                y_axis.append(self.laser_scanner.list[i])  

        # 后续的数据处理代码保持不变  
        x = np.arange(len(y_axis))  
        y = np.array(y_axis)  
        smoothed = lowess(y, x, frac=0.3)  
        xs, ys = smoothed[:, 0], smoothed[:, 1]  
        min_index = np.argmin(ys)  
        min_x = xs[min_index]  
        min_y = ys[min_index]  

        # 计算实际角度  
        actual_angle = (min_x / side_points) * angle_range  

        return actual_angle  # 或者根据需要返回其他值
    
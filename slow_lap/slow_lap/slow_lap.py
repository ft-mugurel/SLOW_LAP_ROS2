#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan 
from fs_msgs.msg import ControlCommand
import numpy as np
import matplotlib.pyplot as plt

coun = 0
MAX_RANGE = 7.0

def inf_to_max(data):
    filiter_data = []
    for check in data:
        if str(check) == "inf" or check > MAX_RANGE:
            filiter_data.append(MAX_RANGE)
        else:
            filiter_data.append(check)
    return filiter_data

def cone_count(data):
    count = 0
    last = False
    for det in data:
        if last == False and det < MAX_RANGE:
            last = True
        elif last == True and det == MAX_RANGE:
            count = count + 1
            last = False
    return count

def cones_data(data):
    global angle, last
    cone_ranges = []
    cone_angles = []
    cones_data = []
    angle = 0
    last = False
    cone_count = 0
    for det in data:
        angle = angle + 1
        if last == False and det < 5:
            cone_ranges.append(det)
            cone_angles.append(angle)
            last = True
        elif last == True and det < 5:
            cone_ranges.append(det)
            cone_angles.append(angle)
        elif last == True and det == 5:
            cones_data.append(cone_ranges)
            cones_data.append(cone_angles)
            cone_ranges = []
            cone_angles = []
            cone_count = cone_count + 1 
            last = False
            
    return cones_data

def cones_opt(data):
    count = 0
    data_fil = []
    for det in data:
        count = count +1
        if (int(count)%2==0):
            data_fil.append(np.mean(det))
        else:
            data_fil.append(np.min(det))
    return data_fil

def cones_position(data):
    global x, y, range, angles, xy_ratio
    count = 0
    angles = 0
    range = 0
    x = 0.0
    y = 0.0
    xy_ratio = 0
    conex = []
    coney = []
    cones = []
    for det in data:
        count = count + 1
        if (int(count)%2==0):
            angles = det
            if angles < 528:
                xy_ratio = (angles/528)
                y = (range*xy_ratio)
                x = (range-y)
            else:
                xy_ratio = ((angles-528)/528)
                x = (0-(range*xy_ratio))
                y = (range-(range*xy_ratio))
            conex.append(x)
            coney.append(y)
        elif (int(count)%2==1):
            range = det
    cones.append(conex)
    cones.append(coney)
    conex = []
    coney = []
    plt.axis([-20, 20, -20, 20])
    plt.plot(cones[0], cones[1], 'ro')
    plt.pause(0.000000000000000001)
    return cones

def stering(data):
    #count0 = -1
    #count1 = -1
    #count2 = -1
    #min_negative_y = 0
    #min_positive_y = 0
    #negative_number_x = []
    #positive_number_x = []
    #for det0 in data[0]:
    #    count0 = count0 + 1
    #    if det0 < 0:
    #        negative_number_x.append(count0)
    #    elif det0 >= 0:
    #        positive_number_x.append(count0)
    #for det1 in data[1]:
    #    count1 = count1 + 1
    #    for num0 in negative_number_x:
    #        if count1 == num0:
    #            if min_negative_y > det1: 
    #                min_negative_y = det1
    #for det2 in data[1]:
    #    count2 = count2 + 1
    #    for num1 in positive_number_x:
    #        if count2 == num1:
    #            if min_positive_y > det2:
    #                min_positive_y = det2
    max_x = np.max(data[0])
    min_x = np.min(data[0])
    track = ((abs(max_x))+(abs(min_x)))
    steering_angle = (((((track - abs(max_x))*1)/track)*2)-1)
    return steering_angle

def real_time_plot(data):
    global coun
    coun = coun + 1
    if int(coun) % 5 == 0:
        x_cords = data[0]
        y_cords = data[1]
        print(x_cords, y_cords)
        plt.axis([-10, 10, -10, 10])
        plt.plot(x_cords, y_cords, 'ro')
        plt.pause(0.000000000000000001)

def plot_lidar_data(data):
    i = 0
    conex = []
    coney = []
    conex.append(0)
    coney.append(0)
    for dat in data:
        i = i + 1
        if dat < MAX_RANGE:
            degree = (i*360.0)/(len(data)*2)
            y = (dat*math.sin(math.radians(degree)))
            x = (dat*math.cos(math.radians(degree)))
            if y < 4:
                conex.append(x)
                coney.append(y)
    max_x = (np.max(conex)+0.9)
    min_x = (np.min(conex)+0.9)
    #plt.axis([-5, 5, -5, 5])
    #plt.plot(conex, coney, 'ro')
    #plt.pause(0.000000000000000001)
    #plt.cla()
    track = ((abs(max_x))+(abs(min_x)))
    steering_angle = (((((track - abs(max_x))*2)/track)*2)-1)
    conex = []
    coney = []
    if str(steering_angle) == "nan":
        steering_angle = 0.0
    return ((steering_angle*-1)/1.7)
    
class Lidar(Node):

    def __init__(self):
        super().__init__('SlowLap')
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)
        self.subscription = self.create_subscription(LaserScan, '/ytu', self.lidarcallback, qos_profile=qos_policy)
        self.subscription
        self.publisher_ = self.create_publisher(ControlCommand, '/control_command', 10)
        self.cmd = ControlCommand()
        self.flag = 20

    def lidarcallback(self, lidar):
        self.rangescount = len(lidar.ranges)
        self.data = np.array(lidar.ranges)
        self.clean_data = inf_to_max(self.data)
        self.steering_data = plot_lidar_data(self.clean_data)
        print(self.steering_data)
        #print(len(lidar.data))
        #if (self.count != 0):
        #    self.cones_data = cones_data(self.clean_data)
        #    self.cones_opt = cones_opt(self.cones_data)
        #    self.cones_position = cones_position(self.cones_opt)
        #    self.steering_data = stering(self.cones_position)
        #print(self.steering_data)
        ##self.speed = (5.3 - (abs(self.steering_data)*3)) 
        if (self.flag == 0):
            self.flag = 20
            self.speed = 0.19
            self.cmd.throttle = self.speed
            self.cmd.steering = self.steering_data
            self.publisher_.publish(self.cmd)
        else:
            self.flag = self.flag-1
            self.speed = 0.0
            self.cmd.throttle = self.speed
            self.cmd.steering = self.steering_data
            self.publisher_.publish(self.cmd)

def main(args=None):
    rclpy.init(args=args)
    lidar = Lidar()
    rclpy.spin(lidar)
    lidar.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
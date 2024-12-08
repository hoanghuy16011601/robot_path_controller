#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import time

class Lidar():
    def __init__(self):
        self.Head_Distance = 1000
        self.Back_Distance = 1000
        self.Left_Distance = 1000
        self.Right_Distance = 1000

    def Get_Distance(self):
        return (self.Head_Distance ,self.Back_Distance ,self.Left_Distance ,self.Right_Distance)
    
    def Convert_To_Algorithm_Distance(Ranges):
        print("test")
        # Get Head
def scan_callback(msg:LaserScan):
    # Lấy giá trị khoảng cách của tia giữa
    head_index = len(msg.ranges) // 2
    left_index = len(msg.ranges) // 4
    right_index = len(msg.ranges)*3 //4
    back_index = 0
    rospy.loginfo("Head_Distance: %.2f m", msg.ranges[head_index])
    rospy.loginfo("Left_Distance: %.2f m", msg.ranges[left_index])
    rospy.loginfo("Back_Distance: %.2f m", msg.ranges[back_index])
    rospy.loginfo("Right_Distance: %.2f m", msg.ranges[right_index])
    # Lấy toàn bộ giá trị khoảng cách (nếu cần)
    time.sleep(0.5)

def main():
    # Khởi tạo ROS node
    rospy.init_node('scan_listener', anonymous=True)

    # Tạo subscriber lắng nghe topic /scan
    rospy.Subscriber('/scan', LaserScan, scan_callback)

    # Duy trì node hoạt động
    rospy.spin()

if __name__ == '__main__':
    main()

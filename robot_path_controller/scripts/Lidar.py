#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

def scan_callback(msg):
    # In thông tin cơ bản về quét laser
    rospy.loginfo("Số tia quét: %d", len(msg.ranges))
    rospy.loginfo("Góc quét: từ %.2f đến %.2f radian", msg.angle_min, msg.angle_max)
    rospy.loginfo("Góc giữa các tia: %.5f radian", msg.angle_increment)
    
    # Lấy giá trị khoảng cách của tia giữa
    mid_index = len(msg.ranges) // 2
    rospy.loginfo("Khoảng cách tia giữa: %.2f mét", msg.ranges[mid_index])
    
    # Lấy toàn bộ giá trị khoảng cách (nếu cần)
    ranges = msg.ranges

def main():
    # Khởi tạo ROS node
    rospy.init_node('scan_listener', anonymous=True)

    # Tạo subscriber lắng nghe topic /scan
    rospy.Subscriber('/scan', LaserScan, scan_callback)

    # Duy trì node hoạt động
    rospy.spin()

if __name__ == '__main__':
    main()

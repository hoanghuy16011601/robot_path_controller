import rospy
import tf
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import tf.transformations

rospy.init_node("Publish_Odom_Node")
pub_odom = rospy.Publisher("odom",Odometry,queue_size=50)

def handle_velociy_Robot(msg):
    print("Transform")
    Current_Time = rospy.Time.now()
    broadcaster = tf.TransformBroadcaster()
    broadcaster.sendTransform((0,0,0),(tf.transformations.quaternion_from_euler(0,0,0)),Current_Time,"base_link","odom")


    odom_msg = Odometry()
    odom_msg.header.stamp = Current_Time
    odom_msg.header.frame_id = "odom"

    odom_msg.pose.pose.position.x = 0
    odom_msg.pose.pose.position.y = 0
    odom_msg.pose.pose.position.z = 0
    odom_msg.pose.pose.orientation = 0

    odom_msg.child_frame_id = "base_link"
    odom_msg.twist.twist.linear.x = 0
    odom_msg.twist.twist.linear.y = 0
    odom_msg.twist.twist.linear.z = 0

    pub_odom.publish(odom_msg)

def odometry_subscribe():
    rospy.Subscriber("test",String, handle_velociy_Robot)
    rospy.spin()

if __name__ == '__main__':
    print("Transform odometry node is started")
    try:
        odometry_subscribe()
    except rospy.ROSInterruptException:
        pass



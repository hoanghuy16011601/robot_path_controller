from flood_fill.msg import Command
import rospy


def Command_Handler(Command):
    Command_Type = Command.type
    Command_Value = Command.value

    print(Command_Type)
    print(Command_Value)

if __name__ == "__main__":
    rospy.init_node('STM32_Controller',anonymous = True)
    rospy.Subscriber("Commander",Command, Command_Handler)
    rospy.spin()
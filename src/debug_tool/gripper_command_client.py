#!/usr/bin/env python
import rospy
from kortex_motion_planning.srv import SendGripperCommand, SendGripperCommandRequest

def call_kortex_service(gripper_position):
    rospy.wait_for_service('/kortex_gripper_command_service')
    try:
        kortex_service = rospy.ServiceProxy('/kortex_gripper_command_service', SendGripperCommand)
        request = SendGripperCommandRequest()
        request.gripper_position = gripper_position
        response = kortex_service(request)
        return response
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

if __name__ == '__main__':
    rospy.init_node('kortex_test_client')

    # Test position
    gripper_position = 0
    
    rospy.loginfo("Calling kortex_gripper_command_service")
    result = call_kortex_service(gripper_position)
    rospy.loginfo("Service response: %s" % result)

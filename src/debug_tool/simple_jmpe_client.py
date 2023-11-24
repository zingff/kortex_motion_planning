#!/usr/bin/env python
import rospy
from kortex_motion_planning.srv import KortexSimpleJmpe, KortexSimpleJmpeRequest

def call_kortex_service(joint_positions):
    rospy.wait_for_service('/kortex_simple_joint_motion_service')
    try:
        kortex_service = rospy.ServiceProxy('/kortex_simple_joint_motion_service', KortexSimpleJmpe)
        request = KortexSimpleJmpeRequest()
        request.target_positions.joint_positions = joint_positions
        response = kortex_service(request)
        return response
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

if __name__ == '__main__':
    rospy.init_node('kortex_test_client')

    # Test joint positions
    test_positions = [0.0212474, 6.01921-6.28, 3.15111, 4.13476-6.28, 0.0608379, 5.37321-6.28, 1.58046]

    rospy.loginfo("Calling kortex_simple_joint_motion_service")
    result = call_kortex_service(test_positions)
    rospy.loginfo("Service response: %s" % result)

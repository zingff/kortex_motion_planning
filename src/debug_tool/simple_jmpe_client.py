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
    # test_positions = [1.38348, 0.467969, 3.13848, -2.24192, -0.3143, 1.08215, 0.241945]
    # test_positions = [1.38236, 0.466606, 3.09348, 4.05094-6.28, 6.28229, 1.04642, 0.070713]  # bowl transfer initial 
    # test_positions = [1.3686876090750437, 0.28184642233856894, 3.144328097731239, 4.023280977312391-6.28, 6.009197207678883, 0.9603996509598605, 1.6590017452006982]  # door open initial
    test_positions = [5.69694, 0.576636, 3.46146, 4.25487-6.28, 0.676436, 6.13939-6.28, 0.537054 ]  # placement place
        
    rospy.loginfo("Calling kortex_simple_joint_motion_service")
    result = call_kortex_service(test_positions)
    rospy.loginfo("Service response: %s" % result)

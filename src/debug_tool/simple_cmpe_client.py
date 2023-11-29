#!/usr/bin/env python
import rospy
from kortex_motion_planning.srv import KortexSimpleJmpe, KortexSimpleJmpeRequest
from kortex_motion_planning.srv import KortexSimpleCmpe, KortexSimpleCmpeRequest
from geometry_msgs.msg import Pose

def call_kortex_service(cartesian_pose):
    rospy.wait_for_service('/kortex_simple_cartesian_motion_service')
    try:
        kortex_service = rospy.ServiceProxy('/kortex_simple_cartesian_motion_service', KortexSimpleCmpe)
        request = KortexSimpleCmpeRequest()
        request.target_pose = cartesian_pose
        response = kortex_service(request)
        return response
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

if __name__ == '__main__':
    rospy.init_node('kortex_cartesian_client')

    # Test joint positions
    test_positions = [0.0212474, 6.01921-6.28, 3.15111, 4.13476-6.28, 0.0608379, 5.37321-6.28, 1.58046]
    feeding_pose = Pose()
    feeding_pose.position.x = -0.15135
    feeding_pose.position.y = 0.235484
    feeding_pose.position.z = 0.557796
    feeding_pose.orientation.x = 0.3872724
    feeding_pose.orientation.y = -0.4914169
    feeding_pose.orientation.z = -0.604657
    feeding_pose.orientation.w = 0.4928685

    rospy.loginfo("Calling kortex_simple_joint_motion_service")
    result = call_kortex_service(feeding_pose)
    
    rospy.loginfo("Service response: %s" % result)

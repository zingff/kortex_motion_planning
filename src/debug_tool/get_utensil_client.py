#!/usr/bin/env python
import rospy
from kortex_motion_planning.srv import GetUtensil, GetUtensilRequest
from kortex_motion_planning.msg import JointPositions

def call_kortex_service(gripper_position):
    rospy.wait_for_service('kortex_get_utensil_service')
    try:
        kortex_service = rospy.ServiceProxy('kortex_get_utensil_service', GetUtensil)
        request = GetUtensilRequest()
        holder_positions = JointPositions()
        utensil_positions = JointPositions()
        holder_positions.joint_positions = [0.302989, -0.31447, 3.09643, -2.40708, 6.26457, -1.063090, 1.81463]
        utensil_positions.joint_positions = [0.206082, -0.305769, 2.64442, -2.3651, 6.09785, -1.06134, 1.36814]
        
        request.get_utensil_flag = True
        request.holder_positions.joint_positions = holder_positions.joint_positions
        request.utensil_positions.joint_positions = utensil_positions.joint_positions
        
        response = kortex_service(request)
        return response
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

if __name__ == '__main__':
    rospy.init_node('kortex_test_client')

    # Test position
    gripper_position = 1
    
    rospy.loginfo("Calling kortex_get_utensil_service")
    result = call_kortex_service(gripper_position)
    rospy.loginfo("Service response: %s" % result)

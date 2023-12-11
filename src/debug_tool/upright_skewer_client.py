#!/usr/bin/env python
import rospy
import kortex_motion_planning.srv

def call_kortex_service():
    rospy.wait_for_service('/kortex_upright_skewer_action_service')
    try:
        kortex_service = rospy.ServiceProxy('/kortex_upright_skewer_action_service', kortex_motion_planning.srv.UprightSkewerAction)
        request = kortex_motion_planning.srv.UprightSkewerActionRequest()
        request.skewer_action_flag = True
        response = kortex_service(request)
        response = True
        return response
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

if __name__ == '__main__':
    rospy.init_node('kortex_test_client')
     
    rospy.loginfo("Calling kortex_upright_skewer_action_service")
    result = call_kortex_service()
    rospy.loginfo("Service response: %s" % result)

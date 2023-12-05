#!/usr/bin/env python
import rospy
import door_open_task.srv

def call_kortex_service():
    rospy.wait_for_service('/feeding_task/reach_apriltag_service')
    try:
        kortex_service = rospy.ServiceProxy('/feeding_task/reach_apriltag_service', door_open_task.srv.ReachApriltag)
        request = door_open_task.srv.ReachApriltagRequest()
        request.reach_apriltag_flag = True
        response = kortex_service(request)
        # response = door_open_task.srv.DoorOpenResponse()
        response = True
        return response
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

if __name__ == '__main__':
    rospy.init_node('kortex_test_client')
     
    rospy.loginfo("Calling door_open_service")
    result = call_kortex_service()
    rospy.loginfo("Service response: %s" % result)

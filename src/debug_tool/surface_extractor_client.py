#!/usr/bin/env python
import rospy
from door_open_task.srv import DoorOpen, DoorOpenRequest
import door_open_task.srv
import anygrasp_generation.srv

def call_kortex_service():
    rospy.wait_for_service('/suitable_positions_finder')
    try:
        kortex_service = rospy.ServiceProxy('/suitable_positions_finder', anygrasp_generation.srv.SurfaceExtraction)
        request = anygrasp_generation.srv.SurfaceExtractionRequest()
        request.extract_surface_flag = True
        response = kortex_service(request)
        response = True
        return response
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

if __name__ == '__main__':
    rospy.init_node('kortex_test_client')
     
    rospy.loginfo("Calling surface_extractor_service")
    result = call_kortex_service()
    rospy.loginfo("Service response: %s" % result)

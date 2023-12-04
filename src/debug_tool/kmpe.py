#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import Pose
from kortex_motion_planning.srv import GenerateKortexMotionPlan, ExecuteMotionPlan, GenerateKortexMotionPlanRequest, ExecuteMotionPlanRequest
import time
import numpy as np

def normalize_quaternion(quaternion):
    return quaternion / np.linalg.norm(quaternion)

class KortexMotionWidget:
    def __init__(self):
        self.motion_planning_client = rospy.ServiceProxy('motion_planning_server', GenerateKortexMotionPlan)
        self.motion_execution_client = rospy.ServiceProxy('motion_execution_server', ExecuteMotionPlan)
        self.target_pose = Pose()

    def plan(self):
        target_pose = [0.419719-0.05, -0.128634, 0.118447, -0.559172, -0.512252, -0.467371, 0.454411]
        self.target_pose.position.x = target_pose[0]
        self.target_pose.position.y = target_pose[1]
        self.target_pose.position.z = target_pose[2]
        self.target_pose.orientation.x = target_pose[3]
        self.target_pose.orientation.y = target_pose[4]
        self.target_pose.orientation.z = target_pose[5]
        self.target_pose.orientation.w = target_pose[6]
        quaternion = (self.target_pose.orientation.x, self.target_pose.orientation.y,
                      self.target_pose.orientation.z, self.target_pose.orientation.w)
        normalized_quaternion = normalize_quaternion(quaternion)
        print(normalized_quaternion)

        self.target_pose.orientation.x = normalized_quaternion[0]
        self.target_pose.orientation.y = normalized_quaternion[1]
        self.target_pose.orientation.z = normalized_quaternion[2]
        self.target_pose.orientation.w = normalized_quaternion[3]

        rospy.loginfo("Normalized quaternion:")
        rospy.loginfo("x: %s", self.target_pose.orientation.x)
        rospy.loginfo("y: %s", self.target_pose.orientation.y)
        rospy.loginfo("z: %s", self.target_pose.orientation.z)
        rospy.loginfo("w: %s", self.target_pose.orientation.w)

        request = GenerateKortexMotionPlanRequest()
        request.target_pose = self.target_pose
        if True:
            rospy.loginfo("Succeeded to call kortex motion planning service!")
            self.motion_plan_response = self.motion_planning_client.call(request)
        else:
            rospy.logerr("Failed to call kortex motion planning service!")
            exit(1)

    def execute(self):
        request = ExecuteMotionPlanRequest()
        request.motion_plan = self.motion_plan_response.motion_plan
        if self.motion_execution_client.call(request):
            rospy.loginfo("Succeeded to call kortex motion execution service!")
        else:
            rospy.logerr("Failed to call kortex motion execution service!")
            exit(1)


def main():
    rospy.init_node('kortex_motion_widget')
    rospy.loginfo("Kortex planning motion widget initialized!")

    kmt = KortexMotionWidget()

    start_time = time.time()
    rospy.loginfo("Motion planning started at %s", time.ctime(start_time))

    kmt.plan()

    end_time = time.time()
    elapsed_time = end_time - start_time
    rospy.loginfo("Motion planning finished at %s", time.ctime(end_time))
    rospy.loginfo("Elapsed time for motion planning: %f seconds", elapsed_time)

    kmt.execute()


if __name__ == '__main__':
    main()

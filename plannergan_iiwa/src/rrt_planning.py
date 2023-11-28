#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from moveit2 import MoveGroupInterface
from geometry_msgs.msg import Pose

class RRTPlanner(Node):
    def __init__(self):
        super().__init__('rrt_planning')
        self.move_group = MoveGroupInterface("iiwa_arm", "robot_description")

    def plan_to_pose(self, target_pose):
        self.move_group.set_pose_target(target_pose)
        plan = self.move_group.plan()
        return plan

    def execute_plan(self, plan):
        self.move_group.execute(plan, wait=True)

    def create_pose(self, x, y, z, w):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = w
        return pose

def main(args=None):
    rclpy.init(args=args)
    rrt_planner = RRTPlanner()

    start_pose = rrt_planner.create_pose(0.4, 0.1, 0.4, 1)
    end_pose = rrt_planner.create_pose(0.6, -0.1, 0.6, 1)

    start_plan = rrt_planner.plan_to_pose(start_pose)
    rrt_planner.execute_plan(start_plan)

    end_plan = rrt_planner.plan_to_pose(end_pose)
    rrt_planner.execute_plan(end_plan)

    rclpy.spin(rrt_planner)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
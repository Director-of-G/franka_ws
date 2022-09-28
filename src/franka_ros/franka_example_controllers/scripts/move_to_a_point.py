#!/usr/bin/env python
# produced by yxj 20211203
# copied from move_to_start.py
import rospy
from moveit_commander import MoveGroupCommander
from actionlib_msgs.msg import GoalStatusArray

if __name__ == '__main__':
    rospy.init_node('move_to_start')
    rospy.wait_for_message('move_group/status', GoalStatusArray)
    commander = MoveGroupCommander('panda_arm')
    # commander.set_named_target('ready')
    current_pose = commander.get_current_pose(end_effector_link="panda_hand")
    print("current pose is",current_pose)
    goal_pose = current_pose
    goal_pose.pose.position.z += 0.05
    target_position_list = [goal_pose.pose.position.x,goal_pose.pose.position.y,goal_pose.pose.position.z]

    commander.set_position_target(target_position_list,end_effector_link="panda_hand")
    commander.go()
    # while(1):
    #     print(commander.get_current_pose(end_effector_link="panda_joint8"))

#!/usr/bin/python2

# ROS
import actionlib
import control_msgs.msg
import rospy
import trajectory_msgs.msg

# TU/e Manipulation
import tue_manipulation_msgs.msg

""" Test grasp precompute specifically for amigo (to avoid all kinds of dependencies)
"""

AC_JOINT = None
AC_GRASP = None 

# Create actionlib interfaces
ARM_JOINT_NAMES = ["shoulder_yaw_joint",  "shoulder_pitch_joint", "shoulder_roll_joint",
                   "elbow_pitch_joint", "elbow_roll_joint", "wrist_pitch_joint", 
                   "wrist_yaw_joint"]

def create_connections():
    """ Sets up the actionlib connections as global variables
    """
    global AC_JOINT
    global AC_GRASP
    AC_JOINT = actionlib.SimpleActionClient('/amigo/body/joint_trajectory_action',
                                                 control_msgs.msg.FollowJointTrajectoryAction)
    AC_GRASP = actionlib.SimpleActionClient("/amigo/left_arm/grasp_precompute",
                                        tue_manipulation_msgs.msg.GraspPrecomputeAction)
    rospy.sleep(0.5)  # Sleep to make sure ros has made its connections


def reset_arms_torso():
    """ Resets the arms and torso
    """
    torso_goal = control_msgs.msg.FollowJointTrajectoryGoal()
    torso_goal_point = trajectory_msgs.msg.JointTrajectoryPoint()
    torso_goal.trajectory.joint_names = ["torso_joint"]
    torso_goal_point.positions = [0.35]
    torso_goal.trajectory.points.append(torso_goal_point)
    rospy.sleep(0.01)
    rospy.loginfo("Sending torso goal")
    AC_JOINT.send_goal(torso_goal)

    for side in ["left", "right"]:
        arm_goal = control_msgs.msg.FollowJointTrajectoryGoal()
        arm_goal_point = trajectory_msgs.msg.JointTrajectoryPoint()
        arm_goal.trajectory.joint_names = [name + "_" + side for name in ARM_JOINT_NAMES]
        arm_goal_point.positions = [-0.1, -0.2, 0.2, 0.8, 0.0, 0.0, 0.0]
        arm_goal.trajectory.points.append(arm_goal_point)
        rospy.sleep(0.01)
        rospy.loginfo("Sending {} joint goal".format(side))
        AC_JOINT.send_goal(arm_goal)

    rospy.loginfo("Waiting for motions to finish")
    rospy.sleep(5.0)


def send_left_goal():
    """ Sends a goal to the left arm using the grasp precompute interface
    """
    goal = tue_manipulation_msgs.msg.GraspPrecomputeGoal()
    goal.goal.header.frame_id = "amigo/base_link"
    goal.goal.header.stamp = rospy.Time.now()
    goal.PERFORM_PRE_GRASP = True
    goal.FIRST_JOINT_POS_ONLY = False
    goal.goal.x = 0.5
    goal.goal.y = 0.2
    goal.goal.z = 0.8
    rospy.loginfo("Sending grasp precompute goal: {}, {}, {}".format(goal.goal.x, goal.goal.y, goal.goal.z))
    AC_GRASP.send_goal_and_wait(goal, rospy.Duration(10.0))


if __name__ == "__main__":

    # Init rosnode, create action clients
    rospy.init_node("grasp_precompute_tester")
    create_connections()

    # Reset arms and torso
    reset_arms_torso()

    # Send goal to left arm
    send_left_goal()

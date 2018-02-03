#ifndef GRASP_PRECOMPUTE_
#define GRASP_PRECOMPUTE_

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include <tue_manipulation_msgs/GraspPrecomputeAction.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <tf/transform_listener.h>

#include <moveit/move_group_interface/move_group_interface.h>

#include <memory>

class GraspPrecompute
{

public:

    //!
    //! \brief initialize Initialize component
    //! \return True if success, false otherwise
    //!
    bool initialize();

private:

    bool computeStraightLineTrajectory(const geometry_msgs::Pose &start_pose, const geometry_msgs::Pose &goal_pose, moveit_msgs::RobotTrajectory& cartesian_moveit_trajectory);

private:

    /** Cartesian goal Action server */
    std::shared_ptr<actionlib::SimpleActionServer<tue_manipulation_msgs::GraspPrecomputeAction>> as_;

    /** Cartesian goal callback function */
    void execute(const tue_manipulation_msgs::GraspPrecomputeGoalConstPtr& goal);

    /** TF listener */
    std::shared_ptr<tf::TransformListener> listener_;

    /** Root and tip link */
    std::string root_link_, tip_link_;

    /** Offset for pre-grasping in cartesian x-direction [m] */
    double pre_grasp_delta_;

    /** Step-size for yaw sampling [rad] */
    double yaw_sampling_step_;

    /** Maximum offset from desired yaw [rad] */
    double max_yaw_;

    /** MoveIt group */
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> moveit_group_;

    /** Map with joint limits */
    struct limits {
        double lower;
        double upper;
    };
    std::map<std::string, limits> joint_limits_;

};

#endif

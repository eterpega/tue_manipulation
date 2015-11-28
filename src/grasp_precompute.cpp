#include "tue/manipulation/grasp_precompute.h"

#include <ros/ros.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

const double EPS = 1e-6;

////////////////////////////////////////////////////////////////////////////////

GraspPrecompute::GraspPrecompute()
{
	ROS_INFO("Starting GraspPrecompute");

    /// TF listener
    listener_ = new tf::TransformListener();

    /// Nodehandles
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    /// Parameters
    std::string side;
    nh_private.param<std::string>("side", side, ""); //determine for which side this node operates
    if (side.empty()){
        ROS_ERROR("Missing parameter 'side'.");
        return;
    }

    nh_private.param<std::string>("root_link", root_link_, "");
    if (root_link_.empty()){
        ROS_ERROR("Missing parameter 'root_link'.");
        return;
    }

    nh_private.param<std::string>("tip_link", tip_link_, "");
    if (tip_link_.empty()){
        ROS_ERROR("Missing parameter 'tip_link'.");
        return;
    }

    nh_private.param("pre_grasp_delta", pre_grasp_delta_, 0.05);
    nh_private.param("max_yaw_delta", max_yaw_, 2.0);
    nh_private.param("yaw_sampling_step", yaw_sampling_step_, 0.2);

    /// MoveIt
    moveit::planning_interface::MoveGroup::Options options(side+"_arm", "/amigo/robot_description", nh);
    moveit_group_ = new moveit::planning_interface::MoveGroup(options);
    moveit_group_->setPoseReferenceFrame(root_link_);
    moveit_group_->setEndEffectorLink(tip_link_);

    /// Start Cartesian action server
    as_ = new actionlib::SimpleActionServer<tue_manipulation::GraspPrecomputeAction>(nh, "grasp_precompute", boost::bind(&GraspPrecompute::execute, this, _1), false);
    as_->start();

    /// Start joint action server
//    jas_ = new actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>(nh, "joint_trajectory", boost::bind(&GraspPrecompute::joint_execute, this, _1), false);
//    jas_->start();

}

////////////////////////////////////////////////////////////////////////////////

GraspPrecompute::~GraspPrecompute()
{
    delete as_;
    delete jas_;
    delete listener_;
    delete moveit_group_;
}

////////////////////////////////////////////////////////////////////////////////

void GraspPrecompute::execute(const tue_manipulation::GraspPrecomputeGoalConstPtr& goal)
{
    /// Initialize variables
    unsigned int num_grasp_points = 1;
    unsigned int pre_grasp_inbetween_sampling_steps = 20; // Hardcoded, we might not need this... //PRE_GRASP_INBETWEEN_SAMPLING_STEPS
    moveit::planning_interface::MoveGroup::Plan my_plan;

    /// Check for absolute or delta (and ambiqious goals)
    bool absolute_requested=false, delta_requested=false;
    if(fabs(goal->goal.x)>EPS || fabs(goal->goal.y)>EPS || fabs(goal->goal.z)>EPS || fabs(goal->goal.roll)>EPS || fabs(goal->goal.pitch)>EPS || fabs(goal->goal.yaw)>EPS)
        absolute_requested = true;
    if(fabs(goal->delta.x)>EPS || fabs(goal->delta.y)>EPS || fabs(goal->delta.z)>EPS || fabs(goal->delta.roll)>EPS || fabs(goal->delta.pitch)>EPS || fabs(goal->delta.yaw)>EPS)
        delta_requested = true;
    if(absolute_requested && delta_requested)
    {
        as_->setAborted();
        ROS_WARN("grasp_precompute_action: goal consists out of both absolute AND delta values. Choose only one!");
        return;
    }

    /// Create input variable which we will work with
    geometry_msgs::PoseStamped stamped_in;
    stamped_in.header = goal->goal.header;
    stamped_in.pose.position.x = goal->goal.x;
    stamped_in.pose.position.y = goal->goal.y;
    stamped_in.pose.position.z = goal->goal.z;
    stamped_in.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(goal->goal.roll, goal->goal.pitch, goal->goal.yaw);

    /// Check if a delta is requested
    if(delta_requested)
    {
        ROS_DEBUG("Delta requested of x=%f \t y=%f \t z=%f \t roll=%f \t pitch=%f \t yaw=%f",goal->delta.x,goal->delta.y,goal->delta.z,goal->delta.roll,goal->delta.pitch,goal->delta.yaw);
        // Create tmp variable
        tf::StampedTransform tmp;

        // Lookup the current vector of the desired TF to the grippoint
        if(listener_->waitForTransform(goal->delta.header.frame_id, tip_link_, goal->delta.header.stamp, ros::Duration(1.0)))
        {
            try
            {
                listener_->lookupTransform(goal->delta.header.frame_id, tip_link_, goal->delta.header.stamp, tmp);
            }
            catch (tf::TransformException ex)
            {
                as_->setAborted();
                ROS_ERROR("%s",ex.what());
                return;
            }
        } else
        {
            as_->setAborted();
            ROS_ERROR("grasp_precompute_action: listener__ could not find transform from gripper to %s:",goal->delta.header.frame_id.c_str());
            return;
        }

        // Print the transform
        ROS_DEBUG("tmp.x=%f \t tmp.y=%f \t tmp.z=%f tmp.X=%f \t tmp.Y=%f \t tmp.Z=%f \t tmp.W=%f",tmp.getOrigin().getX(),tmp.getOrigin().getY(),tmp.getOrigin().getZ(),tmp.getRotation().getX(),tmp.getRotation().getY(),tmp.getRotation().getZ(),tmp.getRotation().getW());

        // Add the delta values and overwrite input variable
        ROS_INFO("Child frame id=%s", tmp.frame_id_.c_str());
        stamped_in.header.frame_id  = tmp.frame_id_;
        stamped_in.header.stamp     = tmp.stamp_;
        stamped_in.pose.position.x  = tmp.getOrigin().getX() + goal->delta.x;
        stamped_in.pose.position.y  = tmp.getOrigin().getY() + goal->delta.y;
        stamped_in.pose.position.z  = tmp.getOrigin().getZ() + goal->delta.z;
        double roll, pitch, yaw;
        tmp.getBasis().getRPY(roll, pitch, yaw);
        stamped_in.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll + goal->delta.roll, pitch + goal->delta.pitch, yaw + goal->delta.yaw);
    }

    /// Check if a pre-grasp is required
    if (goal->PERFORM_PRE_GRASP)
    {
        num_grasp_points = pre_grasp_inbetween_sampling_steps + 2; // Inbetween sample points + Pre-grasp + Grasp point
    }

    /// Evaluate if grasp is feasible
    tf::Transform grasp_pose;
    tf::poseMsgToTF(stamped_in.pose,grasp_pose);

    tf::Transform new_grasp_pose;
    tf::Transform new_pre_grasp_pose;

    bool grasp_feasible = false;
    bool sampling_boundaries_reached = false;
    double yaw_delta = 0.0;
    int yaw_sampling_direction = 1;

    ROS_INFO("Starting sampling...");

    /// Try to determine a trajectory
    while(ros::ok() && !grasp_feasible && !sampling_boundaries_reached )
    {
        // Define new_grasp_pose
        ROS_INFO("Computing new grasp pose...");
        tf::Transform yaw_offset(tf::createQuaternionFromYaw(yaw_sampling_direction * yaw_delta),tf::Point(0,0,0));
        new_grasp_pose = grasp_pose * yaw_offset;

        ROS_INFO("Computing intermediate points...");
        std::vector<geometry_msgs::Pose> waypoints(num_grasp_points);
        for (int i = num_grasp_points - 1; i >= 0; --i) {

            //cout << i << " offset = " << -PRE_GRASP_DELTA*i/(PRE_GRASP_INBETWEEN_SAMPLING_STEPS+1.0) << endl;
            tf::Transform pre_grasp_offset(tf::Quaternion(0,0,0,1),tf::Point(-pre_grasp_delta_*i/(pre_grasp_inbetween_sampling_steps+1.0),0,0));
            new_pre_grasp_pose = new_grasp_pose * pre_grasp_offset;

            tf::poseTFToMsg(new_pre_grasp_pose, waypoints[i]);

        }

        // ToDo: reverse vector for clarity???

        /// Compute a plan to the first waypoint
        ros::Duration(0.05).sleep(); // Make sure the robot is at the robot state before setStartState is called
        moveit_group_->setStartStateToCurrentState();
        moveit_group_->setPoseTarget(waypoints[num_grasp_points-1]);
        moveit_group_->setGoalTolerance(0.01);
//        ROS_INFO("x: %f, y: %f, z: %f", waypoints[num_grasp_points-1].position.x, waypoints[num_grasp_points-1].position.y, waypoints[num_grasp_points-1].position.z);
        grasp_feasible = moveit_group_->plan(my_plan);
//        ROS_INFO("Grasp feasible: %i", grasp_feasible);

        /// If we have a pre-grasp vector, compute the rest of the path
        if (num_grasp_points > 1 && grasp_feasible)
        {
            moveit_msgs::RobotState start_state;
            start_state.joint_state.name = my_plan.trajectory_.joint_trajectory.joint_names;
            unsigned int size = my_plan.trajectory_.joint_trajectory.points.size();
            start_state.joint_state.position = my_plan.trajectory_.joint_trajectory.points[size-1].positions;
            moveit_group_->setStartState(start_state);

            moveit::planning_interface::MoveGroup::Plan my_second_plan;

            /*
            Currently, we have two approaches to make sure we follow the approach vector.
            * The 'Plan' approach, which simply calls group->plan, hence using an RRT. This seems to be fast
              but there is no guarantee for a straight path in Cartesian space
            * Explicitly computing a Cartesian path. Seems less robust (gives no results more often) but
              will always result in a straight path
            */

            // 'Plan' approach
//            group->setPoseTarget(waypoints[0]);
//            GRASP_FEASIBLE = group->plan(my_second_plan);

            // Alternative: 'computeCartesianPath' approach
            std::vector<geometry_msgs::Pose> wps(2);
            wps[0] = waypoints[num_grasp_points-1];
            wps[1] = waypoints[0];
            moveit_msgs::RobotTrajectory cartesian_moveit_trajectory;
            double res = moveit_group_->computeCartesianPath(wps, 0.01, 10.0, cartesian_moveit_trajectory, false);
            // Check if more than 90% of the trajectory has been computed
            if (res > 0.9)
            {
                grasp_feasible = true;
            } else{
                grasp_feasible = false;
            }

            /// Interpolate to get velocities
            // First to create a RobotTrajectory object
            robot_trajectory::RobotTrajectory rt(moveit_group_->getCurrentState()->getRobotModel(), moveit_group_->getName());

            // Second get a RobotTrajectory from trajectory
            robot_state::RobotState rs(moveit_group_->getCurrentState()->getRobotModel());
            for (unsigned int i = 0; i < cartesian_moveit_trajectory.joint_trajectory.joint_names.size(); i++){
                rs.setJointPositions(cartesian_moveit_trajectory.joint_trajectory.joint_names[i], &cartesian_moveit_trajectory.joint_trajectory.points[0].positions[i]);
            }
            rt.setRobotTrajectoryMsg(rs, cartesian_moveit_trajectory);

            // Third create a IterativeParabolicTimeParameterization object
            trajectory_processing::IterativeParabolicTimeParameterization iptp;

            // Fourth compute computeTimeStamps
            if (!iptp.computeTimeStamps(rt)){
                ROS_WARN("Failed to calculate velocities for cartesian path.");
                grasp_feasible = false;
            }

            // If still feasible and the entire trajectory is to be executed (FIRST_JOINT_POS_ONLY is false), append trajectory
            // ToDo: make nice!
            if (grasp_feasible && !goal->FIRST_JOINT_POS_ONLY)
            {
                rt.getRobotTrajectoryMsg(cartesian_moveit_trajectory);
                my_second_plan.trajectory_ = cartesian_moveit_trajectory;
                for (unsigned int i = 0; i < my_second_plan.trajectory_.joint_trajectory.points.size(); i++)
                {
                    my_second_plan.trajectory_.joint_trajectory.points[i].time_from_start += my_plan.trajectory_.joint_trajectory.points[size-1].time_from_start;
                    my_plan.trajectory_.joint_trajectory.points.push_back(my_second_plan.trajectory_.joint_trajectory.points[i]);
                }
            }
        }

        /// If grasp not feasible, resample yaw
        if (grasp_feasible != 1)
        {
            ROS_DEBUG("Not all grasp points feasible: resampling yaw");
            yaw_delta = yaw_delta + yaw_sampling_step_;
            yaw_sampling_direction = -1 * yaw_sampling_direction;

            if(yaw_delta > max_yaw_)
            {
                ROS_WARN("Sampling boundaries reached. No feasible sample found\n");
                sampling_boundaries_reached = true;
                as_->setAborted(); // ToDo: set failed
                return;

            }
        }
    }

    // ToDo: make nice

    /// Planning succeeded, so execute it!
    grasp_feasible = moveit_group_->execute(my_plan);

    if (grasp_feasible)
    {
        ROS_INFO("Arm motion succeeded");
        as_->setSucceeded();
    } else {
        ROS_INFO("Arm motion failed");
        as_->setAborted(); // ToDo: set failed
    }

}

////////////////////////////////////////////////////////////////////////////////

void GraspPrecompute::joint_execute(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal)
{

    /// Check goal
    // ToDo

    /// Initialize variables
    moveit::planning_interface::MoveGroup::Plan my_plan;

    /// Initialize MoveIt
    moveit_group_->setStartStateToCurrentState();
    // goal->trajectory.joint_names
    // goal->trajectory.points.positions

    /// Plan the first point
    std::map<std::string, double> trajectory_point;
    for (unsigned int i = 0; i < goal->trajectory.joint_names.size(); ++i)
    {
        trajectory_point[ goal->trajectory.joint_names[i] ] = goal->trajectory.points[0].positions[i];
    }
    moveit_group_->setJointValueTarget(trajectory_point);
    bool feasible = moveit_group_->plan(my_plan);

    /// Plan the rest of the points
    // ToDo: plan the rest of the points...
//    if (goal->trajectory.points.size() > 1)
//    {
//        // Set robot start state

//        // Set target

//        // Plan

//        // Append
//    }

    if (!feasible)
    {
        ROS_WARN("Joint goal not feasible, aborting");
        jas_->setAborted();
        return;
    }

    /// Execute
    bool succeeded = moveit_group_->execute(my_plan);
    if (feasible)
    {
        ROS_INFO("Arm motion succeeded");
        jas_->setSucceeded();
    } else {
        ROS_INFO("Arm motion failed");
        jas_->setAborted(); // ToDo: set failed
    }

}

////////////////////////////////////////////////////////////////////////////////
#include "tue/manipulation/grasp_precompute.h"

#include <ros/ros.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <moveit/robot_model/joint_model.h>

const double EPS = 1e-6;

////////////////////////////////////////////////////////////////////////////////

bool GraspPrecompute::initialize()
{
	ROS_INFO("Starting GraspPrecompute");

    /// TF listener
    listener_ = std::shared_ptr<tf::TransformListener>(new tf::TransformListener());

    /// Nodehandles
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    /// Parameters
    std::string side;
    nh_private.param<std::string>("side", side, ""); //determine for which side this node operates
    if (side.empty()){
        ROS_ERROR("Missing parameter 'side'.");
        return false;
    }

    nh_private.param<std::string>("root_link", root_link_, "");
    if (root_link_.empty()){
        ROS_ERROR("Missing parameter 'root_link'.");
        return false;
    }

    nh_private.param<std::string>("tip_link", tip_link_, "");
    if (tip_link_.empty()){
        ROS_ERROR("Missing parameter 'tip_link'.");
        return false;
    }

    nh_private.param("pre_grasp_delta", pre_grasp_delta_, 0.05);
    nh_private.param("max_yaw_delta", max_yaw_, 2.0);
    nh_private.param("yaw_sampling_step", yaw_sampling_step_, 0.2);

    /// MoveIt
    moveit::planning_interface::MoveGroupInterface::Options options(side+"_arm", "/amigo/robot_description", nh);
    moveit_group_ = std::shared_ptr<moveit::planning_interface::MoveGroupInterface>(
          new moveit::planning_interface::MoveGroupInterface(options));
    moveit_group_->setPoseReferenceFrame(root_link_);
    moveit_group_->setEndEffectorLink(tip_link_);

    // try to fetch the robot state
    robot_state::RobotModelConstPtr robot_model = 0;
    while (!robot_model)
    {
      ROS_INFO_DELAYED_THROTTLE(1, "Waiting for robot model for group '%s' and description '%s'..",
                                options.group_name_.c_str(), options.robot_description_.c_str());
      robot_state::RobotStateConstPtr state = moveit_group_->getCurrentState();
      if (state)
      {
        robot_model = state->getRobotModel();
      }
      ros::Duration(0.1).sleep();

      if (!ros::ok())
      {
        return false;
      }
    }

    /// Read the joint limits
    const std::vector<const moveit::core::JointModel*> joint_models = robot_model->getJointModels();
    for (std::vector<const moveit::core::JointModel*>::const_iterator it = joint_models.begin(); it != joint_models.end(); ++it)
    {
        const std::string name = (*it)->getName();
        const std::vector<moveit::core::VariableBounds> bounds = (*it)->getVariableBounds();
        if (bounds.size() > 0)
        {
            limits climits;
            climits.lower = bounds[0].min_position_;
            climits.upper = bounds[0].max_position_;
//            ROS_INFO("Joint %s: lower: %f, upper: %f", name.c_str(), lower, upper);
            joint_limits_[name] = climits;
        }
    }

    /// Start Cartesian action server
    as_ = std::shared_ptr<actionlib::SimpleActionServer<tue_manipulation_msgs::GraspPrecomputeAction>>(
          new actionlib::SimpleActionServer<tue_manipulation_msgs::GraspPrecomputeAction>(nh, "grasp_precompute",
                    boost::bind(&GraspPrecompute::execute, this, _1), false));
    as_->start();

    ROS_INFO("Grasp precompute action server started");

    return true;
}

////////////////////////////////////////////////////////////////////////////////

void GraspPrecompute::execute(const tue_manipulation_msgs::GraspPrecomputeGoalConstPtr& goal)
{
    /// Initialize variables
    unsigned int num_grasp_points = 1;
    unsigned int pre_grasp_inbetween_sampling_steps = 20; // Hardcoded, we might not need this... //PRE_GRASP_INBETWEEN_SAMPLING_STEPS
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    robot_state::RobotState kinematic_state(moveit_group_->getRobotModel());
    const moveit::core::JointModelGroup* joint_model_group = kinematic_state.getJointModelGroup(moveit_group_->getName());

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
    geometry_msgs::PoseStamped goal_pose_stamped;
    goal_pose_stamped.header = goal->goal.header;
    goal_pose_stamped.pose.position.x = goal->goal.x;
    goal_pose_stamped.pose.position.y = goal->goal.y;
    goal_pose_stamped.pose.position.z = goal->goal.z;
    goal_pose_stamped.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(goal->goal.roll, goal->goal.pitch, goal->goal.yaw);

    /// Check if a delta is requested
    if(delta_requested)
    {
        updateDeltaGoal(goal, goal_pose_stamped);
    }

    /// Check if a pre-grasp is required
    if (goal->PERFORM_PRE_GRASP)
    {
        num_grasp_points = pre_grasp_inbetween_sampling_steps + 2; // Inbetween sample points + Pre-grasp + Grasp point
    }

    /// Evaluate if grasp is feasible
    tf::Transform grasp_pose;
    tf::poseMsgToTF(goal_pose_stamped.pose, grasp_pose);

    bool grasp_feasible = false;
    bool sampling_boundaries_reached = false;
    double yaw_delta = 0.0;
    int yaw_sampling_direction = 1;

    ROS_INFO("Starting sampling...");

    ros::Time start_stamp = ros::Time::now();

    /// Try to determine a trajectory
    while(ros::ok() && !grasp_feasible && !sampling_boundaries_reached )
    {

        /// Check if a cancel has been requested
        if (as_->isPreemptRequested()) {
            ROS_INFO("Goal cancelled");
            as_->setPreempted();
            return;
        }

        // Compute new approach vector
        std::vector<geometry_msgs::Pose> waypoints(num_grasp_points);
        precomputeApproach(grasp_pose, yaw_sampling_direction * yaw_delta, num_grasp_points, pre_grasp_inbetween_sampling_steps, waypoints);
        
        /// Test: put the first point 5 cm higher
        if (waypoints.size() > 1)
        {
            waypoints[num_grasp_points-1].position.z += 0.05;
        }

        /// Sanity check if it is feasible at all
        grasp_feasible = kinematic_state.setFromIK(joint_model_group, waypoints[num_grasp_points-1], 10, 0.1);
        ROS_DEBUG("FOUND IK: %d", grasp_feasible);
        grasp_feasible = true;  // This already was the default

        /// Compute a plan to the first waypoint
        if (grasp_feasible)
        {

            ros::Duration(0.1).sleep(); // Make sure the robot is at the robot state before setStartState is called
            moveit_group_->setStartStateToCurrentState();

            // 'Normal' approach
//            grasp_feasible = planTrajectory(waypoints[num_grasp_points-1], my_plan);

            // Alternative: use straight line interpolator here as wel...
            // ToDo: add try/catch
            // ToDo: aren't there conversion methods to do this bookkeeping?

            tf::StampedTransform start_pose_tf;
            listener_->lookupTransform(root_link_, tip_link_, ros::Time(0), start_pose_tf);
            tf::Vector3 position = start_pose_tf.getOrigin();
            double x = position.getX();
            double y = position.getY();
            double z = position.getZ();
            ROS_INFO("Current [x, y, z]: [%.2f, %.2f, %.2f]", x, y, z);
            geometry_msgs::Pose start_pose;
            start_pose.position.x = start_pose_tf.getOrigin().getX();
            start_pose.position.y = start_pose_tf.getOrigin().getY();
            start_pose.position.z = start_pose_tf.getOrigin().getZ();
            tf::Quaternion quat = start_pose_tf.getRotation();
            start_pose.orientation.x = quat.getX();
            start_pose.orientation.y = quat.getY();
            start_pose.orientation.z = quat.getZ();
            start_pose.orientation.w = quat.getW();
            grasp_feasible = computeStraightLineTrajectory(start_pose, waypoints[num_grasp_points-1], my_plan);

            ///// TEST STUFF
//            as_->setAborted(); // ToDo: set failed
//            return;
            /////

        }

        /// If we have a pre-grasp vector, compute the rest of the path
        if (num_grasp_points > 1 && grasp_feasible)
        {
            ros::Time approach_start_stamp = ros::Time::now();

            // Update start state
            setStartStateToTrajectoryEndPoint(my_plan.trajectory_, moveit_group_);

            moveit::planning_interface::MoveGroupInterface::Plan my_second_plan;

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
            grasp_feasible = computeStraightLineTrajectory(waypoints[num_grasp_points-1], waypoints[0], my_second_plan);

            // If still feasible and the entire trajectory is to be executed (FIRST_JOINT_POS_ONLY is false), append trajectory
            if (grasp_feasible && !goal->FIRST_JOINT_POS_ONLY)
            {
                appendTrajectories(my_plan, my_second_plan);
            }

            ros::Time approach_end_stamp = ros::Time::now();
            ROS_INFO("Approach took %.2f seconds", (approach_end_stamp - approach_start_stamp).toSec());
        }

        /// If grasp not feasible, resample yaw
        if (!grasp_feasible)
        {
            ROS_DEBUG("Not all grasp points feasible: resampling yaw");
            if (yaw_sampling_direction > 0)
            {
                yaw_delta += yaw_sampling_step_;
            }
            yaw_sampling_direction *= -1;

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

    /// Double check joint limits (MoveIt might provide trajectories that are just outside the bounds)
    applyJointLimits(my_plan);

    ros::Time planning_stamp = ros::Time::now();
    ROS_DEBUG("Planning took %.2f seconds", (planning_stamp - start_stamp).toSec());

    /// Planning succeeded, so execute it!
    bool motion_succeeded = (moveit_group_->execute(my_plan) == moveit_msgs::MoveItErrorCodes::SUCCESS);

    ros::Time execution_stamp = ros::Time::now();
    ROS_DEBUG("Execution took %.2f seconds", (execution_stamp - planning_stamp).toSec());
    ROS_INFO("Planning: %.2f seconds, execution: %.2f seconds", (planning_stamp - start_stamp).toSec(), (execution_stamp - planning_stamp).toSec());

    std::cout << "Planning: " << (planning_stamp - start_stamp).toSec() << "seconds, execution: " << (execution_stamp - planning_stamp).toSec() << "seconds" << std::endl;

    if (motion_succeeded)
    {
        ROS_INFO("Arm motion succeeded");
        as_->setSucceeded();
    } else {
        ROS_INFO("Arm motion failed");
        as_->setAborted(); // ToDo: set failed
    }

}

////////////////////////////////////////////////////////////////////////////////

void GraspPrecompute::precomputeApproach(tf::Transform& grasp_pose, double yaw_angle,
                                         int num_grasp_points, unsigned int pre_grasp_inbetween_sampling_steps,
                                         std::vector<geometry_msgs::Pose>& waypoints)
{
    // Define new_grasp_pose
    ROS_INFO("Computing new grasp pose...");
    tf::Transform yaw_offset(tf::createQuaternionFromYaw(yaw_angle), tf::Point(0, 0, 0));
    tf::Transform new_grasp_pose = grasp_pose * yaw_offset;

    ROS_INFO("Computing intermediate points...");
//    std::vector<geometry_msgs::Pose> waypoints(num_grasp_points);
    for (int i = num_grasp_points - 1; i >= 0; --i) {

        //cout << i << " offset = " << -PRE_GRASP_DELTA*i/(PRE_GRASP_INBETWEEN_SAMPLING_STEPS+1.0) << endl;
        tf::Transform pre_grasp_offset(tf::Quaternion(0, 0, 0, 1),
                                       tf::Point(-pre_grasp_delta_ * i / (pre_grasp_inbetween_sampling_steps + 1.0), 0, 0));
        tf::Transform new_pre_grasp_pose = new_grasp_pose * pre_grasp_offset;

        tf::poseTFToMsg(new_pre_grasp_pose, waypoints[i]);

    }

}

////////////////////////////////////////////////////////////////////////////////

bool GraspPrecompute::planTrajectory(const geometry_msgs::Pose &goal_pose, moveit::planning_interface::MoveGroupInterface::Plan& plan)
{
    moveit_group_->setPoseTarget(goal_pose);
    moveit_group_->setGoalPositionTolerance(0.01);
    moveit_group_->setGoalOrientationTolerance(0.1);
    ROS_INFO("x: %f, y: %f, z: %f", goal_pose.position.x, goal_pose.position.y, goal_pose.position.z);
    return (moveit_group_->plan(plan) == moveit_msgs::MoveItErrorCodes::SUCCESS);

}

////////////////////////////////////////////////////////////////////////////////

bool GraspPrecompute::computeStraightLineTrajectory(const geometry_msgs::Pose& start_pose,
                                                    const geometry_msgs::Pose& goal_pose,
                                                    moveit::planning_interface::MoveGroupInterface::Plan &plan)
{
    // ToDo: update C++ vector
    std::vector<geometry_msgs::Pose> wps(2);
    wps[0] = start_pose;
    wps[1] = goal_pose;

    moveit_msgs::RobotTrajectory cartesian_moveit_trajectory;
    double res = moveit_group_->computeCartesianPath(wps, 0.01, 10.0, cartesian_moveit_trajectory, false);
    // Check if more than 90% of the trajectory has been computed
    if (res < 0.9)
    {
        ROS_WARN("Failed to compute Cartesian path");
        return false;
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
        return false;
    } else {
        rt.getRobotTrajectoryMsg(cartesian_moveit_trajectory);
        plan.trajectory_ = cartesian_moveit_trajectory;
        return true;
    }

}

////////////////////////////////////////////////////////////////////////////////

void GraspPrecompute::setStartStateToTrajectoryEndPoint(const moveit_msgs::RobotTrajectory &trajectory,
                                                        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> moveit_group)
{
    moveit_msgs::RobotState start_state;
    start_state.joint_state.name = trajectory.joint_trajectory.joint_names;
    unsigned int size = trajectory.joint_trajectory.points.size();
    start_state.joint_state.position = trajectory.joint_trajectory.points[size-1].positions;
    moveit_group->setStartState(start_state);
}

////////////////////////////////////////////////////////////////////////////////

void GraspPrecompute::appendTrajectories(moveit::planning_interface::MoveGroupInterface::Plan& plan1, moveit::planning_interface::MoveGroupInterface::Plan& plan2) const
{
    unsigned int size1 = plan1.trajectory_.joint_trajectory.points.size();
    ros::Duration time_offset = plan1.trajectory_.joint_trajectory.points[size1-1].time_from_start;
    for (unsigned int i = 1; i < plan2.trajectory_.joint_trajectory.points.size(); i++)
    {
        plan2.trajectory_.joint_trajectory.points[i].time_from_start += time_offset;
        plan1.trajectory_.joint_trajectory.points.push_back(plan2.trajectory_.joint_trajectory.points[i]);
    }
}

////////////////////////////////////////////////////////////////////////////////

void GraspPrecompute::updateDeltaGoal(const tue_manipulation_msgs::GraspPrecomputeGoalConstPtr& goal, geometry_msgs::PoseStamped& goal_stamped_pose)
{
    ROS_DEBUG("Delta requested of x=%f \t y=%f \t z=%f \t roll=%f \t pitch=%f \t yaw=%f",
              goal->delta.x, goal->delta.y, goal->delta.z,
              goal->delta.roll, goal->delta.pitch, goal->delta.yaw);
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
    goal_stamped_pose.header.frame_id  = tmp.frame_id_;
    goal_stamped_pose.header.stamp     = tmp.stamp_;
    goal_stamped_pose.pose.position.x  = tmp.getOrigin().getX() + goal->delta.x;
    goal_stamped_pose.pose.position.y  = tmp.getOrigin().getY() + goal->delta.y;
    goal_stamped_pose.pose.position.z  = tmp.getOrigin().getZ() + goal->delta.z;
    double roll, pitch, yaw;
    tmp.getBasis().getRPY(roll, pitch, yaw);
    goal_stamped_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll + goal->delta.roll, pitch + goal->delta.pitch, yaw + goal->delta.yaw);
}

////////////////////////////////////////////////////////////////////////////////

void GraspPrecompute::applyJointLimits(moveit::planning_interface::MoveGroupInterface::Plan &plan)
{
    // Loop over all joints
    for (unsigned int i = 0; i < plan.trajectory_.joint_trajectory.joint_names.size(); i++)
    {
        std::string joint_name = plan.trajectory_.joint_trajectory.joint_names[i];
        limits climits = joint_limits_[joint_name];

        // Loop over all trajectory points
        for (unsigned int j = 0; j < plan.trajectory_.joint_trajectory.points.size(); j++)
        {
            plan.trajectory_.joint_trajectory.points[j].positions[i] = std::min(std::max(climits.lower,
                                                                                         plan.trajectory_.joint_trajectory.points[j].positions[i]),
                                                                                climits.upper);
        }
    }
}

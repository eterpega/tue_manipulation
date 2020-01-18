#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <tue_manipulation_msgs/GripperCommandAction.h>
//#include <tue_msgs/GripperMeasurement.h>

//std::string gripper_measurement_topic_;

actionlib::SimpleActionServer<tue_manipulation_msgs::GripperCommandAction>* as_;

ros::Publisher gripper_pub_;

tue_msgs::GripperCommand gripper_command_;
tue_msgs::GripperMeasurement gripper_meas_;

bool new_measurement_available = false;


void gripperMeasurementCb(const tue_msgs::GripperMeasurementConstPtr& meas) {
    if (as_->isActive()) {
        gripper_meas_ = *meas;
        new_measurement_available = true;
    }
}

void executeCB(const tue_manipulation_msgs::GripperCommandGoalConstPtr& gripper_goal) {
    ros::NodeHandle n;

    //std::stringstream meas_topic;
    //meas_topic << "/arm_" << side_ << "_controller/gripper_measurement";
    ros::Subscriber gripper_sub = n.subscribe("measurements", 1, &gripperMeasurementCb);

    gripper_command_ = gripper_goal->command;
    gripper_pub_.publish(gripper_command_);

    //gripper_executing_ = true;
    ros::Rate r(10);
    new_measurement_available = false;
    while(n.ok() && as_->isActive()) {

        if (as_->isNewGoalAvailable()) {
            gripper_command_ = as_->acceptNewGoal()->command;
            ROS_DEBUG("New goal received.");
            gripper_pub_.publish(gripper_command_);
        }

        // check that preempt has not been requested by the client
        if (as_->isPreemptRequested() || !ros::ok()) {
            // set the action state to preempted
            as_->setPreempted();
            break;
        }

        if(new_measurement_available)
        {
            if (gripper_meas_.end_position_reached) {
                ROS_DEBUG("Gripper server reports: End position reached");
                tue_manipulation_msgs::GripperCommandResult result;
                result.measurement = gripper_meas_;
                as_->setSucceeded(result, "End position reached.");
                break;
            } else if (gripper_meas_.max_force_reached) {
                ROS_DEBUG("Gripper server reports: Max torque reached");
                tue_manipulation_msgs::GripperommandResult result;
                result.measurement = gripper_meas_;
                as_->setSucceeded(result, "Max torque reached.");
                break;
            } else {
                tue_manipulation_msgs::GripperCommandFeedback feedback;
                feedback.measurement = gripper_meas_;
                as_->publishFeedback(feedback);
            }
            new_measurement_available = false;
        }
        r.sleep();
    }

    gripper_sub.shutdown();
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "gripper_server");
    ros::NodeHandle nh;

    //std::stringstream as_name;
    //as_name << "/gripper_server_" << side_;
    as_ = new actionlib::SimpleActionServer<tue_manipulation_msgs::GripperCommandAction>(nh, "action", &executeCB, false);

    //std::stringstream cmd_topic;
    //cmd_topic << "/arm_" << side_ << "_controller/gripper_command";
    gripper_pub_ = nh.advertise<tue_msgs::GripperCommand>("references", 100);

    as_->start();

    ROS_INFO("Gripper server is active and spinning...");

    ros::spin();

    return 0;
}


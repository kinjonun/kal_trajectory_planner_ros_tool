#include "trajectory_planner.hpp"

#include <stdexcept>

#include <Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.h>

#include "conversions.hpp"

namespace kal_trajectory_planner_ros_tool {

/**
 * Initialization
 */
TrajectoryPlannerNode::TrajectoryPlannerNode(const ros::NodeHandle& nhPrivate)
        : interface_{nhPrivate}, reconfigureServer_{nhPrivate} {

    interface_.fromParamServer();
    trajectoryPlanner_.setPolynomialDegree(interface_.polynomial_degree);


    reconfigureServer_.setCallback(boost::bind(&TrajectoryPlannerNode::reconfigureCallback, this, _1, _2));
    interface_.path_subscriber->registerCallback(&TrajectoryPlannerNode::pathCallback, this);
    interface_.cone_path_subscriber->registerCallback(&TrajectoryPlannerNode::cone_pathCallback, this);

    trajectoryGenerationTimer_ =
        nhPrivate.createTimer(ros::Rate(interface_.timer_rate), &TrajectoryPlannerNode::trajectoryCallback, this);

    cone_trajectoryGenerationTimer_ =
        nhPrivate.createTimer(ros::Rate(interface_.timer_rate), &TrajectoryPlannerNode::cone_trajectoryCallback, this);

    interface_.showNodeInfo();
    interface_.logDebug("Node initialized.");
}

void TrajectoryPlannerNode::pathCallback(const nav_msgs::Path::ConstPtr& pathMsg) {
    Path path = conversions::pathMsgToPath(pathMsg);
    trajectoryPlanner_.setPath(path);
    pathInitialized_ = true;
}


void TrajectoryPlannerNode::cone_pathCallback(const nav_msgs::Path::ConstPtr& pathMsg) {
    Path path = conversions::pathMsgToPath(pathMsg);
    cone_trajectoryPlanner_.setPath(path);
    cone_pathInitialized_ = true;
}

void TrajectoryPlannerNode::trajectoryCallback(const ros::TimerEvent& /*timer*/) {
    // Make sure path is initialized, otherwise trajectory planner will crash
    if (!pathInitialized_) {
        interface_.logWarn("Cannot compute trajectory because no path has been set yet.");
        return;
    }

    // Find vehicle pose
    Pose vehiclePose;
    if (!currentVehiclePose(vehiclePose)) {
        interface_.logWarn("Cannot compute trajectory because vehicle pose was not found in tf tree.");
        return;
    }
    // Compute trajectory
    Trajectory trajectory = trajectoryPlanner_.computeTrajectory(
        vehiclePose, interface_.desired_speed, interface_.trajectory_length, interface_.trajectory_num_of_points);

    // Publish trajectory
    interface_.trajectory_publisher.publish(conversions::trajectoryToPathMsg(trajectory, interface_.map_frame));
}



void TrajectoryPlannerNode::cone_trajectoryCallback(const ros::TimerEvent& /*timer*/) {
    // Make sure path is initialized, otherwise trajectory planner will crash
    if (!cone_pathInitialized_) {
        interface_.logWarn("Cannot compute trajectory because no path has been set yet.");
        return;
    }

    // Find vehicle pose
    Pose vehiclePose;
    if (!cone_currentVehiclePose(vehiclePose)) {
        interface_.logWarn("Cannot compute trajectory because vehicle pose was not found in tf tree.");
        return;
    }
    // Compute trajectory
    Trajectory trajectory = cone_trajectoryPlanner_.computeTrajectory(
        vehiclePose, interface_.desired_speed, interface_.trajectory_length, interface_.trajectory_num_of_points);

    // Publish trajectory
    interface_.cone_trajectory_publisher.publish(conversions::trajectoryToPathMsg(trajectory, interface_.map_frame));
}



void TrajectoryPlannerNode::reconfigureCallback(const Interface::Config& config, uint32_t /*level*/) {
    interface_.fromConfig(config);
    trajectoryPlanner_.setPolynomialDegree(interface_.polynomial_degree);
}

bool TrajectoryPlannerNode::currentVehiclePose(Pose& pose) const {

    // Find vehicle position in tf tree
    geometry_msgs::TransformStamped vehiclePoseRos;
    Eigen::Isometry3d vehiclePose3d;
    try {
        vehiclePoseRos = tfBuffer_.lookupTransform(interface_.map_frame, interface_.vehicle_frame, ros::Time(0));
    } catch (const tf2::TransformException& e) {
        return false;
    }

    // Transform to Eigen type
    vehiclePose3d = tf2::transformToEigen(vehiclePoseRos);

    // Convert to two dimensions
    pose = conversions::isometry2dFromIsometry3d(vehiclePose3d);

    return true;
}

bool TrajectoryPlannerNode::cone_currentVehiclePose(Pose& pose) const {

    // Find vehicle position in tf tree
    geometry_msgs::TransformStamped vehiclePoseRos;
    Eigen::Isometry3d vehiclePose3d;
    try {
        vehiclePoseRos = cone_tfBuffer_.lookupTransform(interface_.map_frame, interface_.vehicle_frame, ros::Time(0));
    } catch (const tf2::TransformException& e) {
        return false;
    }

    // Transform to Eigen type
    vehiclePose3d = tf2::transformToEigen(vehiclePoseRos);

    // Convert to two dimensions
    pose = conversions::isometry2dFromIsometry3d(vehiclePose3d);

    return true;
}

} // namespace kal_trajectory_planner_ros_tool

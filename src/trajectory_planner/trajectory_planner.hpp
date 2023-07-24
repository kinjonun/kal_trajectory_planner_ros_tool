#pragma once

#include <dynamic_reconfigure/server.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#include <kal_trajectory_planner/trajectory_planner.hpp>
#include <kal_trajectory_planner/types.hpp>

#include "kal_trajectory_planner_ros_tool/TrajectoryPlannerInterface.h"
namespace kal_trajectory_planner_ros_tool {

using namespace kal_trajectory_planner;

class TrajectoryPlannerNode {
public:
    using Interface = TrajectoryPlannerInterface;

    explicit TrajectoryPlannerNode(const ros::NodeHandle& nhPrivate);

private:

    void pathCallback(const nav_msgs::Path::ConstPtr& pathMsg);
    void cone_pathCallback(const nav_msgs::Path::ConstPtr& pathMsg);


    void trajectoryCallback(const ros::TimerEvent&);
    void cone_trajectoryCallback(const ros::TimerEvent&);

    void reconfigureCallback(const Interface::Config& config, uint32_t /*level*/);

    bool currentVehiclePose(Pose& pose) const;
    bool cone_currentVehiclePose(Pose& pose) const;

    TrajectoryPlanner trajectoryPlanner_;
    TrajectoryPlanner cone_trajectoryPlanner_;

    ros::Timer trajectoryGenerationTimer_;
    ros::Timer cone_trajectoryGenerationTimer_;

    tf2_ros::Buffer tfBuffer_;
    tf2_ros::Buffer cone_tfBuffer_;

    tf2_ros::TransformListener tfListener_{tfBuffer_};
    tf2_ros::TransformListener cone_tfListener_{cone_tfBuffer_};

    Interface interface_;
    dynamic_reconfigure::Server<Interface::Config> reconfigureServer_;

    bool pathInitialized_{false};
    bool cone_pathInitialized_{false};
};
} // namespace kal_trajectory_planner_ros_tool

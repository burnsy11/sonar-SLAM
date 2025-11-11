#pragma once
#include <bruce_msgs/msg/isam2_update.hpp>

#include <gtsam/nonlinear/ISAM2.h>
#include <memory>

namespace bruce_msgs
{
using ISAM2UpdatePtr = std::shared_ptr<bruce_msgs::msg::ISAM2Update>;

ISAM2UpdatePtr toMsg(const gtsam::ISAM2 &isam2, const gtsam::NonlinearFactorGraph &graph = gtsam::NonlinearFactorGraph(),
                    const gtsam::Values &values = gtsam::Values());
void fromMsg(const bruce_msgs::msg::ISAM2Update &slam_update_msg, gtsam::ISAM2 &isam2);
void fromMsg(const bruce_msgs::msg::ISAM2Update &slam_update_msg, gtsam::NonlinearFactorGraph &graph, gtsam::Values &values);
void fromMsg(const bruce_msgs::msg::ISAM2Update &slam_update_msg, gtsam::ISAM2 &isam2, gtsam::NonlinearFactorGraph &graph,
             gtsam::Values &values);

}  // namespace bruce_msgs
/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_NODE_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_NODE_H

#include <map>
#include <memory>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "cartographer/common/fixed_ratio_sampler.h"
#include "cartographer/common/mutex.h"
#include "cartographer/mapping/map_builder_interface.h"
#include "cartographer/mapping/pose_extrapolator.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/io/submap_painter.h"

#include "map_builder_bridge.h"
#include "node_constants.h"
#include "node_options.h"
#include "trajectory_options.h"


#include "cartographer_ros_msgs/SensorTopics.h"
#include "cartographer_ros_msgs/StatusResponse.h"

#include "imudata.h"
#include "laserdata.h"
#include "Submapdata.h"

/*
#include "cartographer_ros_msgs/FinishTrajectory.h"
#include "cartographer_ros_msgs/SensorTopics.h"
#include "cartographer_ros_msgs/StartTrajectory.h"
#include "cartographer_ros_msgs/StatusResponse.h"
#include "cartographer_ros_msgs/SubmapEntry.h"
#include "cartographer_ros_msgs/SubmapList.h"
#include "cartographer_ros_msgs/SubmapQuery.h"
#include "cartographer_ros_msgs/TrajectoryOptions.h"
#include "cartographer_ros_msgs/WriteState.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/MultiEchoLaserScan.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf2_ros/transform_broadcaster.h"
*/

namespace cartographer_ros
{

// Wires up ROS topics to SLAM.
class Node
{
public:
    Node(const NodeOptions &node_options,
         std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder);
    ~Node();

    Node(const Node &) = delete;
    Node &
    operator=(const Node &) = delete;

    // Finishes all yet active trajectories.
    void
    FinishAllTrajectories();
    // Finishes a single given trajectory. Returns false if the trajectory did not
    // exist or was already finished.
    bool
    FinishTrajectory(int trajectory_id);

    // Runs final optimization. All trajectories have to be finished when calling.
    void
    RunFinalOptimization();

    // Starts the first trajectory with the default topics.
    void
    StartTrajectoryWithDefaultTopics(const TrajectoryOptions &options);

    // Returns unique SensorIds for multiple input bag files based on
    // their TrajectoryOptions.
    // 'SensorId::id' is the expected ROS topic name.
    std::vector<
        std::set<::cartographer::mapping::TrajectoryBuilderInterface::SensorId>>
    ComputeDefaultSensorIdsForMultipleBags(
        const std::vector<TrajectoryOptions> &bags_options) const;

    // Adds a trajectory for offline processing, i.e. not listening to topics.
    int
    AddOfflineTrajectory(
        const std::set<
            cartographer::mapping::TrajectoryBuilderInterface::SensorId> &
        expected_sensor_ids,
        const TrajectoryOptions &options);

    // The following functions handle adding sensor data to a trajectory.

    void
    HandleImuMessage(int trajectory_id, const std::string &sensor_id,
                     //const sensor_msgs::Imu::ConstPtr &msg
                    const IMU_PandCspace::IMUMessage &msg);

    void
    HandleMultiEchoLaserScanMessage(
        int trajectory_id, const std::string &sensor_id,
        //const sensor_msgs::MultiEchoLaserScan::ConstPtr &msg
        const LASER_PandCspace::LASERMessage &msg);

    // Serializes the complete Node state.
    void
    SerializeState(const std::string &filename);

    // Loads a serialized SLAM state from a .pbstream file.
    void
    GetMap();
    std::string last_frame_id_;
    timeval last_timestamp_;
    std::map<::cartographer::mapping::SubmapId, ::cartographer::io::SubmapSlice> submap_slices_ GUARDED_BY(mutex_);

private:
/*
    bool
    HandleSubmapQuery(
        cartographer_ros_msgs::SubmapQuery::Request &request,
        cartographer_ros_msgs::SubmapQuery::Response &response);
*/
    // Returns the set of SensorIds expected for a trajectory.
    // 'SensorId::id' is the expected ROS topic name.
    std::set<::cartographer::mapping::TrajectoryBuilderInterface::SensorId>
    ComputeExpectedSensorIds(
        const TrajectoryOptions &options,
        const cartographer_ros_msgs::SensorTopics &topics) const;
    int
    AddTrajectory(const TrajectoryOptions &options,
                  const cartographer_ros_msgs::SensorTopics &topics);
/*
    void
    PublishSubmapList(const ::ros::WallTimerEvent &timer_event);
*/


    void
    AddExtrapolator(int trajectory_id, const TrajectoryOptions &options);
    void
    AddSensorSamplers(int trajectory_id, const TrajectoryOptions &options);
    bool
    ValidateTrajectoryOptions(const TrajectoryOptions &options);
    bool
    ValidateTopicNames(const ::cartographer_ros_msgs::SensorTopics &topics,
                       const TrajectoryOptions &options);
    cartographer_ros_msgs::StatusResponse
    FinishTrajectoryUnderLock(
        int trajectory_id) REQUIRES(mutex_);

    const NodeOptions node_options_;

    cartographer::common::Mutex mutex_;
    MapBuilderBridge map_builder_bridge_ GUARDED_BY(mutex_);

    // These ros::ServiceServers need to live for the lifetime of the node.

    struct TrajectorySensorSamplers
    {
        TrajectorySensorSamplers(const double rangefinder_sampling_ratio,
                                 const double odometry_sampling_ratio,
                                 const double fixed_frame_pose_sampling_ratio,
                                 const double imu_sampling_ratio,
                                 const double landmark_sampling_ratio)
            : rangefinder_sampler(rangefinder_sampling_ratio),
              odometry_sampler(odometry_sampling_ratio),
              fixed_frame_pose_sampler(fixed_frame_pose_sampling_ratio),
              imu_sampler(imu_sampling_ratio),
              landmark_sampler(landmark_sampling_ratio) {}

        ::cartographer::common::FixedRatioSampler rangefinder_sampler;
        ::cartographer::common::FixedRatioSampler odometry_sampler;
        ::cartographer::common::FixedRatioSampler fixed_frame_pose_sampler;
        ::cartographer::common::FixedRatioSampler imu_sampler;
        ::cartographer::common::FixedRatioSampler landmark_sampler;
    };

    // These are keyed with 'trajectory_id'.
    std::map<int, ::cartographer::mapping::PoseExtrapolator> extrapolators_;
    std::unordered_map<int, TrajectorySensorSamplers> sensor_samplers_;
    std::unordered_set<std::string> subscribed_topics_;
    std::unordered_map<int, bool> is_active_trajectory_ GUARDED_BY(mutex_);




    // We have to keep the timer handles of ::ros::WallTimers around, otherwise
    // they do not fire.
//    std::vector<::ros::WallTimer> wall_timers_;
};

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_NODE_H

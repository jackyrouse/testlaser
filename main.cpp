#include <iostream>

#include "cartographer_ros/MyBuffer.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer_ros/node.h"
#include "dealimudata.h"
#include "deallaserdata.h"
#include "cartographer_ros/ros_log_sink.h"

DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");
DEFINE_string(load_state_filename, "",
              "If non-empty, filename of a .pbstream file to load, containing "
              "a saved SLAM state.");
DEFINE_bool(load_frozen_state, true,
            "Load the saved state as frozen (non-optimized) trajectories.");
DEFINE_bool(
    start_trajectory_with_default_topics, true,
    "Enable to immediately start the first trajectory with default topics.");
DEFINE_string(
    save_state_filename, "",
    "If non-empty, serialize state and write it to disk before shutting down.");

namespace cartographer_ros
{
namespace
{
std::string FLAGS_configuration_directory;
std::string FLAGS_configuration_basename;

void
Run()
{
    constexpr double kTfBufferCacheTimeInSeconds = 10.;
//    tf2_ros::Buffer tf_buffer{::ros::Duration(kTfBufferCacheTimeInSeconds)};
//    tf2_ros::TransformListener tf(tf_buffer);
    MyBuffer tf_buffer;
    NodeOptions node_options;
    TrajectoryOptions trajectory_options;
    std::tie(node_options, trajectory_options) =
        LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);

    auto map_builder =
        cartographer::common::make_unique<cartographer::mapping::MapBuilder>(
            node_options.map_builder_options);

    Node node(node_options, std::move(map_builder), &tf_buffer);


    node.StartTrajectoryWithDefaultTopics(trajectory_options);

    IMU_PandCspace::InitIMUItemRepository(&IMU_PandCspace::gIMUItemRepository);
    LASER_PandCspace::InitLASERItemRepository(&LASER_PandCspace::gLASERItemRepository);
    std::thread imu_producer(IMU_PandCspace::ProducerIMUTask); // 创建生产者线程.
    std::thread imu_consumer(IMU_PandCspace::ConsumerIMUTask, &node); // 创建消费之线程.
    std::thread laser_producer(LASER_PandCspace::ProducerLASERTask);
    std::thread laser_consumer(LASER_PandCspace::ConsumerLASERTask, &node);
    imu_producer.join();
    imu_consumer.join();
    laser_producer.join();
    laser_consumer.join();

//    ::ros::spin();

    node.FinishAllTrajectories();
    node.RunFinalOptimization();

    if (!FLAGS_save_state_filename.empty()) {
        node.SerializeState(FLAGS_save_state_filename);
    }
}
} // namespace
} // namespace cartographer_ros

int
main(int argc, char **argv)
{
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);

    cartographer_ros::FLAGS_configuration_directory = argv[1];
    cartographer_ros::FLAGS_configuration_basename = argv[2];
    CHECK(!cartographer_ros::FLAGS_configuration_directory.empty())
    << "-configuration_directory is missing.";
    CHECK(!cartographer_ros::FLAGS_configuration_basename.empty())
    << "-configuration_basename is missing.";

    std::cout << "Hello, World!" << std::endl;

    cartographer_ros::ScopedRosLogSink ros_log_sink;
    cartographer_ros::Run();
    return 0;
}
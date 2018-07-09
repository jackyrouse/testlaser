// Generated by gencpp from file cartographer_ros_msgs/SensorTopics.msg
// DO NOT EDIT!


#ifndef CARTOGRAPHER_ROS_MSGS_MESSAGE_SENSORTOPICS_H
#define CARTOGRAPHER_ROS_MSGS_MESSAGE_SENSORTOPICS_H

#include <string>
#include <vector>
#include <map>

namespace cartographer_ros_msgs
{
template<class ContainerAllocator>
struct SensorTopics_
{
    typedef SensorTopics_<ContainerAllocator> Type;

    SensorTopics_()
        : laser_scan_topic(),
          multi_echo_laser_scan_topic(),
          point_cloud2_topic(),
          imu_topic(),
          odometry_topic(),
          nav_sat_fix_topic(),
          landmark_topic()
    {
    }
    SensorTopics_(const ContainerAllocator &_alloc)
        : laser_scan_topic(_alloc),
          multi_echo_laser_scan_topic(_alloc),
          point_cloud2_topic(_alloc),
          imu_topic(_alloc),
          odometry_topic(_alloc),
          nav_sat_fix_topic(_alloc),
          landmark_topic(_alloc)
    {
        (void) _alloc;
    }

    typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>
        _laser_scan_topic_type;
    _laser_scan_topic_type laser_scan_topic;

    typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>
        _multi_echo_laser_scan_topic_type;
    _multi_echo_laser_scan_topic_type multi_echo_laser_scan_topic;

    typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>
        _point_cloud2_topic_type;
    _point_cloud2_topic_type point_cloud2_topic;

    typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>
        _imu_topic_type;
    _imu_topic_type imu_topic;

    typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>
        _odometry_topic_type;
    _odometry_topic_type odometry_topic;

    typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>
        _nav_sat_fix_topic_type;
    _nav_sat_fix_topic_type nav_sat_fix_topic;

    typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>
        _landmark_topic_type;
    _landmark_topic_type landmark_topic;

    typedef boost::shared_ptr<::cartographer_ros_msgs::SensorTopics_<ContainerAllocator> > Ptr;
    typedef boost::shared_ptr<::cartographer_ros_msgs::SensorTopics_<ContainerAllocator> const> ConstPtr;

}; // struct SensorTopics_

typedef ::cartographer_ros_msgs::SensorTopics_<std::allocator<void> > SensorTopics;

typedef boost::shared_ptr<::cartographer_ros_msgs::SensorTopics> SensorTopicsPtr;
typedef boost::shared_ptr<::cartographer_ros_msgs::SensorTopics const> SensorTopicsConstPtr;

// constants requiring out of line definition

} // namespace cartographer_ros_msgs

#endif // CARTOGRAPHER_ROS_MSGS_MESSAGE_SENSORTOPICS_H

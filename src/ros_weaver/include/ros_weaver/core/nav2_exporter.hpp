#ifndef ROS_WEAVER_NAV2_EXPORTER_HPP
#define ROS_WEAVER_NAV2_EXPORTER_HPP

#include <QString>
#include "ros_weaver/core/mission_data.hpp"

namespace ros_weaver {

/**
 * @brief Export missions to Nav2-compatible formats
 *
 * Supports multiple export formats:
 * - Nav2 Waypoint Follower YAML
 * - Behavior Tree XML with waypoint navigation
 * - Pose Array message
 * - Custom YAML with full mission definition
 * - Python script for programmatic control
 * - Launch files
 */
class Nav2Exporter {
public:
  // Export formats
  enum Format {
    Nav2WaypointFollower,    // nav2_waypoint_follower YAML
    Nav2BtNavigator,         // Behavior tree XML with waypoints
    PoseArray,               // geometry_msgs/PoseArray for simple cases
    CustomYAML,              // Full mission YAML with behaviors
    PythonScript,            // Python script for programmatic navigation
    LaunchFile               // ROS2 launch file
  };

  /**
   * @brief Export mission to the specified format
   * @param mission The mission to export
   * @param format The output format
   * @return The exported content as a string
   */
  static QString exportMission(const Mission& mission, Format format);

  /**
   * @brief Save exported mission to file
   * @param mission The mission to export
   * @param format The output format
   * @param filePath The output file path
   * @return true if successful
   */
  static bool saveToFile(const Mission& mission, Format format, const QString& filePath);

  // Individual format generators
  static QString generateWaypointFollowerYAML(const Mission& mission);
  static QString generateNavigationBT(const Mission& mission);
  static QString generatePoseArrayYAML(const Mission& mission);
  static QString generateCustomYAML(const Mission& mission);
  static QString generatePythonScript(const Mission& mission);
  static QString generateLaunchFile(const Mission& mission);

  // Helper methods
  static QString getDefaultExtension(Format format);
  static QString getFormatDescription(Format format);

private:
  static QString waypointToPoseYAML(const Waypoint& wp, int indent = 0);
  static QString generateBehaviorTriggerBT(const BehaviorTrigger& trigger,
                                            const QString& waypointName);
  static QString quaternionFromYaw(double theta);
  static QString indent(int level);
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_NAV2_EXPORTER_HPP

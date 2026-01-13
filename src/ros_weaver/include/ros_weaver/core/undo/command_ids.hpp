#ifndef ROS_WEAVER_COMMAND_IDS_HPP
#define ROS_WEAVER_COMMAND_IDS_HPP

namespace ros_weaver {

// Command IDs for merging consecutive commands of the same type
// Commands with the same ID can be merged (e.g., multiple small moves become one)
// Commands with ID -1 (default) cannot be merged
enum class CommandId {
  None = -1,           // No merging allowed
  MoveBlock = 1,       // Merge consecutive block moves
  ModifyGroupTitle,    // Merge consecutive title edits
  ModifyGroupColor,    // Merge consecutive color changes
  ModifyGroupSize,     // Merge consecutive resize operations
  ModifyBlockParam,    // Merge consecutive parameter changes

  // Mission planner commands
  MoveWaypoint = 100,            // Merge consecutive waypoint moves
  ChangeWaypointOrientation,     // Merge consecutive orientation changes
  MoveStartPose,                 // Merge consecutive start pose moves
  ChangeStartPoseOrientation,    // Merge consecutive start pose orientation changes
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_COMMAND_IDS_HPP

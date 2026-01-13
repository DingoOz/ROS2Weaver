#ifndef ROS_WEAVER_MISSION_DATA_HPP
#define ROS_WEAVER_MISSION_DATA_HPP

#include <QString>
#include <QPointF>
#include <QSizeF>
#include <QList>
#include <QColor>
#include <QJsonObject>
#include <QJsonArray>
#include <cmath>

namespace ros_weaver {

/**
 * @brief Behavior trigger attached to a waypoint
 */
struct BehaviorTrigger {
  enum TriggerType {
    OnArrival,        // Trigger when waypoint is reached
    OnApproach,       // Trigger when within approach distance
    OnDeparture,      // Trigger after leaving waypoint
    WhileNavigating   // Continuous during navigation to this waypoint
  };

  TriggerType type = OnArrival;
  double approachDistance = 2.0;  // For OnApproach trigger (meters)
  QString behaviorTreeId;         // Reference to behavior tree file
  QString behaviorNodeId;         // Specific node to trigger (optional)
  QJsonObject parameters;         // Additional parameters for the behavior

  QJsonObject toJson() const;
  static BehaviorTrigger fromJson(const QJsonObject& json);

  static QString triggerTypeToString(TriggerType type);
  static TriggerType stringToTriggerType(const QString& str);
};

/**
 * @brief Waypoint with position, orientation, and behavior triggers
 */
struct Waypoint {
  int id = 0;
  QString name;
  double x = 0.0;           // Position in meters (map frame)
  double y = 0.0;
  double theta = 0.0;       // Orientation in radians
  double tolerance = 0.3;   // Arrival tolerance in meters

  // Behavior triggers
  QList<BehaviorTrigger> triggers;

  // Visual properties
  QColor color = Qt::blue;
  QString description;

  QJsonObject toJson() const;
  static Waypoint fromJson(const QJsonObject& json);

  // Convenience methods
  QPointF position() const { return QPointF(x, y); }
  void setPosition(const QPointF& pos) { x = pos.x(); y = pos.y(); }

  double thetaDegrees() const { return theta * 180.0 / M_PI; }
  void setThetaDegrees(double degrees) { theta = degrees * M_PI / 180.0; }
};

/**
 * @brief Robot start pose definition
 */
struct RobotStartPose {
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  QString frameId = "map";

  QJsonObject toJson() const;
  static RobotStartPose fromJson(const QJsonObject& json);

  QPointF position() const { return QPointF(x, y); }
  void setPosition(const QPointF& pos) { x = pos.x(); y = pos.y(); }

  double thetaDegrees() const { return theta * 180.0 / M_PI; }
  void setThetaDegrees(double degrees) { theta = degrees * M_PI / 180.0; }
};

/**
 * @brief Map scale calibration data
 */
struct MapScale {
  QPointF pixelPoint1;
  QPointF pixelPoint2;
  double realWorldDistance = 1.0;  // meters
  double metersPerPixel = 0.05;    // calculated value

  void computeScale();

  // Coordinate conversions
  QPointF pixelToMeters(const QPointF& pixel) const;
  QPointF metersToPixel(const QPointF& meters) const;

  QJsonObject toJson() const;
  static MapScale fromJson(const QJsonObject& json);

  bool isValid() const { return metersPerPixel > 0.0; }
};

/**
 * @brief Complete mission definition
 */
struct Mission {
  QString name;
  QString description;
  QString mapImagePath;
  QString mapYamlPath;        // Optional: Nav2 map.yaml for georeferencing
  MapScale scale;
  RobotStartPose startPose;
  QList<Waypoint> waypoints;
  bool loopMission = false;   // Return to start after completion
  int loopCount = 1;          // -1 for infinite

  // Mission-level settings
  double defaultSpeed = 0.5;       // m/s
  double defaultTolerance = 0.3;   // meters
  QString navProfile;              // Nav2 planner profile name

  // Metadata
  QString author;
  QString createdDate;
  QString modifiedDate;

  QJsonObject toJson() const;
  static Mission fromJson(const QJsonObject& json);

  bool save(const QString& filePath) const;
  static Mission load(const QString& filePath);

  // Helper methods
  Waypoint* findWaypoint(int id);
  const Waypoint* findWaypoint(int id) const;
  int nextWaypointId() const;
  void reorderWaypoints(int fromIndex, int toIndex);

  // Validation
  bool isValid() const;
  QStringList validate() const;

  // Calculate total path distance
  double totalDistance() const;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_MISSION_DATA_HPP

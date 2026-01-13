#include "ros_weaver/core/mission_data.hpp"
#include <QFile>
#include <QJsonDocument>
#include <QDateTime>
#include <QLineF>
#include <cmath>

namespace ros_weaver {

// BehaviorTrigger implementation

QString BehaviorTrigger::triggerTypeToString(TriggerType type) {
  switch (type) {
    case OnArrival: return "on_arrival";
    case OnApproach: return "on_approach";
    case OnDeparture: return "on_departure";
    case WhileNavigating: return "while_navigating";
    default: return "on_arrival";
  }
}

BehaviorTrigger::TriggerType BehaviorTrigger::stringToTriggerType(const QString& str) {
  if (str == "on_approach") return OnApproach;
  if (str == "on_departure") return OnDeparture;
  if (str == "while_navigating") return WhileNavigating;
  return OnArrival;
}

QJsonObject BehaviorTrigger::toJson() const {
  QJsonObject json;
  json["type"] = triggerTypeToString(type);
  json["approach_distance"] = approachDistance;
  json["behavior_tree_id"] = behaviorTreeId;
  json["behavior_node_id"] = behaviorNodeId;
  if (!parameters.isEmpty()) {
    json["parameters"] = parameters;
  }
  return json;
}

BehaviorTrigger BehaviorTrigger::fromJson(const QJsonObject& json) {
  BehaviorTrigger trigger;
  trigger.type = stringToTriggerType(json["type"].toString());
  trigger.approachDistance = json["approach_distance"].toDouble(2.0);
  trigger.behaviorTreeId = json["behavior_tree_id"].toString();
  trigger.behaviorNodeId = json["behavior_node_id"].toString();
  trigger.parameters = json["parameters"].toObject();
  return trigger;
}

// Waypoint implementation

QJsonObject Waypoint::toJson() const {
  QJsonObject json;
  json["id"] = id;
  json["name"] = name;
  json["x"] = x;
  json["y"] = y;
  json["theta"] = theta;
  json["tolerance"] = tolerance;
  json["color"] = color.name();
  json["description"] = description;

  QJsonArray triggersArray;
  for (const auto& trigger : triggers) {
    triggersArray.append(trigger.toJson());
  }
  json["triggers"] = triggersArray;

  return json;
}

Waypoint Waypoint::fromJson(const QJsonObject& json) {
  Waypoint wp;
  wp.id = json["id"].toInt();
  wp.name = json["name"].toString();
  wp.x = json["x"].toDouble();
  wp.y = json["y"].toDouble();
  wp.theta = json["theta"].toDouble();
  wp.tolerance = json["tolerance"].toDouble(0.3);
  wp.color = QColor(json["color"].toString("#0000ff"));
  wp.description = json["description"].toString();

  QJsonArray triggersArray = json["triggers"].toArray();
  for (const auto& triggerVal : triggersArray) {
    wp.triggers.append(BehaviorTrigger::fromJson(triggerVal.toObject()));
  }

  return wp;
}

// RobotStartPose implementation

QJsonObject RobotStartPose::toJson() const {
  QJsonObject json;
  json["x"] = x;
  json["y"] = y;
  json["theta"] = theta;
  json["frame_id"] = frameId;
  return json;
}

RobotStartPose RobotStartPose::fromJson(const QJsonObject& json) {
  RobotStartPose pose;
  pose.x = json["x"].toDouble();
  pose.y = json["y"].toDouble();
  pose.theta = json["theta"].toDouble();
  pose.frameId = json["frame_id"].toString("map");
  return pose;
}

// MapScale implementation

void MapScale::computeScale() {
  double pixelDistance = QLineF(pixelPoint1, pixelPoint2).length();
  if (pixelDistance > 0.0) {
    metersPerPixel = realWorldDistance / pixelDistance;
  }
}

QPointF MapScale::pixelToMeters(const QPointF& pixel) const {
  return QPointF(pixel.x() * metersPerPixel, pixel.y() * metersPerPixel);
}

QPointF MapScale::metersToPixel(const QPointF& meters) const {
  if (metersPerPixel > 0.0) {
    return QPointF(meters.x() / metersPerPixel, meters.y() / metersPerPixel);
  }
  return meters;
}

QJsonObject MapScale::toJson() const {
  QJsonObject json;
  json["pixel_point1_x"] = pixelPoint1.x();
  json["pixel_point1_y"] = pixelPoint1.y();
  json["pixel_point2_x"] = pixelPoint2.x();
  json["pixel_point2_y"] = pixelPoint2.y();
  json["real_world_distance"] = realWorldDistance;
  json["meters_per_pixel"] = metersPerPixel;
  return json;
}

MapScale MapScale::fromJson(const QJsonObject& json) {
  MapScale scale;
  scale.pixelPoint1 = QPointF(json["pixel_point1_x"].toDouble(),
                               json["pixel_point1_y"].toDouble());
  scale.pixelPoint2 = QPointF(json["pixel_point2_x"].toDouble(),
                               json["pixel_point2_y"].toDouble());
  scale.realWorldDistance = json["real_world_distance"].toDouble(1.0);
  scale.metersPerPixel = json["meters_per_pixel"].toDouble(0.05);
  return scale;
}

// Mission implementation

QJsonObject Mission::toJson() const {
  QJsonObject json;
  json["version"] = "1.0";
  json["name"] = name;
  json["description"] = description;
  json["map_image_path"] = mapImagePath;
  json["map_yaml_path"] = mapYamlPath;
  json["scale"] = scale.toJson();
  json["start_pose"] = startPose.toJson();
  json["loop_mission"] = loopMission;
  json["loop_count"] = loopCount;
  json["default_speed"] = defaultSpeed;
  json["default_tolerance"] = defaultTolerance;
  json["nav_profile"] = navProfile;
  json["author"] = author;
  json["created_date"] = createdDate;
  json["modified_date"] = QDateTime::currentDateTime().toString(Qt::ISODate);

  QJsonArray waypointsArray;
  for (const auto& wp : waypoints) {
    waypointsArray.append(wp.toJson());
  }
  json["waypoints"] = waypointsArray;

  return json;
}

Mission Mission::fromJson(const QJsonObject& json) {
  Mission mission;
  mission.name = json["name"].toString();
  mission.description = json["description"].toString();
  mission.mapImagePath = json["map_image_path"].toString();
  mission.mapYamlPath = json["map_yaml_path"].toString();
  mission.scale = MapScale::fromJson(json["scale"].toObject());
  mission.startPose = RobotStartPose::fromJson(json["start_pose"].toObject());
  mission.loopMission = json["loop_mission"].toBool();
  mission.loopCount = json["loop_count"].toInt(1);
  mission.defaultSpeed = json["default_speed"].toDouble(0.5);
  mission.defaultTolerance = json["default_tolerance"].toDouble(0.3);
  mission.navProfile = json["nav_profile"].toString();
  mission.author = json["author"].toString();
  mission.createdDate = json["created_date"].toString();
  mission.modifiedDate = json["modified_date"].toString();

  QJsonArray waypointsArray = json["waypoints"].toArray();
  for (const auto& wpVal : waypointsArray) {
    mission.waypoints.append(Waypoint::fromJson(wpVal.toObject()));
  }

  return mission;
}

bool Mission::save(const QString& filePath) const {
  QFile file(filePath);
  if (!file.open(QIODevice::WriteOnly)) {
    return false;
  }

  QJsonDocument doc(toJson());
  file.write(doc.toJson(QJsonDocument::Indented));
  return true;
}

Mission Mission::load(const QString& filePath) {
  Mission mission;

  QFile file(filePath);
  if (!file.open(QIODevice::ReadOnly)) {
    return mission;
  }

  QJsonDocument doc = QJsonDocument::fromJson(file.readAll());
  if (doc.isNull() || !doc.isObject()) {
    return mission;
  }

  return fromJson(doc.object());
}

Waypoint* Mission::findWaypoint(int id) {
  for (auto& wp : waypoints) {
    if (wp.id == id) {
      return &wp;
    }
  }
  return nullptr;
}

const Waypoint* Mission::findWaypoint(int id) const {
  for (const auto& wp : waypoints) {
    if (wp.id == id) {
      return &wp;
    }
  }
  return nullptr;
}

int Mission::nextWaypointId() const {
  int maxId = 0;
  for (const auto& wp : waypoints) {
    if (wp.id > maxId) {
      maxId = wp.id;
    }
  }
  return maxId + 1;
}

void Mission::reorderWaypoints(int fromIndex, int toIndex) {
  if (fromIndex < 0 || fromIndex >= waypoints.size() ||
      toIndex < 0 || toIndex >= waypoints.size()) {
    return;
  }
  waypoints.move(fromIndex, toIndex);
}

bool Mission::isValid() const {
  return validate().isEmpty();
}

QStringList Mission::validate() const {
  QStringList errors;

  if (name.isEmpty()) {
    errors << "Mission name is required";
  }

  if (waypoints.isEmpty()) {
    errors << "At least one waypoint is required";
  }

  if (!scale.isValid()) {
    errors << "Map scale must be calibrated";
  }

  // Check for duplicate waypoint IDs
  QSet<int> seenIds;
  for (const auto& wp : waypoints) {
    if (seenIds.contains(wp.id)) {
      errors << QString("Duplicate waypoint ID: %1").arg(wp.id);
    }
    seenIds.insert(wp.id);
  }

  return errors;
}

double Mission::totalDistance() const {
  if (waypoints.size() < 2) {
    return 0.0;
  }

  double total = 0.0;

  // Distance from start pose to first waypoint
  QPointF startPos(startPose.x, startPose.y);
  total += QLineF(startPos, waypoints.first().position()).length();

  // Distance between consecutive waypoints
  for (int i = 0; i < waypoints.size() - 1; ++i) {
    total += QLineF(waypoints[i].position(), waypoints[i + 1].position()).length();
  }

  // If looping, add distance from last waypoint back to start
  if (loopMission && !waypoints.isEmpty()) {
    total += QLineF(waypoints.last().position(), startPos).length();
  }

  return total;
}

}  // namespace ros_weaver

#ifndef ROS_WEAVER_CORE_TOOLTIP_PROVIDER_HPP
#define ROS_WEAVER_CORE_TOOLTIP_PROVIDER_HPP

#include <QString>
#include <QObject>

namespace ros_weaver {

class PackageBlock;
class ConnectionLine;
struct Pin;

/**
 * @brief Provides rich HTML tooltips for canvas elements
 *
 * Generates formatted tooltips with documentation for:
 * - Package blocks (package info, parameters)
 * - Pins (topic/service info, message types)
 * - Connections (topic info, message schema)
 */
class TooltipProvider : public QObject {
  Q_OBJECT

public:
  static TooltipProvider& instance();

  // Prevent copying
  TooltipProvider(const TooltipProvider&) = delete;
  TooltipProvider& operator=(const TooltipProvider&) = delete;

  /**
   * @brief Generate tooltip for a package block
   */
  QString tooltipForBlock(const PackageBlock* block) const;

  /**
   * @brief Generate tooltip for a specific pin on a block
   */
  QString tooltipForPin(const PackageBlock* block, int pinIndex, bool isOutput) const;

  /**
   * @brief Generate tooltip for a connection line
   */
  QString tooltipForConnection(const ConnectionLine* connection) const;

  /**
   * @brief Generate tooltip for a message type
   */
  QString tooltipForMessageType(const QString& messageType) const;

  /**
   * @brief Set maximum tooltip width for wrapping
   */
  void setMaxWidth(int pixels);
  int maxWidth() const { return maxWidth_; }

  /**
   * @brief Enable/disable including message schema in tooltips
   */
  void setIncludeSchema(bool include);
  bool includeSchema() const { return includeSchema_; }

private:
  TooltipProvider();
  ~TooltipProvider() override = default;

  // Helper to format a pin's information
  QString formatPinInfo(const Pin& pin) const;

  // Helper to get a short description of a message type
  QString getMessageDescription(const QString& messageType) const;

  // Helper to format field list compactly
  QString formatFieldsCompact(const QString& messageType) const;

  // Settings
  int maxWidth_ = 350;
  bool includeSchema_ = true;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_CORE_TOOLTIP_PROVIDER_HPP

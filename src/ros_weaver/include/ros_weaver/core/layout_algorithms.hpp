#ifndef ROS_WEAVER_CORE_LAYOUT_ALGORITHMS_HPP
#define ROS_WEAVER_CORE_LAYOUT_ALGORITHMS_HPP

#include <QPointF>
#include <QRectF>
#include <QList>
#include <QMap>
#include <QUuid>

namespace ros_weaver {

class PackageBlock;
class ConnectionLine;

/**
 * @brief Enumeration of available layout algorithms
 */
enum class LayoutType {
  Hierarchical,   // Top-to-bottom or left-to-right data flow
  ForceDirected,  // Physics-based balanced distribution
  Circular,       // Arrange nodes in a circle
  Grid            // Snap all nodes to regular grid pattern
};

/**
 * @brief Options for layout algorithms
 */
struct LayoutOptions {
  // Common options
  qreal spacing = 150.0;         // Base spacing between nodes
  qreal padding = 50.0;          // Padding from edges

  // Hierarchical layout options
  bool leftToRight = false;      // true: left-to-right, false: top-to-bottom
  qreal levelSpacing = 200.0;    // Spacing between hierarchy levels

  // Force-directed options
  int iterations = 100;          // Number of simulation iterations
  qreal repulsion = 10000.0;     // Repulsion force between nodes
  qreal attraction = 0.01;       // Attraction force for connections
  qreal damping = 0.85;          // Velocity damping factor

  // Circular layout options
  qreal minRadius = 200.0;       // Minimum circle radius
  qreal startAngle = 0.0;        // Starting angle in radians

  // Grid layout options
  int gridSpacing = 200;         // Grid cell size
  int columns = 0;               // 0 = auto-calculate based on aspect ratio
};

/**
 * @brief Result of a layout operation
 */
struct LayoutResult {
  QMap<QUuid, QPointF> positions;  // New positions for each block
  QRectF bounds;                    // Bounding rect of layout
  bool success = false;
  QString errorMessage;
};

/**
 * @brief Static class providing various layout algorithms
 */
class LayoutAlgorithms {
public:
  /**
   * @brief Apply a layout algorithm to a set of blocks
   * @param blocks List of blocks to layout
   * @param connections List of connections between blocks
   * @param type The layout algorithm to use
   * @param options Layout options
   * @return Layout result with new positions
   */
  static LayoutResult apply(
      const QList<PackageBlock*>& blocks,
      const QList<ConnectionLine*>& connections,
      LayoutType type,
      const LayoutOptions& options = LayoutOptions());

  /**
   * @brief Apply hierarchical layout (data flow visualization)
   *
   * Organizes nodes in levels based on their connections.
   * Source nodes (no inputs) at top/left, sinks at bottom/right.
   */
  static LayoutResult hierarchical(
      const QList<PackageBlock*>& blocks,
      const QList<ConnectionLine*>& connections,
      const LayoutOptions& options = LayoutOptions());

  /**
   * @brief Apply force-directed layout (spring/charge simulation)
   *
   * Uses physics simulation where:
   * - All nodes repel each other
   * - Connected nodes attract each other
   * - Results in balanced, aesthetically pleasing layouts
   */
  static LayoutResult forceDirected(
      const QList<PackageBlock*>& blocks,
      const QList<ConnectionLine*>& connections,
      const LayoutOptions& options = LayoutOptions());

  /**
   * @brief Apply circular layout
   *
   * Arranges all nodes in a circle, useful for:
   * - Cyclic architectures
   * - Equal importance nodes
   * - Clear connection visibility
   */
  static LayoutResult circular(
      const QList<PackageBlock*>& blocks,
      const QList<ConnectionLine*>& connections,
      const LayoutOptions& options = LayoutOptions());

  /**
   * @brief Apply grid layout
   *
   * Snaps all nodes to a regular grid pattern.
   * Good for organized, structured views.
   */
  static LayoutResult grid(
      const QList<PackageBlock*>& blocks,
      const QList<ConnectionLine*>& connections,
      const LayoutOptions& options = LayoutOptions());

private:
  // Helper: Build adjacency list from connections
  static QMap<QUuid, QList<QUuid>> buildAdjacencyList(
      const QList<PackageBlock*>& blocks,
      const QList<ConnectionLine*>& connections,
      bool outgoing = true);

  // Helper: Calculate hierarchy level for each block (for hierarchical layout)
  static QMap<QUuid, int> calculateLevels(
      const QList<PackageBlock*>& blocks,
      const QMap<QUuid, QList<QUuid>>& outgoing,
      const QMap<QUuid, QList<QUuid>>& incoming);

  // Helper: Calculate bounding rect from positions
  static QRectF calculateBounds(
      const QMap<QUuid, QPointF>& positions,
      const QList<PackageBlock*>& blocks);
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_CORE_LAYOUT_ALGORITHMS_HPP

#include "ros_weaver/core/layout_algorithms.hpp"
#include "ros_weaver/canvas/package_block.hpp"
#include "ros_weaver/canvas/connection_line.hpp"

#include <QQueue>
#include <QSet>
#include <cmath>
#include <algorithm>

namespace ros_weaver {

LayoutResult LayoutAlgorithms::apply(
    const QList<PackageBlock*>& blocks,
    const QList<ConnectionLine*>& connections,
    LayoutType type,
    const LayoutOptions& options) {

  switch (type) {
    case LayoutType::Hierarchical:
      return hierarchical(blocks, connections, options);
    case LayoutType::ForceDirected:
      return forceDirected(blocks, connections, options);
    case LayoutType::Circular:
      return circular(blocks, connections, options);
    case LayoutType::Grid:
      return grid(blocks, connections, options);
    default:
      LayoutResult result;
      result.success = false;
      result.errorMessage = "Unknown layout type";
      return result;
  }
}

LayoutResult LayoutAlgorithms::hierarchical(
    const QList<PackageBlock*>& blocks,
    const QList<ConnectionLine*>& connections,
    const LayoutOptions& options) {

  LayoutResult result;

  if (blocks.isEmpty()) {
    result.success = true;
    return result;
  }

  // Build adjacency lists
  auto outgoing = buildAdjacencyList(blocks, connections, true);
  auto incoming = buildAdjacencyList(blocks, connections, false);

  // Calculate hierarchy levels
  auto levels = calculateLevels(blocks, outgoing, incoming);

  // Find the maximum level
  int maxLevel = 0;
  for (int level : levels) {
    maxLevel = std::max(maxLevel, level);
  }

  // Group blocks by level
  QMap<int, QList<PackageBlock*>> levelBlocks;
  for (PackageBlock* block : blocks) {
    int level = levels.value(block->id(), 0);
    levelBlocks[level].append(block);
  }

  // Position blocks
  qreal currentX = options.padding;
  qreal currentY = options.padding;

  for (int level = 0; level <= maxLevel; ++level) {
    const auto& blocksAtLevel = levelBlocks[level];
    if (blocksAtLevel.isEmpty()) continue;

    qreal maxWidth = 0;
    qreal maxHeight = 0;

    // Calculate max dimensions for this level
    for (PackageBlock* block : blocksAtLevel) {
      QRectF bounds = block->boundingRect();
      maxWidth = std::max(maxWidth, bounds.width());
      maxHeight = std::max(maxHeight, bounds.height());
    }

    // Position each block at this level
    qreal posInLevel = options.padding;
    for (PackageBlock* block : blocksAtLevel) {
      QPointF pos;
      if (options.leftToRight) {
        pos = QPointF(currentX, posInLevel);
        posInLevel += block->boundingRect().height() + options.spacing;
      } else {
        pos = QPointF(posInLevel, currentY);
        posInLevel += block->boundingRect().width() + options.spacing;
      }
      result.positions[block->id()] = pos;
    }

    // Move to next level
    if (options.leftToRight) {
      currentX += maxWidth + options.levelSpacing;
    } else {
      currentY += maxHeight + options.levelSpacing;
    }
  }

  result.bounds = calculateBounds(result.positions, blocks);
  result.success = true;
  return result;
}

LayoutResult LayoutAlgorithms::forceDirected(
    const QList<PackageBlock*>& blocks,
    const QList<ConnectionLine*>& connections,
    const LayoutOptions& options) {

  LayoutResult result;

  if (blocks.isEmpty()) {
    result.success = true;
    return result;
  }

  // Initialize positions with current positions or random
  QMap<QUuid, QPointF> positions;
  QMap<QUuid, QPointF> velocities;

  for (PackageBlock* block : blocks) {
    positions[block->id()] = block->pos();
    velocities[block->id()] = QPointF(0, 0);
  }

  // Build connection map for quick lookup
  QMap<QUuid, QSet<QUuid>> connected;
  for (ConnectionLine* conn : connections) {
    if (conn->sourceBlock() && conn->targetBlock()) {
      connected[conn->sourceBlock()->id()].insert(conn->targetBlock()->id());
      connected[conn->targetBlock()->id()].insert(conn->sourceBlock()->id());
    }
  }

  // Run simulation
  for (int iter = 0; iter < options.iterations; ++iter) {
    QMap<QUuid, QPointF> forces;

    // Initialize forces
    for (PackageBlock* block : blocks) {
      forces[block->id()] = QPointF(0, 0);
    }

    // Calculate repulsion forces (all pairs)
    for (int i = 0; i < blocks.size(); ++i) {
      for (int j = i + 1; j < blocks.size(); ++j) {
        PackageBlock* a = blocks[i];
        PackageBlock* b = blocks[j];

        QPointF posA = positions[a->id()];
        QPointF posB = positions[b->id()];

        QPointF delta = posA - posB;
        qreal distance = std::sqrt(delta.x() * delta.x() + delta.y() * delta.y());

        if (distance < 1.0) distance = 1.0;  // Prevent division by zero

        // Repulsion force (inverse square law)
        qreal force = options.repulsion / (distance * distance);

        QPointF forceVec = delta / distance * force;

        forces[a->id()] += forceVec;
        forces[b->id()] -= forceVec;
      }
    }

    // Calculate attraction forces (connected pairs)
    for (ConnectionLine* conn : connections) {
      if (!conn->sourceBlock() || !conn->targetBlock()) continue;

      QUuid srcId = conn->sourceBlock()->id();
      QUuid tgtId = conn->targetBlock()->id();

      QPointF posA = positions[srcId];
      QPointF posB = positions[tgtId];

      QPointF delta = posB - posA;
      qreal distance = std::sqrt(delta.x() * delta.x() + delta.y() * delta.y());

      if (distance < 1.0) continue;

      // Attraction force (linear spring)
      qreal force = distance * options.attraction;

      QPointF forceVec = delta / distance * force;

      forces[srcId] += forceVec;
      forces[tgtId] -= forceVec;
    }

    // Apply forces with damping
    for (PackageBlock* block : blocks) {
      QUuid id = block->id();
      velocities[id] = (velocities[id] + forces[id]) * options.damping;
      positions[id] += velocities[id];
    }
  }

  // Normalize positions (shift to positive coordinates)
  qreal minX = std::numeric_limits<qreal>::max();
  qreal minY = std::numeric_limits<qreal>::max();

  for (const QPointF& pos : positions) {
    minX = std::min(minX, pos.x());
    minY = std::min(minY, pos.y());
  }

  for (PackageBlock* block : blocks) {
    QPointF pos = positions[block->id()];
    pos.setX(pos.x() - minX + options.padding);
    pos.setY(pos.y() - minY + options.padding);
    result.positions[block->id()] = pos;
  }

  result.bounds = calculateBounds(result.positions, blocks);
  result.success = true;
  return result;
}

LayoutResult LayoutAlgorithms::circular(
    const QList<PackageBlock*>& blocks,
    const QList<ConnectionLine*>& connections,
    const LayoutOptions& options) {

  Q_UNUSED(connections)

  LayoutResult result;

  if (blocks.isEmpty()) {
    result.success = true;
    return result;
  }

  int count = blocks.size();

  // Calculate required radius based on block sizes and spacing
  qreal totalSize = 0;
  for (PackageBlock* block : blocks) {
    totalSize += std::max(block->boundingRect().width(),
                          block->boundingRect().height());
    totalSize += options.spacing;
  }

  qreal circumference = totalSize;
  qreal radius = std::max(options.minRadius, circumference / (2.0 * M_PI));

  // Position blocks in a circle
  qreal angleStep = 2.0 * M_PI / count;
  qreal currentAngle = options.startAngle;

  // Center of the circle
  qreal centerX = radius + options.padding;
  qreal centerY = radius + options.padding;

  for (PackageBlock* block : blocks) {
    qreal x = centerX + radius * std::cos(currentAngle);
    qreal y = centerY + radius * std::sin(currentAngle);

    // Offset by half block size to center the block on the circle point
    QRectF bounds = block->boundingRect();
    x -= bounds.width() / 2.0;
    y -= bounds.height() / 2.0;

    result.positions[block->id()] = QPointF(x, y);
    currentAngle += angleStep;
  }

  result.bounds = calculateBounds(result.positions, blocks);
  result.success = true;
  return result;
}

LayoutResult LayoutAlgorithms::grid(
    const QList<PackageBlock*>& blocks,
    const QList<ConnectionLine*>& connections,
    const LayoutOptions& options) {

  Q_UNUSED(connections)

  LayoutResult result;

  if (blocks.isEmpty()) {
    result.success = true;
    return result;
  }

  int count = blocks.size();

  // Calculate number of columns
  int columns = options.columns;
  if (columns <= 0) {
    // Auto-calculate: try for a roughly square grid
    columns = static_cast<int>(std::ceil(std::sqrt(count)));
  }

  int rows = (count + columns - 1) / columns;

  // Position blocks in grid
  int index = 0;
  for (int row = 0; row < rows && index < count; ++row) {
    for (int col = 0; col < columns && index < count; ++col) {
      PackageBlock* block = blocks[index];

      qreal x = options.padding + col * options.gridSpacing;
      qreal y = options.padding + row * options.gridSpacing;

      result.positions[block->id()] = QPointF(x, y);
      ++index;
    }
  }

  result.bounds = calculateBounds(result.positions, blocks);
  result.success = true;
  return result;
}

QMap<QUuid, QList<QUuid>> LayoutAlgorithms::buildAdjacencyList(
    const QList<PackageBlock*>& blocks,
    const QList<ConnectionLine*>& connections,
    bool outgoing) {

  QMap<QUuid, QList<QUuid>> adjacency;

  // Initialize empty lists for all blocks
  for (PackageBlock* block : blocks) {
    adjacency[block->id()] = QList<QUuid>();
  }

  // Build adjacency from connections
  for (ConnectionLine* conn : connections) {
    if (!conn->sourceBlock() || !conn->targetBlock()) continue;

    QUuid srcId = conn->sourceBlock()->id();
    QUuid tgtId = conn->targetBlock()->id();

    if (outgoing) {
      adjacency[srcId].append(tgtId);
    } else {
      adjacency[tgtId].append(srcId);
    }
  }

  return adjacency;
}

QMap<QUuid, int> LayoutAlgorithms::calculateLevels(
    const QList<PackageBlock*>& blocks,
    const QMap<QUuid, QList<QUuid>>& outgoing,
    const QMap<QUuid, QList<QUuid>>& incoming) {

  QMap<QUuid, int> levels;

  // Find source nodes (no incoming connections)
  QQueue<QUuid> queue;
  for (PackageBlock* block : blocks) {
    QUuid id = block->id();
    if (incoming[id].isEmpty()) {
      levels[id] = 0;
      queue.enqueue(id);
    }
  }

  // If no sources found, start with all nodes at level 0
  if (queue.isEmpty()) {
    for (PackageBlock* block : blocks) {
      levels[block->id()] = 0;
      queue.enqueue(block->id());
    }
  }

  // BFS to assign levels
  QSet<QUuid> visited;
  while (!queue.isEmpty()) {
    QUuid current = queue.dequeue();

    if (visited.contains(current)) continue;
    visited.insert(current);

    int currentLevel = levels.value(current, 0);

    // Process outgoing edges
    for (const QUuid& neighbor : outgoing[current]) {
      int neighborLevel = levels.value(neighbor, -1);
      int newLevel = currentLevel + 1;

      if (neighborLevel < newLevel) {
        levels[neighbor] = newLevel;
      }

      if (!visited.contains(neighbor)) {
        queue.enqueue(neighbor);
      }
    }
  }

  // Handle disconnected nodes
  for (PackageBlock* block : blocks) {
    if (!levels.contains(block->id())) {
      levels[block->id()] = 0;
    }
  }

  return levels;
}

QRectF LayoutAlgorithms::calculateBounds(
    const QMap<QUuid, QPointF>& positions,
    const QList<PackageBlock*>& blocks) {

  if (positions.isEmpty()) return QRectF();

  qreal minX = std::numeric_limits<qreal>::max();
  qreal minY = std::numeric_limits<qreal>::max();
  qreal maxX = std::numeric_limits<qreal>::lowest();
  qreal maxY = std::numeric_limits<qreal>::lowest();

  for (PackageBlock* block : blocks) {
    if (!positions.contains(block->id())) continue;

    QPointF pos = positions[block->id()];
    QRectF bounds = block->boundingRect();

    minX = std::min(minX, pos.x());
    minY = std::min(minY, pos.y());
    maxX = std::max(maxX, pos.x() + bounds.width());
    maxY = std::max(maxY, pos.y() + bounds.height());
  }

  return QRectF(minX, minY, maxX - minX, maxY - minY);
}

}  // namespace ros_weaver

#include "ros_weaver/core/canvas_mapper.hpp"
#include "ros_weaver/core/project.hpp"

#include <QDateTime>
#include <QSet>
#include <algorithm>
#include <cmath>

namespace ros_weaver {

CanvasMapper::CanvasMapper(QObject* parent)
  : QObject(parent)
{
}

CanvasMapper::~CanvasMapper() = default;

void CanvasMapper::setIgnoreNamespace(bool ignore) {
  ignoreNamespace_ = ignore;
}

void CanvasMapper::setCaseSensitive(bool sensitive) {
  caseSensitive_ = sensitive;
}

void CanvasMapper::setFuzzyThreshold(double threshold) {
  fuzzyThreshold_ = qBound(0.0, threshold, 1.0);
}

void CanvasMapper::mapCanvasToSystem(const Project& project, const SystemGraph& systemGraph) {
  emit mappingStarted();
  emit mappingProgress(0, tr("Starting mapping..."));

  MappingResults results;
  results.summary.timestamp = QDateTime::currentMSecsSinceEpoch();
  results.summary.totalCanvasBlocks = project.blocks().size();

  // Track which ROS2 nodes have been matched
  QSet<QString> matchedRos2Nodes;
  QSet<QString> matchedRos2Topics;

  int processedBlocks = 0;
  int totalBlocks = project.blocks().size();

  // Map each canvas block
  for (const BlockData& block : project.blocks()) {
    emit mappingProgress(
      (processedBlocks * 80) / std::max(1, totalBlocks),
      tr("Mapping block: %1").arg(block.name)
    );

    BlockMappingResult blockResult;
    blockResult.canvasBlockId = block.id;
    blockResult.canvasName = block.name;
    blockResult.totalCanvasTopics = block.inputPins.size() + block.outputPins.size();
    results.summary.totalCanvasTopics += blockResult.totalCanvasTopics;

    // Find best matching ROS2 node
    MatchConfidence confidence;
    QString matchReason;
    const DiscoveredNode* matchedNode = findBestMatch(block, systemGraph.nodes, confidence, matchReason);

    if (matchedNode) {
      blockResult.ros2NodeName = matchedNode->fullName;
      blockResult.ros2Namespace = matchedNode->namespacePath;
      blockResult.confidence = confidence;
      blockResult.matchReason = matchReason;
      matchedRos2Nodes.insert(matchedNode->fullName);

      // Match topics
      blockResult.topicMappings = matchTopics(block, *matchedNode, systemGraph);

      // Count matched and active topics
      for (const TopicMapping& tm : blockResult.topicMappings) {
        if (!tm.ros2TopicName.isEmpty()) {
          blockResult.matchedTopics++;
          results.summary.matchedTopics++;
          matchedRos2Topics.insert(tm.ros2TopicName);
        }
        if (tm.isActive) {
          results.summary.activeTopics++;
        }
      }

      results.summary.matchedBlocks++;

      switch (confidence) {
        case MatchConfidence::High:
          results.summary.highConfidenceMatches++;
          break;
        case MatchConfidence::Medium:
          results.summary.mediumConfidenceMatches++;
          break;
        case MatchConfidence::Low:
          results.summary.lowConfidenceMatches++;
          break;
        default:
          break;
      }
    } else {
      blockResult.confidence = MatchConfidence::None;
      blockResult.matchReason = tr("No matching node found in running system");
    }

    results.blockMappings.append(blockResult);
    processedBlocks++;
  }

  emit mappingProgress(85, tr("Finding unmatched system elements..."));

  // Find unmatched ROS2 nodes
  for (const DiscoveredNode& node : systemGraph.nodes) {
    if (!matchedRos2Nodes.contains(node.fullName)) {
      results.summary.unmatchedRos2Nodes.append(node.fullName);
    }
  }

  // Find unmatched ROS2 topics
  for (const DiscoveredTopic& topic : systemGraph.topics) {
    if (!matchedRos2Topics.contains(topic.name)) {
      results.summary.unmatchedRos2Topics.append(topic.name);
    }
  }

  emit mappingProgress(100, tr("Mapping complete"));

  lastResults_ = results;
  emit mappingCompleted(results);
}

const DiscoveredNode* CanvasMapper::findBestMatch(
    const BlockData& block,
    const QList<DiscoveredNode>& nodes,
    MatchConfidence& confidence,
    QString& matchReason) {

  const DiscoveredNode* bestMatch = nullptr;
  MatchConfidence bestConfidence = MatchConfidence::None;
  QString bestReason;
  double bestScore = 0.0;

  for (const DiscoveredNode& node : nodes) {
    QString reason;
    MatchConfidence nodeConfidence = matchBlockToNode(block, node, reason);

    // Calculate a numeric score for comparison
    double score = 0.0;
    switch (nodeConfidence) {
      case MatchConfidence::High: score = 1.0; break;
      case MatchConfidence::Medium: score = 0.6; break;
      case MatchConfidence::Low: score = 0.3; break;
      case MatchConfidence::None: score = 0.0; break;
    }

    // Add bonus for topic matches
    int topicMatches = 0;
    for (const PinData& pin : block.outputPins) {
      if (node.publishers.contains(pin.name) ||
          node.publishers.contains("/" + pin.name)) {
        topicMatches++;
      }
    }
    for (const PinData& pin : block.inputPins) {
      if (node.subscribers.contains(pin.name) ||
          node.subscribers.contains("/" + pin.name)) {
        topicMatches++;
      }
    }
    score += topicMatches * 0.1;

    if (score > bestScore) {
      bestScore = score;
      bestMatch = &node;
      bestConfidence = nodeConfidence;
      bestReason = reason;
    }
  }

  confidence = bestConfidence;
  matchReason = bestReason;
  return bestMatch;
}

MatchConfidence CanvasMapper::matchBlockToNode(
    const BlockData& block,
    const DiscoveredNode& node,
    QString& matchReason) {

  QString canvasName = block.name;
  QString ros2Name = node.name;
  QString ros2FullName = node.fullName;

  // Try exact match first
  if (exactNameMatch(canvasName, ros2Name) || exactNameMatch(canvasName, ros2FullName)) {
    matchReason = tr("Exact name match");
    return MatchConfidence::High;
  }

  // Try fuzzy match
  double score = 0.0;
  if (fuzzyNameMatch(canvasName, ros2Name, score)) {
    matchReason = tr("Fuzzy name match (%.0f%% similarity)").arg(score * 100);
    return MatchConfidence::Medium;
  }

  // Try matching by ignoring common suffixes
  QStringList suffixes = {"_node", "_server", "_client", "_driver", "_controller"};
  QString strippedCanvas = canvasName;
  QString strippedRos2 = ros2Name;

  for (const QString& suffix : suffixes) {
    if (strippedCanvas.endsWith(suffix, Qt::CaseInsensitive)) {
      strippedCanvas = strippedCanvas.left(strippedCanvas.length() - suffix.length());
    }
    if (strippedRos2.endsWith(suffix, Qt::CaseInsensitive)) {
      strippedRos2 = strippedRos2.left(strippedRos2.length() - suffix.length());
    }
  }

  if (exactNameMatch(strippedCanvas, strippedRos2)) {
    matchReason = tr("Name match (ignoring suffixes)");
    return MatchConfidence::Medium;
  }

  // Try topic-based matching
  int matchingTopics = 0;
  int totalCanvasTopics = block.inputPins.size() + block.outputPins.size();

  for (const PinData& pin : block.outputPins) {
    QString topicName = pin.name;
    if (!topicName.startsWith("/")) {
      topicName = "/" + topicName;
    }
    for (const QString& pub : node.publishers) {
      if (pub.endsWith(topicName) || pub == pin.name) {
        matchingTopics++;
        break;
      }
    }
  }

  for (const PinData& pin : block.inputPins) {
    QString topicName = pin.name;
    if (!topicName.startsWith("/")) {
      topicName = "/" + topicName;
    }
    for (const QString& sub : node.subscribers) {
      if (sub.endsWith(topicName) || sub == pin.name) {
        matchingTopics++;
        break;
      }
    }
  }

  if (totalCanvasTopics > 0 && matchingTopics > 0) {
    double topicMatchRatio = static_cast<double>(matchingTopics) / totalCanvasTopics;
    if (topicMatchRatio >= 0.5) {
      matchReason = tr("Topic-based match (%1/%2 topics)")
        .arg(matchingTopics).arg(totalCanvasTopics);
      return MatchConfidence::Low;
    }
  }

  return MatchConfidence::None;
}

bool CanvasMapper::exactNameMatch(const QString& canvasName, const QString& ros2Name) {
  QString cn = canvasName;
  QString rn = ros2Name;

  // Remove leading slash from ROS2 name if present
  if (rn.startsWith("/")) {
    rn = rn.mid(1);
  }

  // If ignoring namespace, extract just the node name
  if (ignoreNamespace_) {
    int lastSlash = rn.lastIndexOf('/');
    if (lastSlash >= 0) {
      rn = rn.mid(lastSlash + 1);
    }
  }

  if (caseSensitive_) {
    return cn == rn;
  } else {
    return cn.compare(rn, Qt::CaseInsensitive) == 0;
  }
}

bool CanvasMapper::fuzzyNameMatch(const QString& canvasName, const QString& ros2Name, double& score) {
  QString cn = canvasName;
  QString rn = ros2Name;

  // Remove leading slash from ROS2 name if present
  if (rn.startsWith("/")) {
    rn = rn.mid(1);
  }

  // If ignoring namespace, extract just the node name
  if (ignoreNamespace_) {
    int lastSlash = rn.lastIndexOf('/');
    if (lastSlash >= 0) {
      rn = rn.mid(lastSlash + 1);
    }
  }

  score = calculateStringSimilarity(cn, rn);
  return score >= fuzzyThreshold_;
}

double CanvasMapper::calculateStringSimilarity(const QString& s1, const QString& s2) {
  // Levenshtein distance-based similarity
  QString a = caseSensitive_ ? s1 : s1.toLower();
  QString b = caseSensitive_ ? s2 : s2.toLower();

  int m = a.length();
  int n = b.length();

  if (m == 0) return n == 0 ? 1.0 : 0.0;
  if (n == 0) return 0.0;

  // Create distance matrix
  QVector<QVector<int>> d(m + 1, QVector<int>(n + 1, 0));

  for (int i = 0; i <= m; i++) d[i][0] = i;
  for (int j = 0; j <= n; j++) d[0][j] = j;

  for (int i = 1; i <= m; i++) {
    for (int j = 1; j <= n; j++) {
      int cost = (a[i-1] == b[j-1]) ? 0 : 1;
      d[i][j] = std::min({
        d[i-1][j] + 1,      // deletion
        d[i][j-1] + 1,      // insertion
        d[i-1][j-1] + cost  // substitution
      });
    }
  }

  int maxLen = std::max(m, n);
  return 1.0 - (static_cast<double>(d[m][n]) / maxLen);
}

QList<TopicMapping> CanvasMapper::matchTopics(
    const BlockData& block,
    const DiscoveredNode& node,
    const SystemGraph& systemGraph) {

  QList<TopicMapping> mappings;

  // Match output pins (publishers)
  for (const PinData& pin : block.outputPins) {
    TopicMapping tm;
    tm.canvasPinName = pin.name;
    tm.canvasTopicName = pin.name;
    tm.messageType = pin.messageType;
    tm.isPublisher = true;

    // Try to find matching topic in node's publishers
    for (const QString& pub : node.publishers) {
      if (pub.endsWith(pin.name) || pub.endsWith("/" + pin.name) || pub == pin.name) {
        tm.ros2TopicName = pub;

        // Check if type matches
        if (systemGraph.topicTypes.contains(pub)) {
          QString ros2Type = systemGraph.topicTypes[pub];
          tm.typeMatches = (ros2Type == pin.messageType) ||
                           ros2Type.endsWith(pin.messageType) ||
                           pin.messageType.isEmpty();
        }

        // Check if topic is active (has subscribers)
        for (const DiscoveredTopic& topic : systemGraph.topics) {
          if (topic.name == pub) {
            tm.isActive = topic.subscriberCount > 0;
            break;
          }
        }
        break;
      }
    }

    mappings.append(tm);
  }

  // Match input pins (subscribers)
  for (const PinData& pin : block.inputPins) {
    TopicMapping tm;
    tm.canvasPinName = pin.name;
    tm.canvasTopicName = pin.name;
    tm.messageType = pin.messageType;
    tm.isPublisher = false;

    // Try to find matching topic in node's subscribers
    for (const QString& sub : node.subscribers) {
      if (sub.endsWith(pin.name) || sub.endsWith("/" + pin.name) || sub == pin.name) {
        tm.ros2TopicName = sub;

        // Check if type matches
        if (systemGraph.topicTypes.contains(sub)) {
          QString ros2Type = systemGraph.topicTypes[sub];
          tm.typeMatches = (ros2Type == pin.messageType) ||
                           ros2Type.endsWith(pin.messageType) ||
                           pin.messageType.isEmpty();
        }

        // Check if topic is active (has publishers)
        for (const DiscoveredTopic& topic : systemGraph.topics) {
          if (topic.name == sub) {
            tm.isActive = topic.publisherCount > 0;
            break;
          }
        }
        break;
      }
    }

    mappings.append(tm);
  }

  return mappings;
}

}  // namespace ros_weaver

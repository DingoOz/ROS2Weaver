#ifndef ROS_WEAVER_BEHAVIOR_TREE_PARSER_HPP
#define ROS_WEAVER_BEHAVIOR_TREE_PARSER_HPP

#include <QString>
#include <QList>
#include <QMap>
#include <QColor>
#include <QtXml/QDomDocument>
#include <memory>

namespace ros_weaver {

/**
 * @brief Types of nodes in a BehaviorTree.CPP tree
 */
enum class BTNodeType {
  Root,           // Root container
  BehaviorTree,   // Named tree definition
  SubTree,        // Reference to another tree
  Sequence,       // Execute children in order until one fails
  Fallback,       // Execute children until one succeeds (also called Selector)
  Parallel,       // Execute children concurrently
  ReactiveSequence,
  ReactiveFallback,
  IfThenElse,
  WhileDoElse,
  Decorator,      // Modifies child behavior (Inverter, Repeat, Retry, etc.)
  Action,         // Leaf node that performs an action
  Condition,      // Leaf node that checks a condition
  Unknown
};

/**
 * @brief Represents a single node in the behavior tree
 */
struct BTNode {
  BTNodeType type = BTNodeType::Unknown;
  QString id;                           // Node ID (from ID attribute)
  QString name;                         // Node name/type (XML tag name)
  QString displayName;                  // Display name for visualization
  QMap<QString, QString> attributes;    // All XML attributes
  QList<std::shared_ptr<BTNode>> children;
  std::weak_ptr<BTNode> parent;

  // For visualization
  qreal x = 0;
  qreal y = 0;

  bool isControlFlow() const {
    return type == BTNodeType::Sequence ||
           type == BTNodeType::Fallback ||
           type == BTNodeType::Parallel ||
           type == BTNodeType::ReactiveSequence ||
           type == BTNodeType::ReactiveFallback ||
           type == BTNodeType::IfThenElse ||
           type == BTNodeType::WhileDoElse;
  }

  bool isLeaf() const {
    return type == BTNodeType::Action || type == BTNodeType::Condition;
  }
};

/**
 * @brief Represents a complete behavior tree
 */
struct BehaviorTree {
  QString name;                         // Tree ID/name
  QString mainTreeId;                   // Which tree to execute (from root)
  std::shared_ptr<BTNode> root;
  QMap<QString, std::shared_ptr<BTNode>> treeDefinitions;  // Named subtrees
  bool isValid = false;
  QString errorMessage;
};

/**
 * @brief Parser for BehaviorTree.CPP XML format
 */
class BehaviorTreeParser {
public:
  BehaviorTreeParser() = default;

  /**
   * @brief Parse a BehaviorTree.CPP XML file
   * @param filePath Path to the XML file
   * @return Parsed behavior tree
   */
  BehaviorTree parseFile(const QString& filePath);

  /**
   * @brief Parse BehaviorTree.CPP XML from string
   * @param xmlContent XML content as string
   * @return Parsed behavior tree
   */
  BehaviorTree parseString(const QString& xmlContent);

  /**
   * @brief Get the color associated with a node type
   */
  static QColor getNodeColor(BTNodeType type);

  /**
   * @brief Get a human-readable name for the node type
   */
  static QString getNodeTypeName(BTNodeType type);

  /**
   * @brief Get the symbol/icon character for the node type
   */
  static QString getNodeSymbol(BTNodeType type);

private:
  BTNodeType determineNodeType(const QString& tagName) const;
  std::shared_ptr<BTNode> parseNode(const QDomElement& element);
  void calculateLayout(std::shared_ptr<BTNode> node, int depth, qreal& xOffset);
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_BEHAVIOR_TREE_PARSER_HPP

#include "ros_weaver/core/behavior_tree_parser.hpp"
#include <QFile>
#include <QDomDocument>
#include <QDebug>

namespace ros_weaver {

BehaviorTree BehaviorTreeParser::parseFile(const QString& filePath) {
  QFile file(filePath);
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
    BehaviorTree bt;
    bt.errorMessage = QString("Could not open file: %1").arg(filePath);
    return bt;
  }

  QString content = QString::fromUtf8(file.readAll());
  file.close();

  return parseString(content);
}

BehaviorTree BehaviorTreeParser::parseString(const QString& xmlContent) {
  BehaviorTree bt;

  QDomDocument doc;
  QString errorMsg;
  int errorLine, errorColumn;

  if (!doc.setContent(xmlContent, &errorMsg, &errorLine, &errorColumn)) {
    bt.errorMessage = QString("XML parse error at line %1, column %2: %3")
                          .arg(errorLine)
                          .arg(errorColumn)
                          .arg(errorMsg);
    return bt;
  }

  QDomElement root = doc.documentElement();

  // Handle both <root> and <BehaviorTree> as top-level elements
  if (root.tagName() == "root") {
    bt.mainTreeId = root.attribute("main_tree_to_execute", "MainTree");

    // Parse all BehaviorTree definitions
    QDomNodeList treeNodes = root.elementsByTagName("BehaviorTree");
    for (int i = 0; i < treeNodes.count(); ++i) {
      QDomElement treeElement = treeNodes.at(i).toElement();
      QString treeId = treeElement.attribute("ID", QString("Tree%1").arg(i));

      auto treeNode = std::make_shared<BTNode>();
      treeNode->type = BTNodeType::BehaviorTree;
      treeNode->id = treeId;
      treeNode->name = "BehaviorTree";
      treeNode->displayName = treeId;

      // Parse children of the BehaviorTree
      QDomNode child = treeElement.firstChild();
      while (!child.isNull()) {
        if (child.isElement()) {
          auto childNode = parseNode(child.toElement());
          if (childNode) {
            childNode->parent = treeNode;
            treeNode->children.append(childNode);
          }
        }
        child = child.nextSibling();
      }

      bt.treeDefinitions[treeId] = treeNode;

      // Set the main tree as root
      if (treeId == bt.mainTreeId) {
        bt.root = treeNode;
        bt.name = treeId;
      }
    }

    // If no main tree found, use the first one
    if (!bt.root && !bt.treeDefinitions.isEmpty()) {
      bt.root = bt.treeDefinitions.first();
      bt.name = bt.treeDefinitions.firstKey();
    }
  } else if (root.tagName() == "BehaviorTree") {
    // Single tree definition
    bt.mainTreeId = root.attribute("ID", "MainTree");
    bt.name = bt.mainTreeId;

    auto treeNode = std::make_shared<BTNode>();
    treeNode->type = BTNodeType::BehaviorTree;
    treeNode->id = bt.mainTreeId;
    treeNode->name = "BehaviorTree";
    treeNode->displayName = bt.mainTreeId;

    QDomNode child = root.firstChild();
    while (!child.isNull()) {
      if (child.isElement()) {
        auto childNode = parseNode(child.toElement());
        if (childNode) {
          childNode->parent = treeNode;
          treeNode->children.append(childNode);
        }
      }
      child = child.nextSibling();
    }

    bt.root = treeNode;
    bt.treeDefinitions[bt.mainTreeId] = treeNode;
  } else {
    bt.errorMessage = QString("Unknown root element: %1. Expected 'root' or 'BehaviorTree'")
                          .arg(root.tagName());
    return bt;
  }

  if (bt.root) {
    // Calculate layout positions
    qreal xOffset = 0;
    calculateLayout(bt.root, 0, xOffset);
    bt.isValid = true;
  } else {
    bt.errorMessage = "No valid behavior tree found in the document";
  }

  return bt;
}

std::shared_ptr<BTNode> BehaviorTreeParser::parseNode(const QDomElement& element) {
  auto node = std::make_shared<BTNode>();
  node->name = element.tagName();
  node->type = determineNodeType(element.tagName());
  node->id = element.attribute("ID", element.attribute("name", element.tagName()));
  node->displayName = element.attribute("name", node->id);

  // Store all attributes
  QDomNamedNodeMap attrs = element.attributes();
  for (int i = 0; i < attrs.count(); ++i) {
    QDomAttr attr = attrs.item(i).toAttr();
    node->attributes[attr.name()] = attr.value();
  }

  // Parse children
  QDomNode child = element.firstChild();
  while (!child.isNull()) {
    if (child.isElement()) {
      auto childNode = parseNode(child.toElement());
      if (childNode) {
        childNode->parent = node;
        node->children.append(childNode);
      }
    }
    child = child.nextSibling();
  }

  return node;
}

BTNodeType BehaviorTreeParser::determineNodeType(const QString& tagName) const {
  QString lower = tagName.toLower();

  // Control flow nodes
  if (lower == "sequence") return BTNodeType::Sequence;
  if (lower == "reactivesequence") return BTNodeType::ReactiveSequence;
  if (lower == "fallback" || lower == "selector") return BTNodeType::Fallback;
  if (lower == "reactivefallback") return BTNodeType::ReactiveFallback;
  if (lower == "parallel") return BTNodeType::Parallel;
  if (lower == "ifthenelse") return BTNodeType::IfThenElse;
  if (lower == "whiledoelse") return BTNodeType::WhileDoElse;

  // SubTree reference
  if (lower == "subtree" || lower == "subtreeplus") return BTNodeType::SubTree;

  // Decorators (common ones)
  if (lower == "inverter" || lower == "forcesuccess" || lower == "forcefailure" ||
      lower == "repeat" || lower == "retry" || lower == "timeout" ||
      lower == "delay" || lower == "keeprunninguntilsuccess" ||
      lower.startsWith("decorator")) {
    return BTNodeType::Decorator;
  }

  // Conditions (often end with "Condition" or check-like names)
  if (lower.endsWith("condition") || lower.startsWith("is") ||
      lower.startsWith("has") || lower.startsWith("check")) {
    return BTNodeType::Condition;
  }

  // Default to Action for leaf nodes, Unknown for others
  // If it has children, it's likely a custom control node
  return BTNodeType::Action;
}

void BehaviorTreeParser::calculateLayout(std::shared_ptr<BTNode> node, int depth, qreal& xOffset) {
  static const qreal NODE_WIDTH = 150.0;
  static const qreal NODE_HEIGHT = 60.0;
  static const qreal H_SPACING = 30.0;
  static const qreal V_SPACING = 80.0;

  node->y = depth * (NODE_HEIGHT + V_SPACING);

  if (node->children.isEmpty()) {
    // Leaf node
    node->x = xOffset;
    xOffset += NODE_WIDTH + H_SPACING;
  } else {
    // Parent node - position children first
    qreal startX = xOffset;
    for (auto& child : node->children) {
      calculateLayout(child, depth + 1, xOffset);
    }
    qreal endX = xOffset - H_SPACING;

    // Center parent above children
    node->x = (startX + endX - NODE_WIDTH) / 2.0;
  }
}

QColor BehaviorTreeParser::getNodeColor(BTNodeType type) {
  switch (type) {
    case BTNodeType::Root:
    case BTNodeType::BehaviorTree:
      return QColor(100, 100, 100);       // Gray
    case BTNodeType::Sequence:
    case BTNodeType::ReactiveSequence:
      return QColor(76, 175, 80);         // Green
    case BTNodeType::Fallback:
    case BTNodeType::ReactiveFallback:
      return QColor(255, 152, 0);         // Orange
    case BTNodeType::Parallel:
      return QColor(33, 150, 243);        // Blue
    case BTNodeType::IfThenElse:
    case BTNodeType::WhileDoElse:
      return QColor(156, 39, 176);        // Purple
    case BTNodeType::Decorator:
      return QColor(0, 188, 212);         // Cyan
    case BTNodeType::Action:
      return QColor(63, 81, 181);         // Indigo
    case BTNodeType::Condition:
      return QColor(255, 193, 7);         // Amber
    case BTNodeType::SubTree:
      return QColor(121, 85, 72);         // Brown
    default:
      return QColor(158, 158, 158);       // Gray
  }
}

QString BehaviorTreeParser::getNodeTypeName(BTNodeType type) {
  switch (type) {
    case BTNodeType::Root: return "Root";
    case BTNodeType::BehaviorTree: return "BehaviorTree";
    case BTNodeType::SubTree: return "SubTree";
    case BTNodeType::Sequence: return "Sequence";
    case BTNodeType::ReactiveSequence: return "ReactiveSequence";
    case BTNodeType::Fallback: return "Fallback";
    case BTNodeType::ReactiveFallback: return "ReactiveFallback";
    case BTNodeType::Parallel: return "Parallel";
    case BTNodeType::IfThenElse: return "IfThenElse";
    case BTNodeType::WhileDoElse: return "WhileDoElse";
    case BTNodeType::Decorator: return "Decorator";
    case BTNodeType::Action: return "Action";
    case BTNodeType::Condition: return "Condition";
    default: return "Unknown";
  }
}

QString BehaviorTreeParser::getNodeSymbol(BTNodeType type) {
  switch (type) {
    case BTNodeType::Sequence:
    case BTNodeType::ReactiveSequence:
      return "→";                         // Right arrow for sequence
    case BTNodeType::Fallback:
    case BTNodeType::ReactiveFallback:
      return "?";                         // Question mark for fallback
    case BTNodeType::Parallel:
      return "⇉";                         // Parallel arrows
    case BTNodeType::Decorator:
      return "◇";                         // Diamond
    case BTNodeType::Action:
      return "▶";                         // Play/action
    case BTNodeType::Condition:
      return "?";                         // Question/check
    case BTNodeType::SubTree:
      return "↳";                         // Tree reference
    default:
      return "";
  }
}

}  // namespace ros_weaver

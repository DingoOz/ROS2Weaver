#include "ros_weaver/core/tooltip_provider.hpp"
#include "ros_weaver/core/ros_docs_provider.hpp"
#include "ros_weaver/canvas/package_block.hpp"
#include "ros_weaver/canvas/connection_line.hpp"

namespace ros_weaver {

TooltipProvider& TooltipProvider::instance() {
  static TooltipProvider instance;
  return instance;
}

TooltipProvider::TooltipProvider()
  : QObject(nullptr)
  , maxWidth_(350)
  , includeSchema_(true)
{
}

QString TooltipProvider::tooltipForBlock(const PackageBlock* block) const {
  if (!block) return QString();

  QString html;
  html += QString("<div style='max-width: %1px;'>").arg(maxWidth_);

  // Block name as title
  html += QString("<b style='font-size: 11pt; color: #2a82da;'>%1</b>")
          .arg(block->packageName());

  // Runtime status if available
  switch (block->runtimeStatus()) {
    case BlockRuntimeStatus::Running:
      html += " <span style='color: #4caf50;'>[Running]</span>";
      if (!block->matchedNodeName().isEmpty()) {
        html += QString("<br/><small>Node: %1</small>").arg(block->matchedNodeName());
      }
      break;
    case BlockRuntimeStatus::PartialMatch:
      html += " <span style='color: #ff9800;'>[Partial Match]</span>";
      break;
    case BlockRuntimeStatus::NotFound:
      html += " <span style='color: #9e9e9e;'>[Not Found]</span>";
      break;
    default:
      break;
  }

  // Try to get package documentation
  PackageDoc packageDoc = RosDocsProvider::instance().getPackageDoc(block->packageName());
  if (packageDoc.isValid && !packageDoc.description.isEmpty()) {
    html += QString("<br/><br/><i>%1</i>").arg(packageDoc.description);
  }

  // Input pins summary
  const auto& inputs = block->inputPins();
  if (!inputs.isEmpty()) {
    html += "<br/><br/><b>Inputs:</b>";
    int count = 0;
    for (const Pin& pin : inputs) {
      if (count >= 3) {
        html += QString("<br/>&nbsp;&nbsp;... and %1 more").arg(inputs.size() - 3);
        break;
      }
      html += QString("<br/>&nbsp;&nbsp;%1 <span style='color: #888;'>(%2)</span>")
              .arg(pin.name, pin.messageType.isEmpty() ? "any" : pin.messageType);
      ++count;
    }
  }

  // Output pins summary
  const auto& outputs = block->outputPins();
  if (!outputs.isEmpty()) {
    html += "<br/><br/><b>Outputs:</b>";
    int count = 0;
    for (const Pin& pin : outputs) {
      if (count >= 3) {
        html += QString("<br/>&nbsp;&nbsp;... and %1 more").arg(outputs.size() - 3);
        break;
      }
      html += QString("<br/>&nbsp;&nbsp;%1 <span style='color: #888;'>(%2)</span>")
              .arg(pin.name, pin.messageType.isEmpty() ? "any" : pin.messageType);
      ++count;
    }
  }

  // Parameters count
  if (block->hasParameters()) {
    html += QString("<br/><br/><small>%1 parameters configured</small>")
            .arg(block->parameters().size());
  }

  html += "</div>";
  return html;
}

QString TooltipProvider::tooltipForPin(const PackageBlock* block, int pinIndex, bool isOutput) const {
  if (!block) return QString();

  const QList<Pin>& pins = isOutput ? block->outputPins() : block->inputPins();
  if (pinIndex < 0 || pinIndex >= pins.size()) return QString();

  const Pin& pin = pins[pinIndex];

  QString html;
  html += QString("<div style='max-width: %1px;'>").arg(maxWidth_);

  // Pin type icon/color indicator
  QString typeColor;
  QString typeName;
  switch (pin.dataType) {
    case Pin::DataType::Topic:
      typeColor = "#64c864";
      typeName = "Topic";
      break;
    case Pin::DataType::Service:
      typeColor = "#6496ff";
      typeName = "Service";
      break;
    case Pin::DataType::Action:
      typeColor = "#ffb464";
      typeName = "Action";
      break;
    case Pin::DataType::Parameter:
      typeColor = "#c864c8";
      typeName = "Parameter";
      break;
  }

  // Pin name and type
  html += QString("<b style='font-size: 11pt; color: %1;'>%2</b> <span style='color: #888;'>(%3 %4)</span>")
          .arg(typeColor, pin.name, typeName, isOutput ? "output" : "input");

  // Message type
  if (!pin.messageType.isEmpty()) {
    html += QString("<br/><br/><b>Type:</b> <code style='background: #333; padding: 2px 4px;'>%1</code>")
            .arg(pin.messageType);

    // Include schema if enabled
    if (includeSchema_) {
      QString fields = formatFieldsCompact(pin.messageType);
      if (!fields.isEmpty()) {
        html += QString("<br/><br/><b>Fields:</b><br/>%1").arg(fields);
      }
    }
  }

  html += "</div>";
  return html;
}

QString TooltipProvider::tooltipForConnection(const ConnectionLine* connection) const {
  if (!connection) return QString();

  QString html;
  html += QString("<div style='max-width: %1px;'>").arg(maxWidth_);

  // Topic/Service name
  QString topicName = connection->topicName();
  if (topicName.isEmpty()) {
    topicName = "Unnamed connection";
  }

  html += QString("<b style='font-size: 11pt; color: #64c864;'>%1</b>").arg(topicName);

  // Source and target
  if (connection->sourceBlock() && connection->targetBlock()) {
    html += QString("<br/><br/>%1 → %2")
            .arg(connection->sourceBlock()->packageName(),
                 connection->targetBlock()->packageName());
  }

  // Message type
  QString msgType = connection->messageType();
  if (!msgType.isEmpty()) {
    html += QString("<br/><br/><b>Type:</b> <code style='background: #333; padding: 2px 4px;'>%1</code>")
            .arg(msgType);

    // Include schema if enabled
    if (includeSchema_) {
      QString fields = formatFieldsCompact(msgType);
      if (!fields.isEmpty()) {
        html += QString("<br/><br/><b>Fields:</b><br/>%1").arg(fields);
      }
    }
  }

  // Message rate if available
  if (connection->isLiveMonitoringEnabled()) {
    double rate = connection->messageRate();
    if (rate > 0) {
      html += QString("<br/><br/><b>Rate:</b> %1 Hz").arg(rate, 0, 'f', 1);
    }
  }

  html += "</div>";
  return html;
}

QString TooltipProvider::tooltipForMessageType(const QString& messageType) const {
  if (messageType.isEmpty()) return QString();

  QString html;
  html += QString("<div style='max-width: %1px;'>").arg(maxWidth_);

  html += QString("<b style='font-size: 11pt;'>%1</b>").arg(messageType);

  // Get documentation
  InterfaceDoc doc = RosDocsProvider::instance().getInterfaceDoc(messageType);
  if (doc.isValid) {
    if (!doc.description.isEmpty()) {
      html += QString("<br/><br/><i>%1</i>").arg(doc.description);
    }

    // Show fields
    if (!doc.fields.isEmpty()) {
      html += "<br/><br/><b>Fields:</b>";
      int count = 0;
      for (const FieldInfo& field : doc.fields) {
        if (count >= 8) {
          html += QString("<br/>&nbsp;&nbsp;... and %1 more fields").arg(doc.fields.size() - 8);
          break;
        }
        QString indent = QString("&nbsp;").repeated(field.depth * 2 + 2);
        html += QString("<br/>%1<span style='color: #6496ff;'>%2</span> %3")
                .arg(indent, field.type, field.name);
        ++count;
      }
    }
  }

  html += "</div>";
  return html;
}

QString TooltipProvider::formatPinInfo(const Pin& pin) const {
  QString info;
  info += QString("<b>%1</b>").arg(pin.name);
  if (!pin.messageType.isEmpty()) {
    info += QString(" <span style='color: #888;'>(%1)</span>").arg(pin.messageType);
  }
  return info;
}

QString TooltipProvider::getMessageDescription(const QString& messageType) const {
  if (messageType.isEmpty()) return QString();

  InterfaceDoc doc = RosDocsProvider::instance().getInterfaceDoc(messageType);
  if (doc.isValid && !doc.description.isEmpty()) {
    return doc.description;
  }
  return QString();
}

QString TooltipProvider::formatFieldsCompact(const QString& messageType) const {
  if (messageType.isEmpty()) return QString();

  InterfaceDoc doc = RosDocsProvider::instance().getInterfaceDoc(messageType);
  if (!doc.isValid || doc.fields.isEmpty()) return QString();

  QString result;
  int count = 0;
  for (const FieldInfo& field : doc.fields) {
    if (field.depth > 0) continue;  // Only show top-level fields

    if (count >= 5) {
      result += QString("<br/>&nbsp;&nbsp;... and more fields");
      break;
    }

    result += QString("<br/>&nbsp;&nbsp;<span style='color: #6496ff;'>%1</span> %2")
              .arg(field.type, field.name);
    ++count;
  }

  return result;
}

void TooltipProvider::setMaxWidth(int pixels) {
  maxWidth_ = pixels;
}

void TooltipProvider::setIncludeSchema(bool include) {
  includeSchema_ = include;
}

}  // namespace ros_weaver

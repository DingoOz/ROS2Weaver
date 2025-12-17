// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2025, ROS Weaver Contributors

#pragma once

#include <QJsonObject>
#include <QJsonArray>
#include <QString>
#include <QUuid>
#include <QDateTime>
#include <QVariant>
#include <QList>
#include <QMap>

namespace ros_weaver {

/**
 * @brief MCP Tool definition
 * Describes a callable function/tool exposed by an MCP server
 */
struct MCPTool {
    QString name;
    QString description;
    QJsonObject inputSchema;  // JSON Schema for parameters

    QJsonObject toJson() const {
        QJsonObject obj;
        obj["name"] = name;
        obj["description"] = description;
        obj["inputSchema"] = inputSchema;
        return obj;
    }

    static MCPTool fromJson(const QJsonObject& obj) {
        MCPTool tool;
        tool.name = obj["name"].toString();
        tool.description = obj["description"].toString();
        tool.inputSchema = obj["inputSchema"].toObject();
        return tool;
    }
};

/**
 * @brief MCP Resource definition
 * Describes a data resource accessible via an MCP server
 */
struct MCPResource {
    QString uri;
    QString name;
    QString description;
    QString mimeType;

    QJsonObject toJson() const {
        QJsonObject obj;
        obj["uri"] = uri;
        obj["name"] = name;
        obj["description"] = description;
        obj["mimeType"] = mimeType;
        return obj;
    }

    static MCPResource fromJson(const QJsonObject& obj) {
        MCPResource res;
        res.uri = obj["uri"].toString();
        res.name = obj["name"].toString();
        res.description = obj["description"].toString();
        res.mimeType = obj["mimeType"].toString();
        return res;
    }
};

/**
 * @brief Result of an MCP tool call
 */
struct MCPToolResult {
    bool success = false;
    QString message;
    QJsonObject data;
    QByteArray binaryData;  // For images/binary content
    QString mimeType;       // Content type if binary
    qint64 executionTimeMs = 0;

    QJsonObject toJson() const {
        QJsonObject obj;
        obj["success"] = success;
        obj["message"] = message;
        obj["data"] = data;
        obj["executionTimeMs"] = executionTimeMs;
        if (!mimeType.isEmpty()) {
            obj["mimeType"] = mimeType;
        }
        return obj;
    }
};

/**
 * @brief Result of reading an MCP resource
 */
struct MCPResourceContent {
    QString uri;
    QString mimeType;
    QString textContent;
    QByteArray binaryContent;  // For images/binary
    bool isBinary = false;

    QJsonObject toJson() const {
        QJsonObject obj;
        obj["uri"] = uri;
        obj["mimeType"] = mimeType;
        if (isBinary) {
            obj["blob"] = QString::fromLatin1(binaryContent.toBase64());
        } else {
            obj["text"] = textContent;
        }
        return obj;
    }
};

/**
 * @brief MCP Server state
 */
enum class MCPServerState {
    Disconnected,
    Connecting,
    Connected,
    Error
};

/**
 * @brief Data direction for MCP communication
 */
enum class MCPDataDirection {
    None,
    Sending,    // Data going to MCP server
    Receiving,  // Data coming from MCP server
    Bidirectional
};

/**
 * @brief Statistics for an MCP server
 */
struct MCPServerStats {
    qint64 bytesReceived = 0;
    qint64 bytesSent = 0;
    qint64 toolCallCount = 0;
    qint64 resourceReadCount = 0;
    qint64 errorCount = 0;
    QDateTime connectedSince;
    double cpuUsagePercent = 0.0;
    qint64 memoryUsageBytes = 0;

    qint64 totalBytesTransferred() const {
        return bytesReceived + bytesSent;
    }

    QString uptime() const {
        if (!connectedSince.isValid()) return "Not connected";
        qint64 secs = connectedSince.secsTo(QDateTime::currentDateTime());
        int hours = secs / 3600;
        int mins = (secs % 3600) / 60;
        int s = secs % 60;
        return QString("%1h %2m %3s").arg(hours).arg(mins).arg(s);
    }

    QJsonObject toJson() const {
        QJsonObject obj;
        obj["bytesReceived"] = bytesReceived;
        obj["bytesSent"] = bytesSent;
        obj["toolCallCount"] = toolCallCount;
        obj["resourceReadCount"] = resourceReadCount;
        obj["errorCount"] = errorCount;
        obj["uptime"] = uptime();
        obj["cpuUsagePercent"] = cpuUsagePercent;
        obj["memoryUsageBytes"] = memoryUsageBytes;
        return obj;
    }
};

/**
 * @brief MCP Server configuration
 */
struct MCPServerConfig {
    QUuid id;
    QString name;
    QString description;
    QString serverType;  // "ros_logs", "ros_topics", "system_stats", etc.
    bool enabled = true;
    bool autoStart = true;
    QJsonObject settings;  // Server-specific settings

    QJsonObject toJson() const {
        QJsonObject obj;
        obj["id"] = id.toString();
        obj["name"] = name;
        obj["description"] = description;
        obj["serverType"] = serverType;
        obj["enabled"] = enabled;
        obj["autoStart"] = autoStart;
        obj["settings"] = settings;
        return obj;
    }

    static MCPServerConfig fromJson(const QJsonObject& obj) {
        MCPServerConfig config;
        config.id = QUuid::fromString(obj["id"].toString());
        if (config.id.isNull()) {
            config.id = QUuid::createUuid();
        }
        config.name = obj["name"].toString();
        config.description = obj["description"].toString();
        config.serverType = obj["serverType"].toString();
        config.enabled = obj["enabled"].toBool(true);
        config.autoStart = obj["autoStart"].toBool(true);
        config.settings = obj["settings"].toObject();
        return config;
    }
};

}  // namespace ros_weaver

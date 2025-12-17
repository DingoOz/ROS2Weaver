// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2025, ROS Weaver Contributors

#pragma once

#include <QObject>
#include <QTimer>
#include <QMutex>
#include <memory>
#include "mcp_types.hpp"

namespace ros_weaver {

/**
 * @brief Abstract base class for MCP servers
 *
 * An MCP server provides tools and resources that can be accessed by the AI.
 * Subclasses implement specific functionality (ROS logs, topics, system stats, etc.)
 */
class MCPServer : public QObject {
    Q_OBJECT

public:
    explicit MCPServer(const MCPServerConfig& config, QObject* parent = nullptr);
    ~MCPServer() override;

    // Configuration
    QUuid id() const { return config_.id; }
    QString name() const { return config_.name; }
    QString description() const { return config_.description; }
    QString serverType() const { return config_.serverType; }
    MCPServerConfig config() const { return config_; }
    void setConfig(const MCPServerConfig& config);

    // State
    MCPServerState state() const { return state_; }
    bool isConnected() const { return state_ == MCPServerState::Connected; }
    QString lastError() const { return lastError_; }

    // Statistics
    MCPServerStats stats() const;
    void resetStats();

    // Tools and Resources
    virtual QList<MCPTool> availableTools() const = 0;
    virtual QList<MCPResource> availableResources() const = 0;

    // Operations
    virtual MCPToolResult callTool(const QString& toolName, const QJsonObject& params) = 0;
    virtual MCPResourceContent readResource(const QString& uri) = 0;

    // Lifecycle
    virtual void start();
    virtual void stop();
    virtual void restart();

signals:
    void stateChanged(MCPServerState state);
    void errorOccurred(const QString& error);
    void dataReceived(qint64 bytes);
    void dataSent(qint64 bytes);
    void toolCalled(const QString& toolName);
    void resourceRead(const QString& uri);
    void statsUpdated(const MCPServerStats& stats);

protected:
    void setState(MCPServerState state);
    void setError(const QString& error);
    void recordBytesReceived(qint64 bytes);
    void recordBytesSent(qint64 bytes);
    void recordToolCall();
    void recordResourceRead();
    void recordError();

    // Subclasses can override for custom initialization/cleanup
    virtual void onStart() {}
    virtual void onStop() {}

protected:
    MCPServerConfig config_;
    MCPServerState state_ = MCPServerState::Disconnected;
    QString lastError_;
    MCPServerStats stats_;
    mutable QMutex statsMutex_;

private:
    QTimer* statsTimer_;
    void updateStatsTimer();
};

}  // namespace ros_weaver

// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2025, ROS Weaver Contributors

#pragma once

#include <QObject>
#include <QMap>
#include <QTimer>
#include <memory>
#include "mcp_types.hpp"
#include "mcp_server.hpp"

namespace ros_weaver {

class WeaverCanvas;

/**
 * @brief Manager for all MCP servers
 *
 * Handles lifecycle, registration, and coordination of MCP servers.
 * Acts as the central hub for MCP functionality in the application.
 */
class MCPManager : public QObject {
    Q_OBJECT

public:
    static MCPManager& instance();

    // Server management
    void registerServer(std::shared_ptr<MCPServer> server);
    void unregisterServer(const QUuid& serverId);
    std::shared_ptr<MCPServer> server(const QUuid& id) const;
    std::shared_ptr<MCPServer> serverByName(const QString& name) const;
    std::shared_ptr<MCPServer> serverByType(const QString& type) const;
    QList<std::shared_ptr<MCPServer>> allServers() const;
    QList<std::shared_ptr<MCPServer>> activeServers() const;

    // Server factory - creates built-in server types
    std::shared_ptr<MCPServer> createServer(const MCPServerConfig& config);

    // Built-in server types
    QStringList availableServerTypes() const;
    QString serverTypeDescription(const QString& type) const;
    MCPServerConfig defaultConfigForType(const QString& type) const;

    // Lifecycle
    void startAll();
    void stopAll();
    void startServer(const QUuid& id);
    void stopServer(const QUuid& id);

    // Tool/Resource aggregation
    QList<MCPTool> allTools() const;
    QList<MCPResource> allResources() const;
    MCPToolResult callTool(const QString& serverName, const QString& toolName,
                          const QJsonObject& params);
    MCPResourceContent readResource(const QString& serverName, const QString& uri);

    // Global statistics
    MCPServerStats aggregateStats() const;
    qint64 totalBytesTransferred() const;
    qint64 totalToolCalls() const;

    // Configuration persistence
    void saveConfiguration();
    void loadConfiguration();

    // Canvas integration for AI suggestions
    void setCanvas(WeaverCanvas* canvas);
    WeaverCanvas* canvas() const { return canvas_; }

    // AI Integration - suggest MCP server creation
    struct MCPSuggestion {
        QString serverType;
        QString reason;
        MCPServerConfig suggestedConfig;
    };
    MCPSuggestion suggestServer(const QString& userQuery) const;

signals:
    void serverRegistered(const QUuid& id);
    void serverUnregistered(const QUuid& id);
    void serverStateChanged(const QUuid& id, MCPServerState state);
    void serverDataActivity(const QUuid& id, MCPDataDirection direction, qint64 bytes);
    void allStatsUpdated();
    void mcpSuggested(const MCPSuggestion& suggestion);

private:
    MCPManager();
    ~MCPManager() override;

    // Singleton
    MCPManager(const MCPManager&) = delete;
    MCPManager& operator=(const MCPManager&) = delete;

    void setupBuiltInServers();
    void connectServerSignals(std::shared_ptr<MCPServer> server);

private:
    QMap<QUuid, std::shared_ptr<MCPServer>> servers_;
    WeaverCanvas* canvas_ = nullptr;
    QTimer* statsUpdateTimer_;
};

}  // namespace ros_weaver

// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2025, ROS Weaver Contributors

#pragma once

#include "mcp_server.hpp"
#include <QProcess>
#include <QTimer>
#include <QFile>
#include <QDir>
#include <deque>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/log.hpp>

namespace ros_weaver {

/**
 * @brief MCP Server for ROS 2 log messages
 *
 * Provides access to ROS 2 log messages via rosout subscription
 */
class ROSLogsMCPServer : public MCPServer {
    Q_OBJECT

public:
    explicit ROSLogsMCPServer(const MCPServerConfig& config, QObject* parent = nullptr);
    ~ROSLogsMCPServer() override;

    QList<MCPTool> availableTools() const override;
    QList<MCPResource> availableResources() const override;
    MCPToolResult callTool(const QString& toolName, const QJsonObject& params) override;
    MCPResourceContent readResource(const QString& uri) override;

protected:
    void onStart() override;
    void onStop() override;

private:
    struct LogEntry {
        QDateTime timestamp;
        QString level;
        QString nodeName;
        QString message;
        QString function;
        QString file;
        int line;

        QJsonObject toJson() const;
    };

    void processLogMessage(const rcl_interfaces::msg::Log::SharedPtr msg);
    QString severityToString(int severity) const;

    std::shared_ptr<rclcpp::Node> rosNode_;
    rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr subscription_;
    std::unique_ptr<std::thread> spinThread_;
    std::atomic<bool> spinning_{false};

    std::deque<LogEntry> logBuffer_;
    int maxLines_ = 1000;
    QString severityFilter_ = "debug";
    mutable QMutex logMutex_;
};

/**
 * @brief MCP Server for ROS 2 topics
 *
 * Provides tools to list, info, and echo ROS 2 topics
 */
class ROSTopicsMCPServer : public MCPServer {
    Q_OBJECT

public:
    explicit ROSTopicsMCPServer(const MCPServerConfig& config, QObject* parent = nullptr);
    ~ROSTopicsMCPServer() override;

    QList<MCPTool> availableTools() const override;
    QList<MCPResource> availableResources() const override;
    MCPToolResult callTool(const QString& toolName, const QJsonObject& params) override;
    MCPResourceContent readResource(const QString& uri) override;

protected:
    void onStart() override;
    void onStop() override;

private:
    MCPToolResult listTopics();
    MCPToolResult topicInfo(const QString& topicName);
    MCPToolResult echoTopic(const QString& topicName, int count);
    MCPToolResult listNodes();
    MCPToolResult nodeInfo(const QString& nodeName);

    QString runRosCommand(const QStringList& args);

    std::shared_ptr<rclcpp::Node> rosNode_;
    std::unique_ptr<std::thread> spinThread_;
    std::atomic<bool> spinning_{false};
};

/**
 * @brief MCP Server for ROSbag operations
 *
 * Provides tools to check recording status and rosbag metadata
 */
class RosbagMCPServer : public MCPServer {
    Q_OBJECT

public:
    explicit RosbagMCPServer(const MCPServerConfig& config, QObject* parent = nullptr);
    ~RosbagMCPServer() override;

    QList<MCPTool> availableTools() const override;
    QList<MCPResource> availableResources() const override;
    MCPToolResult callTool(const QString& toolName, const QJsonObject& params) override;
    MCPResourceContent readResource(const QString& uri) override;

    // External hooks for integration with RosbagWorkbenchPanel
    void setRecordingState(bool recording, const QString& bagPath = QString());
    bool isRecording() const { return isRecording_; }
    QString currentBagPath() const { return currentBagPath_; }

protected:
    void onStart() override;
    void onStop() override;

private:
    MCPToolResult getRecordingStatus();
    MCPToolResult getBagInfo(const QString& bagPath);
    MCPToolResult listBagsInDirectory(const QString& directory);

    bool isRecording_ = false;
    QString currentBagPath_;
    QDateTime recordingStartTime_;
    QStringList recordingTopics_;
};

/**
 * @brief MCP Server for ros2_control
 *
 * Provides tools to monitor and manage ros2_control lifecycle
 */
class ROSControlMCPServer : public MCPServer {
    Q_OBJECT

public:
    explicit ROSControlMCPServer(const MCPServerConfig& config, QObject* parent = nullptr);
    ~ROSControlMCPServer() override;

    QList<MCPTool> availableTools() const override;
    QList<MCPResource> availableResources() const override;
    MCPToolResult callTool(const QString& toolName, const QJsonObject& params) override;
    MCPResourceContent readResource(const QString& uri) override;

protected:
    void onStart() override;
    void onStop() override;

private:
    MCPToolResult listControllers();
    MCPToolResult controllerInfo(const QString& controllerName);
    MCPToolResult listHardwareInterfaces();
    MCPToolResult switchController(const QString& activate, const QString& deactivate);

    QString runRosControlCommand(const QStringList& args);

    QString controllerManagerName_;
};

/**
 * @brief MCP Server for system statistics
 *
 * Provides tools to monitor CPU, memory, disk, and temperatures
 */
class SystemStatsMCPServer : public MCPServer {
    Q_OBJECT

public:
    explicit SystemStatsMCPServer(const MCPServerConfig& config, QObject* parent = nullptr);
    ~SystemStatsMCPServer() override;

    QList<MCPTool> availableTools() const override;
    QList<MCPResource> availableResources() const override;
    MCPToolResult callTool(const QString& toolName, const QJsonObject& params) override;
    MCPResourceContent readResource(const QString& uri) override;

protected:
    void onStart() override;
    void onStop() override;

private:
    struct CPUStats {
        double usagePercent;
        int coreCount;
        QList<double> perCoreUsage;
        double loadAverage1;
        double loadAverage5;
        double loadAverage15;
    };

    struct MemoryStats {
        qint64 totalBytes;
        qint64 usedBytes;
        qint64 availableBytes;
        qint64 swapTotalBytes;
        qint64 swapUsedBytes;
    };

    struct DiskStats {
        QString mountPoint;
        QString filesystem;
        qint64 totalBytes;
        qint64 usedBytes;
        qint64 availableBytes;
        double usagePercent;
    };

    struct TempReading {
        QString sensor;
        QString label;
        double temperatureCelsius;
        double criticalTemp;
    };

    MCPToolResult getCPUStats();
    MCPToolResult getMemoryStats();
    MCPToolResult getDiskStats();
    MCPToolResult getTemperatures();
    MCPToolResult getAllStats();

    CPUStats readCPUStats();
    MemoryStats readMemoryStats();
    QList<DiskStats> readDiskStats();
    QList<TempReading> readTemperatures();

    QTimer* updateTimer_;
    int updateIntervalMs_ = 1000;
    bool includeTemperatures_ = true;

    // Cached values for CPU calculation
    qint64 prevIdleTime_ = 0;
    qint64 prevTotalTime_ = 0;
};

/**
 * @brief MCP Server for file system access
 *
 * Provides tools to list and read files
 */
class FilesMCPServer : public MCPServer {
    Q_OBJECT

public:
    explicit FilesMCPServer(const MCPServerConfig& config, QObject* parent = nullptr);
    ~FilesMCPServer() override;

    QList<MCPTool> availableTools() const override;
    QList<MCPResource> availableResources() const override;
    MCPToolResult callTool(const QString& toolName, const QJsonObject& params) override;
    MCPResourceContent readResource(const QString& uri) override;

protected:
    void onStart() override;
    void onStop() override;

private:
    MCPToolResult listDirectory(const QString& path, bool showHidden = false);
    MCPToolResult readFile(const QString& path, int maxBytes = -1);
    MCPToolResult fileInfo(const QString& path);
    MCPToolResult searchFiles(const QString& directory, const QString& pattern);

    bool isPathAllowed(const QString& path) const;
    QString normalizePath(const QString& path) const;

    QStringList allowedPaths_;
    qint64 maxFileSize_ = 10485760;  // 10MB
};

// Factory function to create servers
std::shared_ptr<MCPServer> createMCPServer(const MCPServerConfig& config);

}  // namespace ros_weaver

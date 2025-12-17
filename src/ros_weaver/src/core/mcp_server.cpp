// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2025, ROS Weaver Contributors

#include "ros_weaver/core/mcp_server.hpp"
#include <QMutexLocker>

namespace ros_weaver {

MCPServer::MCPServer(const MCPServerConfig& config, QObject* parent)
    : QObject(parent)
    , config_(config)
    , statsTimer_(new QTimer(this))
{
    // Update stats every second
    connect(statsTimer_, &QTimer::timeout, this, &MCPServer::updateStatsTimer);
    statsTimer_->setInterval(1000);
}

MCPServer::~MCPServer() {
    stop();
}

void MCPServer::setConfig(const MCPServerConfig& config) {
    config_ = config;
}

MCPServerStats MCPServer::stats() const {
    QMutexLocker locker(&statsMutex_);
    return stats_;
}

void MCPServer::resetStats() {
    QMutexLocker locker(&statsMutex_);
    stats_ = MCPServerStats();
}

void MCPServer::start() {
    if (state_ == MCPServerState::Connected) {
        return;
    }

    setState(MCPServerState::Connecting);

    // Reset stats on start
    {
        QMutexLocker locker(&statsMutex_);
        stats_.connectedSince = QDateTime::currentDateTime();
    }

    try {
        onStart();
        setState(MCPServerState::Connected);
        statsTimer_->start();
    } catch (const std::exception& e) {
        setError(QString("Failed to start: %1").arg(e.what()));
        setState(MCPServerState::Error);
    }
}

void MCPServer::stop() {
    if (state_ == MCPServerState::Disconnected) {
        return;
    }

    statsTimer_->stop();

    try {
        onStop();
    } catch (...) {
        // Ignore errors during shutdown
    }

    setState(MCPServerState::Disconnected);
}

void MCPServer::restart() {
    stop();
    start();
}

void MCPServer::setState(MCPServerState state) {
    if (state_ != state) {
        state_ = state;
        emit stateChanged(state);
    }
}

void MCPServer::setError(const QString& error) {
    lastError_ = error;
    recordError();
    emit errorOccurred(error);
}

void MCPServer::recordBytesReceived(qint64 bytes) {
    QMutexLocker locker(&statsMutex_);
    stats_.bytesReceived += bytes;
    emit dataReceived(bytes);
}

void MCPServer::recordBytesSent(qint64 bytes) {
    QMutexLocker locker(&statsMutex_);
    stats_.bytesSent += bytes;
    emit dataSent(bytes);
}

void MCPServer::recordToolCall() {
    QMutexLocker locker(&statsMutex_);
    stats_.toolCallCount++;
}

void MCPServer::recordResourceRead() {
    QMutexLocker locker(&statsMutex_);
    stats_.resourceReadCount++;
}

void MCPServer::recordError() {
    QMutexLocker locker(&statsMutex_);
    stats_.errorCount++;
}

void MCPServer::updateStatsTimer() {
    MCPServerStats currentStats;
    {
        QMutexLocker locker(&statsMutex_);
        currentStats = stats_;
    }
    emit statsUpdated(currentStats);
}

}  // namespace ros_weaver

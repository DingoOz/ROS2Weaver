// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2025, ROS Weaver Contributors

#pragma once

#include <QWidget>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsObject>
#include <QTimer>
#include <QPropertyAnimation>
#include <memory>
#include "ros_weaver/core/mcp_types.hpp"

class QVBoxLayout;
class QHBoxLayout;
class QLabel;
class QToolButton;
class QMenu;

namespace ros_weaver {

class MCPManager;
class MCPServer;

/**
 * @brief Graphics item representing a node in the MCP star network
 */
class MCPNodeItem : public QGraphicsObject {
    Q_OBJECT
    Q_PROPERTY(qreal glowIntensity READ glowIntensity WRITE setGlowIntensity)
    Q_PROPERTY(qreal pulsePhase READ pulsePhase WRITE setPulsePhase)

public:
    enum NodeType {
        LocalAI,
        MCPServer
    };

    MCPNodeItem(NodeType type, const QString& name, const QString& description,
                QGraphicsItem* parent = nullptr);
    ~MCPNodeItem() override;

    QRectF boundingRect() const override;
    void paint(QPainter* painter, const QStyleOptionGraphicsItem* option,
               QWidget* widget) override;

    NodeType nodeType() const { return nodeType_; }
    QString name() const { return name_; }
    void setName(const QString& name);
    void setDescription(const QString& description);

    void setServerId(const QUuid& id) { serverId_ = id; }
    QUuid serverId() const { return serverId_; }

    void setState(MCPServerState state);
    MCPServerState state() const { return state_; }

    void setStats(const MCPServerStats& stats);

    // Animation properties
    qreal glowIntensity() const { return glowIntensity_; }
    void setGlowIntensity(qreal intensity);

    qreal pulsePhase() const { return pulsePhase_; }
    void setPulsePhase(qreal phase);

    // Activity indication
    void triggerActivity(MCPDataDirection direction);

    // Node size constants (public for use by connections)
    static constexpr qreal NODE_RADIUS = 40.0;
    static constexpr qreal LOCAL_AI_RADIUS = 60.0;

signals:
    void clicked(MCPNodeItem* item);
    void doubleClicked(MCPNodeItem* item);
    void contextMenuRequested(MCPNodeItem* item, const QPoint& screenPos);

protected:
    void mousePressEvent(QGraphicsSceneMouseEvent* event) override;
    void mouseDoubleClickEvent(QGraphicsSceneMouseEvent* event) override;
    void contextMenuEvent(QGraphicsSceneContextMenuEvent* event) override;
    void hoverEnterEvent(QGraphicsSceneHoverEvent* event) override;
    void hoverLeaveEvent(QGraphicsSceneHoverEvent* event) override;

private:
    QColor stateColor() const;
    void startActivityAnimation();

    NodeType nodeType_;
    QString name_;
    QString description_;
    QUuid serverId_;
    MCPServerState state_ = MCPServerState::Disconnected;
    MCPServerStats stats_;

    qreal glowIntensity_ = 0.0;
    qreal pulsePhase_ = 0.0;
    bool hovered_ = false;

    QPropertyAnimation* glowAnimation_ = nullptr;
    QPropertyAnimation* pulseAnimation_ = nullptr;
};

/**
 * @brief Graphics item representing a connection between Local AI and MCP server
 */
class MCPConnectionItem : public QGraphicsObject {
    Q_OBJECT
    Q_PROPERTY(qreal flowPhase READ flowPhase WRITE setFlowPhase)
    Q_PROPERTY(qreal activityIntensity READ activityIntensity WRITE setActivityIntensity)

public:
    MCPConnectionItem(MCPNodeItem* source, MCPNodeItem* target,
                     QGraphicsItem* parent = nullptr);
    ~MCPConnectionItem() override;

    QRectF boundingRect() const override;
    void paint(QPainter* painter, const QStyleOptionGraphicsItem* option,
               QWidget* widget) override;

    void updatePositions();

    // Animation properties
    qreal flowPhase() const { return flowPhase_; }
    void setFlowPhase(qreal phase);

    qreal activityIntensity() const { return activityIntensity_; }
    void setActivityIntensity(qreal intensity);

    // Activity indication
    void triggerDataFlow(MCPDataDirection direction, qint64 bytes);

private:
    MCPNodeItem* source_;
    MCPNodeItem* target_;
    QPointF sourcePoint_;
    QPointF targetPoint_;

    qreal flowPhase_ = 0.0;
    qreal activityIntensity_ = 0.0;
    MCPDataDirection lastDirection_ = MCPDataDirection::None;

    QPropertyAnimation* flowAnimation_ = nullptr;
    QPropertyAnimation* activityAnimation_ = nullptr;
};

/**
 * @brief Panel for visualizing MCP servers in a star network layout
 */
class MCPExplorerPanel : public QWidget {
    Q_OBJECT

public:
    explicit MCPExplorerPanel(QWidget* parent = nullptr);
    ~MCPExplorerPanel() override;

public slots:
    void refresh();

signals:
    void serverSelected(const QUuid& serverId);
    void addServerRequested();
    void serverSettingsRequested(const QUuid& serverId);

private slots:
    void onServerRegistered(const QUuid& id);
    void onServerUnregistered(const QUuid& id);
    void onServerStateChanged(const QUuid& id, MCPServerState state);
    void onServerDataActivity(const QUuid& id, MCPDataDirection direction, qint64 bytes);
    void onStatsUpdated();
    void onNodeClicked(MCPNodeItem* item);
    void onNodeDoubleClicked(MCPNodeItem* item);
    void onNodeContextMenu(MCPNodeItem* item, const QPoint& screenPos);
    void onAddServerClicked();
    void updateStatsDisplay();

private:
    void setupUi();
    void setupConnections();
    void createLocalAINode();
    void addServerNode(std::shared_ptr<MCPServer> server);
    void removeServerNode(const QUuid& id);
    void arrangeNodesInStar();
    void updateConnectionPositions();

    QVBoxLayout* mainLayout_;
    QHBoxLayout* toolbarLayout_;
    QGraphicsView* graphicsView_;
    QGraphicsScene* scene_;

    // Toolbar widgets
    QToolButton* refreshButton_;
    QToolButton* addServerButton_;
    QLabel* statsLabel_;

    // Nodes
    MCPNodeItem* localAINode_ = nullptr;
    QMap<QUuid, MCPNodeItem*> serverNodes_;
    QMap<QUuid, MCPConnectionItem*> connections_;

    // Animation timer
    QTimer* animationTimer_;

    static constexpr qreal STAR_RADIUS = 200.0;
};

}  // namespace ros_weaver

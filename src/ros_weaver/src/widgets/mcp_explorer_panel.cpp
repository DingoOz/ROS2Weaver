// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2025, ROS Weaver Contributors

#include "ros_weaver/widgets/mcp_explorer_panel.hpp"
#include "ros_weaver/core/mcp_manager.hpp"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QToolButton>
#include <QLabel>
#include <QMenu>
#include <QPainter>
#include <QGraphicsSceneMouseEvent>
#include <QGraphicsSceneContextMenuEvent>
#include <QStyleOptionGraphicsItem>
#include <QtMath>

namespace ros_weaver {

// ============================================================================
// MCPNodeItem
// ============================================================================

MCPNodeItem::MCPNodeItem(NodeType type, const QString& name, const QString& description,
                         QGraphicsItem* parent)
    : QGraphicsObject(parent)
    , nodeType_(type)
    , name_(name)
    , description_(description)
{
    setAcceptHoverEvents(true);
    setFlag(QGraphicsItem::ItemIsSelectable, type == MCPServer);

    // Setup glow animation
    glowAnimation_ = new QPropertyAnimation(this, "glowIntensity", this);
    glowAnimation_->setDuration(500);
    glowAnimation_->setEasingCurve(QEasingCurve::OutQuad);

    // Setup pulse animation for Local AI
    if (type == LocalAI) {
        pulseAnimation_ = new QPropertyAnimation(this, "pulsePhase", this);
        pulseAnimation_->setDuration(2000);
        pulseAnimation_->setStartValue(0.0);
        pulseAnimation_->setEndValue(1.0);
        pulseAnimation_->setLoopCount(-1);
        pulseAnimation_->start();
    }
}

MCPNodeItem::~MCPNodeItem() = default;

QRectF MCPNodeItem::boundingRect() const {
    qreal radius = nodeType_ == LocalAI ? LOCAL_AI_RADIUS : NODE_RADIUS;
    qreal margin = 20.0;  // For glow effect and text
    return QRectF(-radius - margin, -radius - margin,
                  2 * (radius + margin), 2 * (radius + margin) + 30);
}

void MCPNodeItem::paint(QPainter* painter, const QStyleOptionGraphicsItem* option,
                        QWidget* widget) {
    Q_UNUSED(option);
    Q_UNUSED(widget);

    painter->setRenderHint(QPainter::Antialiasing);

    qreal radius = nodeType_ == LocalAI ? LOCAL_AI_RADIUS : NODE_RADIUS;
    QColor baseColor = stateColor();

    // Draw glow effect
    if (glowIntensity_ > 0 || hovered_) {
        qreal glowRadius = radius + 15.0;
        QRadialGradient glow(0, 0, glowRadius);
        QColor glowColor = baseColor;
        glowColor.setAlphaF(0.3 * qMax(glowIntensity_, hovered_ ? 0.5 : 0.0));
        glow.setColorAt(0.6, glowColor);
        glow.setColorAt(1.0, Qt::transparent);
        painter->setBrush(glow);
        painter->setPen(Qt::NoPen);
        painter->drawEllipse(QPointF(0, 0), glowRadius, glowRadius);
    }

    // Draw pulse ring for Local AI
    if (nodeType_ == LocalAI && pulsePhase_ > 0) {
        qreal pulseRadius = radius + 10 + pulsePhase_ * 30;
        qreal pulseAlpha = (1.0 - pulsePhase_) * 0.5;
        QColor pulseColor = baseColor;
        pulseColor.setAlphaF(pulseAlpha);
        painter->setPen(QPen(pulseColor, 2));
        painter->setBrush(Qt::NoBrush);
        painter->drawEllipse(QPointF(0, 0), pulseRadius, pulseRadius);
    }

    // Draw node circle
    QRadialGradient gradient(0, -radius * 0.3, radius * 1.5);
    gradient.setColorAt(0, baseColor.lighter(130));
    gradient.setColorAt(1, baseColor.darker(110));

    painter->setBrush(gradient);
    painter->setPen(QPen(baseColor.darker(150), 2));

    if (isSelected()) {
        painter->setPen(QPen(QColor(0, 150, 255), 3));
    }

    painter->drawEllipse(QPointF(0, 0), radius, radius);

    // Draw icon/symbol
    painter->setPen(QPen(Qt::white, 2));
    if (nodeType_ == LocalAI) {
        // Draw AI symbol (brain-like icon)
        painter->setFont(QFont("Sans", 24, QFont::Bold));
        painter->drawText(QRectF(-radius, -radius, radius * 2, radius * 2),
                         Qt::AlignCenter, "AI");
    } else {
        // Draw server symbol (stacked rectangles)
        qreal iconSize = radius * 0.5;
        painter->setBrush(Qt::white);
        for (int i = 0; i < 3; ++i) {
            qreal y = -iconSize + i * (iconSize * 0.4);
            painter->drawRoundedRect(QRectF(-iconSize * 0.6, y,
                                           iconSize * 1.2, iconSize * 0.3), 2, 2);
        }
    }

    // Draw name label
    painter->setPen(Qt::white);
    QFont font("Sans", 9);
    font.setBold(true);
    painter->setFont(font);

    QRectF textRect(-radius - 10, radius + 5, (radius + 10) * 2, 25);
    QString displayName = name_;
    if (displayName.length() > 15) {
        displayName = displayName.left(12) + "...";
    }

    // Draw text background
    QFontMetrics fm(font);
    int textWidth = fm.horizontalAdvance(displayName);
    QRectF bgRect(- textWidth / 2 - 4, radius + 5, textWidth + 8, 18);
    painter->setBrush(QColor(40, 40, 40, 200));
    painter->setPen(Qt::NoPen);
    painter->drawRoundedRect(bgRect, 4, 4);

    painter->setPen(Qt::white);
    painter->drawText(textRect, Qt::AlignHCenter | Qt::AlignTop, displayName);
}

void MCPNodeItem::setName(const QString& name) {
    name_ = name;
    update();
}

void MCPNodeItem::setDescription(const QString& description) {
    description_ = description;
    setToolTip(description);
}

void MCPNodeItem::setState(MCPServerState state) {
    state_ = state;
    update();
}

void MCPNodeItem::setStats(const MCPServerStats& stats) {
    stats_ = stats;

    QString tooltip = QString("%1\n%2\n\nStatus: %3\nUptime: %4\nBytes transferred: %5\nTool calls: %6")
        .arg(name_)
        .arg(description_)
        .arg(state_ == MCPServerState::Connected ? "Connected" :
             state_ == MCPServerState::Connecting ? "Connecting" :
             state_ == MCPServerState::Error ? "Error" : "Disconnected")
        .arg(stats.uptime())
        .arg(stats.totalBytesTransferred())
        .arg(stats.toolCallCount);

    setToolTip(tooltip);
}

void MCPNodeItem::setGlowIntensity(qreal intensity) {
    glowIntensity_ = intensity;
    update();
}

void MCPNodeItem::setPulsePhase(qreal phase) {
    pulsePhase_ = phase;
    update();
}

void MCPNodeItem::triggerActivity(MCPDataDirection direction) {
    Q_UNUSED(direction);
    startActivityAnimation();
}

QColor MCPNodeItem::stateColor() const {
    if (nodeType_ == LocalAI) {
        return QColor(100, 100, 220);  // Blue for AI
    }

    switch (state_) {
        case MCPServerState::Connected:
            return QColor(80, 180, 80);   // Green
        case MCPServerState::Connecting:
            return QColor(220, 180, 50);  // Yellow
        case MCPServerState::Error:
            return QColor(200, 80, 80);   // Red
        case MCPServerState::Disconnected:
        default:
            return QColor(120, 120, 120); // Gray
    }
}

void MCPNodeItem::startActivityAnimation() {
    glowAnimation_->stop();
    glowAnimation_->setStartValue(1.0);
    glowAnimation_->setEndValue(0.0);
    glowAnimation_->start();
}

void MCPNodeItem::mousePressEvent(QGraphicsSceneMouseEvent* event) {
    if (event->button() == Qt::LeftButton) {
        emit clicked(this);
    }
    QGraphicsObject::mousePressEvent(event);
}

void MCPNodeItem::mouseDoubleClickEvent(QGraphicsSceneMouseEvent* event) {
    if (event->button() == Qt::LeftButton) {
        emit doubleClicked(this);
    }
    QGraphicsObject::mouseDoubleClickEvent(event);
}

void MCPNodeItem::contextMenuEvent(QGraphicsSceneContextMenuEvent* event) {
    emit contextMenuRequested(this, event->screenPos());
}

void MCPNodeItem::hoverEnterEvent(QGraphicsSceneHoverEvent* event) {
    hovered_ = true;
    update();
    QGraphicsObject::hoverEnterEvent(event);
}

void MCPNodeItem::hoverLeaveEvent(QGraphicsSceneHoverEvent* event) {
    hovered_ = false;
    update();
    QGraphicsObject::hoverLeaveEvent(event);
}

// ============================================================================
// MCPConnectionItem
// ============================================================================

MCPConnectionItem::MCPConnectionItem(MCPNodeItem* source, MCPNodeItem* target,
                                     QGraphicsItem* parent)
    : QGraphicsObject(parent)
    , source_(source)
    , target_(target)
{
    setZValue(-1);  // Behind nodes

    // Setup flow animation
    flowAnimation_ = new QPropertyAnimation(this, "flowPhase", this);
    flowAnimation_->setDuration(1000);
    flowAnimation_->setStartValue(0.0);
    flowAnimation_->setEndValue(1.0);
    flowAnimation_->setLoopCount(-1);

    // Setup activity animation
    activityAnimation_ = new QPropertyAnimation(this, "activityIntensity", this);
    activityAnimation_->setDuration(500);
    activityAnimation_->setEasingCurve(QEasingCurve::OutQuad);

    updatePositions();
}

MCPConnectionItem::~MCPConnectionItem() = default;

QRectF MCPConnectionItem::boundingRect() const {
    qreal margin = 20.0;
    return QRectF(qMin(sourcePoint_.x(), targetPoint_.x()) - margin,
                  qMin(sourcePoint_.y(), targetPoint_.y()) - margin,
                  qAbs(targetPoint_.x() - sourcePoint_.x()) + 2 * margin,
                  qAbs(targetPoint_.y() - sourcePoint_.y()) + 2 * margin);
}

void MCPConnectionItem::paint(QPainter* painter, const QStyleOptionGraphicsItem* option,
                              QWidget* widget) {
    Q_UNUSED(option);
    Q_UNUSED(widget);

    painter->setRenderHint(QPainter::Antialiasing);

    // Determine line color based on target state
    QColor lineColor(100, 100, 100);
    if (target_ && target_->state() == MCPServerState::Connected) {
        lineColor = QColor(80, 150, 80);
    }

    // Draw activity glow
    if (activityIntensity_ > 0) {
        QColor glowColor = lineColor;
        glowColor.setAlphaF(activityIntensity_ * 0.5);
        QPen glowPen(glowColor, 8 + activityIntensity_ * 4);
        glowPen.setCapStyle(Qt::RoundCap);
        painter->setPen(glowPen);
        painter->drawLine(sourcePoint_, targetPoint_);
    }

    // Draw main line
    QPen pen(lineColor, 2);
    pen.setCapStyle(Qt::RoundCap);

    // Animated dashes if connected
    if (target_ && target_->state() == MCPServerState::Connected) {
        QVector<qreal> dashes;
        dashes << 8 << 4;
        pen.setDashPattern(dashes);
        pen.setDashOffset(flowPhase_ * 12);  // Animate
        flowAnimation_->start();
    } else {
        flowAnimation_->stop();
    }

    painter->setPen(pen);
    painter->drawLine(sourcePoint_, targetPoint_);

    // Draw direction arrow if there's activity
    if (activityIntensity_ > 0.3 && lastDirection_ != MCPDataDirection::None) {
        // Calculate arrow position (middle of line)
        QPointF mid = (sourcePoint_ + targetPoint_) / 2;

        // Calculate arrow direction
        QLineF line(sourcePoint_, targetPoint_);
        if (lastDirection_ == MCPDataDirection::Sending) {
            line = QLineF(targetPoint_, sourcePoint_);  // Reverse for sending
        }

        qreal angle = std::atan2(-line.dy(), line.dx());

        // Draw arrow
        painter->setBrush(lineColor);
        painter->setPen(Qt::NoPen);

        QPolygonF arrow;
        qreal arrowSize = 10 * activityIntensity_;
        arrow << QPointF(mid.x() + arrowSize * std::cos(angle),
                        mid.y() - arrowSize * std::sin(angle))
              << QPointF(mid.x() + arrowSize * std::cos(angle + 2.5),
                        mid.y() - arrowSize * std::sin(angle + 2.5))
              << QPointF(mid.x() + arrowSize * std::cos(angle - 2.5),
                        mid.y() - arrowSize * std::sin(angle - 2.5));
        painter->drawPolygon(arrow);
    }
}

void MCPConnectionItem::updatePositions() {
    prepareGeometryChange();

    if (source_ && target_) {
        sourcePoint_ = source_->scenePos();
        targetPoint_ = target_->scenePos();

        // Adjust to edge of nodes
        QLineF line(sourcePoint_, targetPoint_);
        qreal sourceRadius = source_->nodeType() == MCPNodeItem::LocalAI ?
            MCPNodeItem::LOCAL_AI_RADIUS : MCPNodeItem::NODE_RADIUS;
        qreal targetRadius = target_->nodeType() == MCPNodeItem::LocalAI ?
            MCPNodeItem::LOCAL_AI_RADIUS : MCPNodeItem::NODE_RADIUS;

        line.setLength(line.length() - targetRadius - 5);
        targetPoint_ = line.p2();

        QLineF reverseLine(targetPoint_, sourcePoint_);
        reverseLine.setLength(reverseLine.length() - sourceRadius - 5);
        sourcePoint_ = reverseLine.p2();
    }

    update();
}

void MCPConnectionItem::setFlowPhase(qreal phase) {
    flowPhase_ = phase;
    update();
}

void MCPConnectionItem::setActivityIntensity(qreal intensity) {
    activityIntensity_ = intensity;
    update();
}

void MCPConnectionItem::triggerDataFlow(MCPDataDirection direction, qint64 bytes) {
    Q_UNUSED(bytes);
    lastDirection_ = direction;

    activityAnimation_->stop();
    activityAnimation_->setStartValue(1.0);
    activityAnimation_->setEndValue(0.0);
    activityAnimation_->start();
}

// ============================================================================
// MCPExplorerPanel
// ============================================================================

MCPExplorerPanel::MCPExplorerPanel(QWidget* parent)
    : QWidget(parent)
    , animationTimer_(new QTimer(this))
{
    setupUi();
    setupConnections();

    // Initial refresh
    QTimer::singleShot(100, this, &MCPExplorerPanel::refresh);
}

MCPExplorerPanel::~MCPExplorerPanel() = default;

void MCPExplorerPanel::setupUi() {
    mainLayout_ = new QVBoxLayout(this);
    mainLayout_->setContentsMargins(4, 4, 4, 4);
    mainLayout_->setSpacing(4);

    // Toolbar
    toolbarLayout_ = new QHBoxLayout();
    toolbarLayout_->setSpacing(4);

    refreshButton_ = new QToolButton(this);
    refreshButton_->setIcon(QIcon::fromTheme("view-refresh"));
    refreshButton_->setToolTip("Refresh MCP servers");

    addServerButton_ = new QToolButton(this);
    addServerButton_->setIcon(QIcon::fromTheme("list-add"));
    addServerButton_->setToolTip("Add new MCP server");

    statsLabel_ = new QLabel(this);
    statsLabel_->setStyleSheet("color: #888; font-size: 10px;");

    toolbarLayout_->addWidget(refreshButton_);
    toolbarLayout_->addWidget(addServerButton_);
    toolbarLayout_->addStretch();
    toolbarLayout_->addWidget(statsLabel_);

    mainLayout_->addLayout(toolbarLayout_);

    // Graphics view
    scene_ = new QGraphicsScene(this);
    scene_->setBackgroundBrush(QColor(30, 30, 35));

    graphicsView_ = new QGraphicsView(scene_, this);
    graphicsView_->setRenderHint(QPainter::Antialiasing);
    graphicsView_->setViewportUpdateMode(QGraphicsView::SmartViewportUpdate);
    graphicsView_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    graphicsView_->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    graphicsView_->setDragMode(QGraphicsView::ScrollHandDrag);
    graphicsView_->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);

    mainLayout_->addWidget(graphicsView_, 1);

    // Create the central Local AI node
    createLocalAINode();

    // Animation timer for position updates
    connect(animationTimer_, &QTimer::timeout, this, &MCPExplorerPanel::updateConnectionPositions);
    animationTimer_->start(50);
}

void MCPExplorerPanel::setupConnections() {
    connect(refreshButton_, &QToolButton::clicked, this, &MCPExplorerPanel::refresh);
    connect(addServerButton_, &QToolButton::clicked, this, &MCPExplorerPanel::onAddServerClicked);

    // Connect to MCPManager
    MCPManager& manager = MCPManager::instance();
    connect(&manager, &MCPManager::serverRegistered,
            this, &MCPExplorerPanel::onServerRegistered);
    connect(&manager, &MCPManager::serverUnregistered,
            this, &MCPExplorerPanel::onServerUnregistered);
    connect(&manager, &MCPManager::serverStateChanged,
            this, &MCPExplorerPanel::onServerStateChanged);
    connect(&manager, &MCPManager::serverDataActivity,
            this, &MCPExplorerPanel::onServerDataActivity);
    connect(&manager, &MCPManager::allStatsUpdated,
            this, &MCPExplorerPanel::onStatsUpdated);
}

void MCPExplorerPanel::createLocalAINode() {
    localAINode_ = new MCPNodeItem(MCPNodeItem::LocalAI, "Local AI",
                                   "Ollama-based Local AI Assistant");
    localAINode_->setPos(0, 0);
    scene_->addItem(localAINode_);

    connect(localAINode_, &MCPNodeItem::clicked, this, &MCPExplorerPanel::onNodeClicked);
    connect(localAINode_, &MCPNodeItem::doubleClicked, this, &MCPExplorerPanel::onNodeDoubleClicked);
    connect(localAINode_, &MCPNodeItem::contextMenuRequested,
            this, &MCPExplorerPanel::onNodeContextMenu);
}

void MCPExplorerPanel::addServerNode(std::shared_ptr<MCPServer> server) {
    if (!server || serverNodes_.contains(server->id())) {
        return;
    }

    auto* node = new MCPNodeItem(MCPNodeItem::MCPServer,
                                 server->name(),
                                 server->description());
    node->setServerId(server->id());
    node->setState(server->state());
    node->setStats(server->stats());

    scene_->addItem(node);
    serverNodes_[server->id()] = node;

    connect(node, &MCPNodeItem::clicked, this, &MCPExplorerPanel::onNodeClicked);
    connect(node, &MCPNodeItem::doubleClicked, this, &MCPExplorerPanel::onNodeDoubleClicked);
    connect(node, &MCPNodeItem::contextMenuRequested, this, &MCPExplorerPanel::onNodeContextMenu);

    // Create connection to Local AI
    auto* connection = new MCPConnectionItem(localAINode_, node);
    scene_->addItem(connection);
    connections_[server->id()] = connection;

    arrangeNodesInStar();
}

void MCPExplorerPanel::removeServerNode(const QUuid& id) {
    if (serverNodes_.contains(id)) {
        auto* node = serverNodes_.take(id);
        scene_->removeItem(node);
        delete node;
    }

    if (connections_.contains(id)) {
        auto* conn = connections_.take(id);
        scene_->removeItem(conn);
        delete conn;
    }

    arrangeNodesInStar();
}

void MCPExplorerPanel::arrangeNodesInStar() {
    int count = serverNodes_.size();
    if (count == 0) return;

    // Arrange nodes in a circle around the Local AI
    qreal angleStep = 2 * M_PI / count;
    qreal currentAngle = -M_PI / 2;  // Start at top

    for (auto it = serverNodes_.begin(); it != serverNodes_.end(); ++it) {
        qreal x = STAR_RADIUS * std::cos(currentAngle);
        qreal y = STAR_RADIUS * std::sin(currentAngle);
        (*it)->setPos(x, y);
        currentAngle += angleStep;
    }

    updateConnectionPositions();

    // Fit view to content
    QRectF bounds = scene_->itemsBoundingRect().adjusted(-50, -50, 50, 50);
    graphicsView_->fitInView(bounds, Qt::KeepAspectRatio);
}

void MCPExplorerPanel::updateConnectionPositions() {
    for (auto* conn : connections_) {
        conn->updatePositions();
    }
}

void MCPExplorerPanel::refresh() {
    MCPManager& manager = MCPManager::instance();

    // Remove nodes for servers that no longer exist
    QList<QUuid> existingIds;
    for (auto server : manager.allServers()) {
        existingIds.append(server->id());
    }

    for (const QUuid& id : serverNodes_.keys()) {
        if (!existingIds.contains(id)) {
            removeServerNode(id);
        }
    }

    // Add nodes for new servers
    for (auto server : manager.allServers()) {
        if (!serverNodes_.contains(server->id())) {
            addServerNode(server);
        }
    }

    updateStatsDisplay();
}

void MCPExplorerPanel::onServerRegistered(const QUuid& id) {
    MCPManager& manager = MCPManager::instance();
    if (auto server = manager.server(id)) {
        addServerNode(server);
    }
}

void MCPExplorerPanel::onServerUnregistered(const QUuid& id) {
    removeServerNode(id);
}

void MCPExplorerPanel::onServerStateChanged(const QUuid& id, MCPServerState state) {
    if (serverNodes_.contains(id)) {
        serverNodes_[id]->setState(state);
    }
}

void MCPExplorerPanel::onServerDataActivity(const QUuid& id, MCPDataDirection direction,
                                            qint64 bytes) {
    if (serverNodes_.contains(id)) {
        serverNodes_[id]->triggerActivity(direction);
    }
    if (connections_.contains(id)) {
        connections_[id]->triggerDataFlow(direction, bytes);
    }
}

void MCPExplorerPanel::onStatsUpdated() {
    MCPManager& manager = MCPManager::instance();

    for (auto server : manager.allServers()) {
        if (serverNodes_.contains(server->id())) {
            serverNodes_[server->id()]->setStats(server->stats());
        }
    }

    updateStatsDisplay();
}

void MCPExplorerPanel::onNodeClicked(MCPNodeItem* item) {
    if (item && item->nodeType() == MCPNodeItem::MCPServer) {
        emit serverSelected(item->serverId());
    }
}

void MCPExplorerPanel::onNodeDoubleClicked(MCPNodeItem* item) {
    if (item && item->nodeType() == MCPNodeItem::MCPServer) {
        emit serverSettingsRequested(item->serverId());
    }
}

void MCPExplorerPanel::onNodeContextMenu(MCPNodeItem* item, const QPoint& screenPos) {
    QMenu menu;

    if (item->nodeType() == MCPNodeItem::LocalAI) {
        menu.addAction("Configure Local AI...");
        menu.addSeparator();
        menu.addAction("Add MCP Server...", this, &MCPExplorerPanel::onAddServerClicked);
    } else {
        MCPManager& manager = MCPManager::instance();
        auto server = manager.server(item->serverId());

        if (server) {
            if (server->isConnected()) {
                menu.addAction("Stop Server", [this, id = item->serverId()]() {
                    MCPManager::instance().stopServer(id);
                });
            } else {
                menu.addAction("Start Server", [this, id = item->serverId()]() {
                    MCPManager::instance().startServer(id);
                });
            }
            menu.addSeparator();
            menu.addAction("Settings...", [this, item]() {
                emit serverSettingsRequested(item->serverId());
            });
            menu.addSeparator();
            menu.addAction("Remove Server", [this, id = item->serverId()]() {
                MCPManager::instance().unregisterServer(id);
            });
        }
    }

    menu.exec(screenPos);
}

void MCPExplorerPanel::onAddServerClicked() {
    emit addServerRequested();
}

// Helper function for byte formatting
static QString formatBytes(qint64 bytes) {
    if (bytes < 1024) return QString("%1 B").arg(bytes);
    if (bytes < 1024 * 1024) return QString("%1 KB").arg(bytes / 1024.0, 0, 'f', 1);
    if (bytes < 1024 * 1024 * 1024) return QString("%1 MB").arg(bytes / (1024.0 * 1024.0), 0, 'f', 1);
    return QString("%1 GB").arg(bytes / (1024.0 * 1024.0 * 1024.0), 0, 'f', 2);
}

void MCPExplorerPanel::updateStatsDisplay() {
    MCPManager& manager = MCPManager::instance();
    auto stats = manager.aggregateStats();

    QString text = QString("Active: %1/%2 | Transferred: %3 | Tools: %4")
        .arg(manager.activeServers().size())
        .arg(manager.allServers().size())
        .arg(formatBytes(stats.totalBytesTransferred()))
        .arg(stats.toolCallCount);

    statsLabel_->setText(text);
}

}  // namespace ros_weaver

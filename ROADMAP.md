# ROS2Weaver Feature Ideas & Roadmap

These are community and AI-suggested features to enhance ROS2Weaver, prioritized by potential impact and feasibility. Contributions are welcome!

## Implementation Status

### Completed Features (v1.1.0)

| Feature | Status | Notes |
|---------|--------|-------|
| Context-Sensitive Help | ✅ Done | F1 key, help browser, context tooltips |
| Inline Tooltips & Schema Viewer | ✅ Done | Message schema viewer panel |
| README Preview Panel | ✅ Done | Dockable panel for package docs |
| Export to PlantUML/Mermaid/Graphviz | ✅ Done | File → Export menu |
| Import from rqt_graph DOT | ✅ Done | File → Import menu |
| Import from CARET YAML | ✅ Done | File → Import menu |
| Project Save/Load (.weaver) | ✅ Done | JSON/YAML project files |
| One-click Gazebo/Ignition Launch | ✅ Done | Simulation menu |
| Rosbag Recording/Playback | ✅ Done | Rosbag Workbench panel |
| Static Analysis | ✅ Done | Tools → Run Analysis |
| Parameter Dashboard | ✅ Done | Properties panel |
| Undo/Redo Stack | ✅ Done | Edit menu, Ctrl+Z/Y |
| Architecture Diff View | ✅ Done | Compare project versions |
| Multi-canvas Tabs | ✅ Done | Tab-based canvas management |
| Canvas Grid Visualization | ✅ Done | View menu, snap-to-grid |
| Mini-map Navigation | ✅ Done | Dockable minimap panel |
| Auto-layout Algorithms | ✅ Done | Hierarchical, force-directed, circular, grid |
| Canvas Annotations | ✅ Done | Right-click → Add Annotation |
| Connection Bandwidth Visualization | ✅ Done | Live message rates on connections |
| Time-synced Playback | ✅ Done | Canvas highlights during bag playback |
| Node Health Dashboard | ✅ Done | CPU, memory, latency monitoring |
| QoS Profile Editor | ✅ Done | Visual QoS configuration |
| Launch File Generator | ✅ Done | Code generation wizard |
| Parameter Presets | ✅ Done | Save/load parameter configurations |
| Node Templates Library | ✅ Done | Pre-built node patterns |
| Workspace Browser Panel | ✅ Done | Browse ROS2 workspace packages |
| Natural Language Architecture Generation | ✅ Done | AI-powered architecture from description |
| Automated Error Fixing Suggestions | ✅ Done | AI suggestions via "Ask AI" button |
| Architecture Optimization Recommendations | ✅ Done | Tools → Optimize Architecture |
| Advanced Node Library Search | ✅ Done | Search tab with filters |
| Improved Namespace/Remapping Visualization | ✅ Done | Visual namespace badges and remapping indicators |

### Completed Features (v1.2.0)

| Feature | Status | Notes |
|---------|--------|-------|
| Live Message Inspector & Publisher | Done | View live messages, publish test messages |
| Node Lifecycle State Visualization | Done | Visual state machine, transition controls |
| Remote Robot Connection | Done | SSH tunnel, DDS configuration, profile management |

### Completed Features (v1.3.0)

| Feature | Status | Notes |
|---------|--------|-------|
| Topic Latency Heatmap | Done | Color-coded latency visualization, auto-tune thresholds, historical graph, CSV export |
| Test Message Scenario Editor | Done | Create/replay test message sequences, pytest export, recording from live system |

### Completed Features (v1.4.0)

| Feature | Status | Notes |
|---------|--------|-------|
| Architecture Documentation Generator | Done | Markdown/HTML/PDF export, Mermaid/PlantUML diagrams, configurable sections |

### Remaining Features

| Feature | Priority | Complexity |
|---------|----------|------------|
| ros2 doctor Integration | Medium | Low |
| Message Schema Diff Tool | Medium | Low |
| Visual Behavior Tree Editor | Low | High |
| Plugin/Extension System | Low | High |
| CI/CD Pipeline Generator | Low | Medium |
| DDS Network Topology View | Low | Medium |
| Embedded mini-RViz / 3D view | Low | High |
| Multi-user collaboration | Low | High |

---

## Contributing Guidelines

### Branch Naming Convention
When implementing a feature from this roadmap, create a new branch using the following convention:

```bash
git checkout -b feature/<feature-name>
```

Examples:
- `feature/mini-rviz-view`
- `feature/ai-architecture-generation`
- `feature/multi-user-collab`

### Workflow
1. Create a new branch from `main` for each feature.
2. Implement the feature with appropriate tests.
3. Submit a pull request back to `main`.
4. Reference the relevant roadmap item in the PR description.

This keeps features isolated and makes code review easier.

---

## Feature Details

### Remaining: Embedded mini-RViz / 3D View
- Overlay live robot model and TF data
- Visualize sensor data in 3D
- Integration with existing TF Tree panel
- Would require significant Qt3D or RViz library integration

### Remaining: Multi-user Collaboration
- Real-time collaborative editing
- Presence indicators
- Conflict resolution
- Would require WebSocket server infrastructure

---

## Detailed Implementation Plans for New Features

### HIGH PRIORITY

---

### Live Message Inspector & Publisher

**Branch:** `feature/live-message-inspector`

**Overview:** A dockable panel that allows users to inspect live messages on any topic and publish custom messages for testing.

#### New Files to Create

```
src/ros_weaver/include/ros_weaver/widgets/message_inspector_panel.hpp
src/ros_weaver/src/widgets/message_inspector_panel.cpp
src/ros_weaver/include/ros_weaver/core/generic_message_handler.hpp
src/ros_weaver/src/core/generic_message_handler.cpp
src/ros_weaver/include/ros_weaver/widgets/message_editor_widget.hpp
src/ros_weaver/src/widgets/message_editor_widget.cpp
```

#### Implementation Steps

**Step 1: Create GenericMessageHandler class**

File: `src/ros_weaver/include/ros_weaver/core/generic_message_handler.hpp`

```cpp
class GenericMessageHandler : public QObject {
  Q_OBJECT
public:
  explicit GenericMessageHandler(rclcpp::Node::SharedPtr node);

  // Subscribe to any topic dynamically
  void subscribeToTopic(const QString& topicName, const QString& messageType);
  void unsubscribeFromTopic(const QString& topicName);

  // Publish to any topic
  void publishMessage(const QString& topicName, const QString& messageType,
                      const QJsonObject& messageData);

  // Get message type info
  QJsonObject getMessageSchema(const QString& messageType);

signals:
  void messageReceived(const QString& topicName, const QJsonObject& message,
                       qint64 timestamp);
  void subscriptionError(const QString& topicName, const QString& error);

private:
  rclcpp::Node::SharedPtr node_;
  QMap<QString, rclcpp::GenericSubscription::SharedPtr> subscriptions_;
  QMap<QString, rclcpp::GenericPublisher::SharedPtr> publishers_;

  // Use rosidl_typesupport for dynamic message handling
  QJsonObject deserializeMessage(const rclcpp::SerializedMessage& msg,
                                  const QString& messageType);
  rclcpp::SerializedMessage serializeMessage(const QJsonObject& json,
                                              const QString& messageType);
};
```

**Step 2: Create MessageInspectorPanel widget**

File: `src/ros_weaver/include/ros_weaver/widgets/message_inspector_panel.hpp`

```cpp
class MessageInspectorPanel : public QWidget {
  Q_OBJECT
public:
  explicit MessageInspectorPanel(QWidget* parent = nullptr);

  void setRosNode(rclcpp::Node::SharedPtr node);

public slots:
  void inspectTopic(const QString& topicName, const QString& messageType);
  void stopInspecting();

private:
  void setupUi();
  void setupConnections();

  // UI Components
  QComboBox* topicSelector_;           // Dropdown of available topics
  QPushButton* refreshTopicsButton_;
  QTreeWidget* messageTreeView_;       // Expandable message fields
  QSpinBox* bufferSizeSpinBox_;        // Keep last N messages
  QListWidget* messageHistoryList_;    // Clickable history
  QPushButton* pauseButton_;
  QPushButton* clearButton_;
  QLabel* rateLabel_;                  // Current message rate
  QLabel* lastReceivedLabel_;          // Timestamp of last message

  // Publisher section
  QGroupBox* publisherGroup_;
  QTextEdit* jsonEditor_;              // JSON message editor
  QPushButton* publishButton_;
  QPushButton* loadSchemaButton_;      // Load default schema

  // State
  GenericMessageHandler* messageHandler_;
  QList<QPair<qint64, QJsonObject>> messageBuffer_;
  int maxBufferSize_ = 100;
  bool isPaused_ = false;

private slots:
  void onTopicSelected(int index);
  void onMessageReceived(const QString& topic, const QJsonObject& msg, qint64 ts);
  void onPublishClicked();
  void populateTopicList();
};
```

**Step 3: Implement message tree visualization**

Create a recursive function to display nested message fields:

```cpp
void MessageInspectorPanel::populateTreeFromJson(QTreeWidgetItem* parent,
                                                   const QJsonObject& obj) {
  for (auto it = obj.begin(); it != obj.end(); ++it) {
    QTreeWidgetItem* item = new QTreeWidgetItem();
    item->setText(0, it.key());  // Field name

    if (it.value().isObject()) {
      item->setText(1, "{...}");
      populateTreeFromJson(item, it.value().toObject());
    } else if (it.value().isArray()) {
      QJsonArray arr = it.value().toArray();
      item->setText(1, QString("[%1 items]").arg(arr.size()));
      // Add array items as children
      for (int i = 0; i < arr.size(); ++i) {
        QTreeWidgetItem* arrItem = new QTreeWidgetItem();
        arrItem->setText(0, QString("[%1]").arg(i));
        if (arr[i].isObject()) {
          populateTreeFromJson(arrItem, arr[i].toObject());
        } else {
          arrItem->setText(1, arr[i].toVariant().toString());
        }
        item->addChild(arrItem);
      }
    } else {
      item->setText(1, it.value().toVariant().toString());
      item->setText(2, getJsonTypeName(it.value()));  // Type column
    }

    if (parent) {
      parent->addChild(item);
    } else {
      messageTreeView_->addTopLevelItem(item);
    }
  }
}
```

**Step 4: Add JSON message editor with schema validation**

```cpp
void MessageInspectorPanel::onLoadSchemaClicked() {
  QString messageType = getCurrentMessageType();
  QJsonObject schema = messageHandler_->getMessageSchema(messageType);

  // Generate template JSON from schema
  QJsonObject template_ = generateTemplateFromSchema(schema);

  // Pretty print to editor
  QJsonDocument doc(template_);
  jsonEditor_->setPlainText(doc.toJson(QJsonDocument::Indented));
}

void MessageInspectorPanel::onPublishClicked() {
  QString json = jsonEditor_->toPlainText();
  QJsonParseError error;
  QJsonDocument doc = QJsonDocument::fromJson(json.toUtf8(), &error);

  if (error.error != QJsonParseError::NoError) {
    QMessageBox::warning(this, tr("Invalid JSON"),
                         tr("Parse error: %1").arg(error.errorString()));
    return;
  }

  QString topic = topicSelector_->currentData().toString();
  QString msgType = topicSelector_->currentData(Qt::UserRole + 1).toString();

  messageHandler_->publishMessage(topic, msgType, doc.object());
}
```

**Step 5: Integration with MainWindow**

In `main_window.cpp`:

```cpp
void MainWindow::createMessageInspectorPanel() {
  messageInspectorPanel_ = new MessageInspectorPanel(this);
  messageInspectorDock_ = new QDockWidget(tr("Message Inspector"), this);
  messageInspectorDock_->setWidget(messageInspectorPanel_);
  addDockWidget(Qt::RightDockWidgetArea, messageInspectorDock_);

  // Connect to canvas selection
  connect(canvas_, &WeaverCanvas::connectionSelected,
          this, [this](ConnectionLine* conn) {
    if (conn && !conn->topicName().isEmpty()) {
      messageInspectorPanel_->inspectTopic(conn->topicName(),
                                            conn->messageType());
    }
  });
}
```

**Step 6: Add to CMakeLists.txt**

```cmake
set(WIDGET_SOURCES
  # ... existing sources ...
  src/widgets/message_inspector_panel.cpp
  src/widgets/message_editor_widget.cpp
)

set(CORE_SOURCES
  # ... existing sources ...
  src/core/generic_message_handler.cpp
)
```

#### Testing

Create test file `test/test_message_inspector.cpp`:

```cpp
TEST(MessageInspectorTest, JsonSerialization) {
  // Test message serialization/deserialization
}

TEST(MessageInspectorTest, SchemaGeneration) {
  // Test schema generation for common message types
}

TEST(MessageInspectorTest, BufferManagement) {
  // Test message buffer with size limits
}
```

---

### Node Lifecycle State Visualization

**Branch:** `feature/lifecycle-visualization`

**Overview:** Visual representation of ROS2 lifecycle node states with interactive state transition controls.

#### New Files to Create

```
src/ros_weaver/include/ros_weaver/widgets/lifecycle_state_widget.hpp
src/ros_weaver/src/widgets/lifecycle_state_widget.cpp
src/ros_weaver/include/ros_weaver/core/lifecycle_manager.hpp
src/ros_weaver/src/core/lifecycle_manager.cpp
```

#### Implementation Steps

**Step 1: Create LifecycleManager class**

File: `src/ros_weaver/include/ros_weaver/core/lifecycle_manager.hpp`

```cpp
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <lifecycle_msgs/msg/transition_event.hpp>

enum class LifecycleState {
  Unknown = 0,
  Unconfigured = 1,
  Inactive = 2,
  Active = 3,
  Finalized = 4,
  // Transitioning states
  Configuring = 10,
  CleaningUp = 11,
  ShuttingDown = 12,
  Activating = 13,
  Deactivating = 14,
  ErrorProcessing = 15
};

struct LifecycleNodeInfo {
  QString nodeName;
  LifecycleState currentState;
  QDateTime lastStateChange;
  QStringList availableTransitions;
};

class LifecycleManager : public QObject {
  Q_OBJECT
public:
  explicit LifecycleManager(rclcpp::Node::SharedPtr node);

  // Query lifecycle nodes
  QStringList discoverLifecycleNodes();
  LifecycleState getNodeState(const QString& nodeName);
  QStringList getAvailableTransitions(const QString& nodeName);

  // Trigger transitions
  void requestTransition(const QString& nodeName, const QString& transition);

  // Batch operations
  void configureAll(const QStringList& nodes);
  void activateAll(const QStringList& nodes);
  void deactivateAll(const QStringList& nodes);

signals:
  void stateChanged(const QString& nodeName, LifecycleState newState);
  void transitionFailed(const QString& nodeName, const QString& error);
  void lifecycleNodesDiscovered(const QStringList& nodes);

private:
  rclcpp::Node::SharedPtr node_;
  QMap<QString, rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr> changeStateClients_;
  QMap<QString, rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr> getStateClients_;
  QMap<QString, rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>::SharedPtr> transitionSubs_;

  void subscribeToTransitionEvents(const QString& nodeName);
};
```

**Step 2: Create LifecycleStateWidget for visual display**

File: `src/ros_weaver/include/ros_weaver/widgets/lifecycle_state_widget.hpp`

```cpp
class LifecycleStateWidget : public QWidget {
  Q_OBJECT
public:
  explicit LifecycleStateWidget(QWidget* parent = nullptr);

  void setLifecycleManager(LifecycleManager* manager);
  void setNodeName(const QString& nodeName);

protected:
  void paintEvent(QPaintEvent* event) override;

private:
  void setupUi();
  void drawStateMachine(QPainter* painter);
  void updateButtonStates();

  // State machine visualization
  struct StateNode {
    QString name;
    QRectF rect;
    QColor color;
    bool isCurrent;
  };
  QList<StateNode> stateNodes_;

  // UI
  QLabel* nodeNameLabel_;
  QLabel* currentStateLabel_;
  QPushButton* configureButton_;
  QPushButton* activateButton_;
  QPushButton* deactivateButton_;
  QPushButton* cleanupButton_;
  QPushButton* shutdownButton_;
  QLabel* historyLabel_;

  LifecycleManager* manager_;
  QString currentNodeName_;
  LifecycleState currentState_;

private slots:
  void onConfigureClicked();
  void onActivateClicked();
  void onDeactivateClicked();
  void onStateChanged(const QString& nodeName, LifecycleState state);
};
```

**Step 3: Draw state machine diagram**

```cpp
void LifecycleStateWidget::drawStateMachine(QPainter* painter) {
  painter->setRenderHint(QPainter::Antialiasing);

  // Define state positions (horizontal layout)
  QMap<LifecycleState, QPointF> statePositions = {
    {LifecycleState::Unconfigured, QPointF(50, 100)},
    {LifecycleState::Inactive, QPointF(200, 100)},
    {LifecycleState::Active, QPointF(350, 100)},
    {LifecycleState::Finalized, QPointF(200, 200)}
  };

  QMap<LifecycleState, QColor> stateColors = {
    {LifecycleState::Unconfigured, QColor(150, 150, 150)},  // Gray
    {LifecycleState::Inactive, QColor(255, 200, 50)},       // Yellow
    {LifecycleState::Active, QColor(100, 200, 100)},        // Green
    {LifecycleState::Finalized, QColor(200, 100, 100)}      // Red
  };

  // Draw transitions (arrows)
  painter->setPen(QPen(QColor(100, 100, 100), 2));
  drawArrow(painter, statePositions[LifecycleState::Unconfigured],
            statePositions[LifecycleState::Inactive], "configure");
  drawArrow(painter, statePositions[LifecycleState::Inactive],
            statePositions[LifecycleState::Active], "activate");
  // ... more transitions

  // Draw state circles
  for (auto it = statePositions.begin(); it != statePositions.end(); ++it) {
    QColor color = stateColors[it.key()];
    bool isCurrent = (it.key() == currentState_);

    // Highlight current state
    if (isCurrent) {
      painter->setBrush(color);
      painter->setPen(QPen(color.darker(120), 3));
    } else {
      painter->setBrush(color.lighter(150));
      painter->setPen(QPen(color, 1));
    }

    QRectF rect(it.value().x() - 30, it.value().y() - 30, 60, 60);
    painter->drawEllipse(rect);

    // Draw state name
    painter->setPen(Qt::black);
    painter->drawText(rect, Qt::AlignCenter, lifecycleStateToString(it.key()));
  }
}
```

**Step 4: Add lifecycle badge to PackageBlock**

Modify `package_block.cpp` to show lifecycle state:

```cpp
void PackageBlock::paintLifecycleBadge(QPainter* painter) {
  if (!isLifecycleNode_ || lifecycleState_ == LifecycleState::Unknown) {
    return;
  }

  // Draw badge in top-right corner
  QRectF badgeRect(BLOCK_WIDTH - 24, 4, 20, 20);

  QColor stateColor;
  QString stateChar;
  switch (lifecycleState_) {
    case LifecycleState::Unconfigured:
      stateColor = QColor(150, 150, 150);
      stateChar = "U";
      break;
    case LifecycleState::Inactive:
      stateColor = QColor(255, 200, 50);
      stateChar = "I";
      break;
    case LifecycleState::Active:
      stateColor = QColor(100, 200, 100);
      stateChar = "A";
      break;
    case LifecycleState::Finalized:
      stateColor = QColor(200, 100, 100);
      stateChar = "F";
      break;
    default:
      stateColor = QColor(100, 100, 100);
      stateChar = "?";
  }

  painter->setBrush(stateColor);
  painter->setPen(Qt::NoPen);
  painter->drawEllipse(badgeRect);

  painter->setPen(Qt::white);
  painter->setFont(QFont("Sans", 10, QFont::Bold));
  painter->drawText(badgeRect, Qt::AlignCenter, stateChar);
}
```

**Step 5: Add context menu for lifecycle transitions**

In `weaver_canvas.cpp`:

```cpp
void WeaverCanvas::showBlockContextMenu(PackageBlock* block, const QPoint& pos) {
  QMenu menu;

  // ... existing menu items ...

  if (block->isLifecycleNode()) {
    QMenu* lifecycleMenu = menu.addMenu(tr("Lifecycle"));

    QAction* configureAction = lifecycleMenu->addAction(tr("Configure"));
    QAction* activateAction = lifecycleMenu->addAction(tr("Activate"));
    QAction* deactivateAction = lifecycleMenu->addAction(tr("Deactivate"));
    QAction* cleanupAction = lifecycleMenu->addAction(tr("Cleanup"));
    QAction* shutdownAction = lifecycleMenu->addAction(tr("Shutdown"));

    // Enable/disable based on current state
    LifecycleState state = block->lifecycleState();
    configureAction->setEnabled(state == LifecycleState::Unconfigured);
    activateAction->setEnabled(state == LifecycleState::Inactive);
    deactivateAction->setEnabled(state == LifecycleState::Active);
    cleanupAction->setEnabled(state == LifecycleState::Inactive);

    connect(configureAction, &QAction::triggered, [this, block]() {
      lifecycleManager_->requestTransition(block->nodeName(), "configure");
    });
    // ... connect other actions
  }

  menu.exec(pos);
}
```

---

### Remote Robot Connection

**Branch:** `feature/remote-connection`

**Overview:** Connect to ROS2 systems running on remote machines over network or SSH tunnel.

#### New Files to Create

```
src/ros_weaver/include/ros_weaver/core/remote_connection_manager.hpp
src/ros_weaver/src/core/remote_connection_manager.cpp
src/ros_weaver/include/ros_weaver/widgets/remote_connection_dialog.hpp
src/ros_weaver/src/widgets/remote_connection_dialog.cpp
src/ros_weaver/include/ros_weaver/core/ssh_tunnel.hpp
src/ros_weaver/src/core/ssh_tunnel.cpp
```

#### Implementation Steps

**Step 1: Create RemoteConnectionManager**

```cpp
struct RobotProfile {
  QString name;
  QString hostname;
  int port = 22;
  QString username;
  QString sshKeyPath;
  int rosDomainId = 0;
  QString rosDistro;
  bool useSshTunnel = true;
  QStringList environmentVariables;
};

class RemoteConnectionManager : public QObject {
  Q_OBJECT
public:
  explicit RemoteConnectionManager(QObject* parent = nullptr);

  // Profile management
  void saveProfile(const RobotProfile& profile);
  void deleteProfile(const QString& name);
  QList<RobotProfile> loadProfiles();

  // Connection
  void connectToRobot(const RobotProfile& profile);
  void disconnect();
  bool isConnected() const;

  // Discovery
  QStringList discoverRemoteTopics();
  QStringList discoverRemoteNodes();
  QStringList discoverRemoteServices();

signals:
  void connectionEstablished(const QString& robotName);
  void connectionFailed(const QString& error);
  void connectionLost();
  void discoveryComplete(const QStringList& topics, const QStringList& nodes);

private:
  SshTunnel* sshTunnel_;
  RobotProfile currentProfile_;
  bool connected_ = false;

  void setupDdsEnvironment(const RobotProfile& profile);
  void startDiscovery();
};
```

**Step 2: Create SSH Tunnel wrapper**

```cpp
class SshTunnel : public QObject {
  Q_OBJECT
public:
  explicit SshTunnel(QObject* parent = nullptr);
  ~SshTunnel();

  void establish(const QString& host, int port, const QString& user,
                 const QString& keyPath, int localPort, int remotePort);
  void close();
  bool isActive() const;

signals:
  void tunnelEstablished(int localPort);
  void tunnelError(const QString& error);
  void tunnelClosed();

private:
  QProcess* sshProcess_;
  int localPort_;

  QString buildSshCommand(const QString& host, int port, const QString& user,
                          const QString& keyPath, int localPort, int remotePort);
};
```

**Step 3: Create connection dialog UI**

```cpp
class RemoteConnectionDialog : public QDialog {
  Q_OBJECT
public:
  explicit RemoteConnectionDialog(QWidget* parent = nullptr);

  RobotProfile getProfile() const;

private:
  void setupUi();
  void loadProfiles();
  void testConnection();

  // Profile selection
  QComboBox* profileCombo_;
  QPushButton* newProfileButton_;
  QPushButton* deleteProfileButton_;

  // Connection details
  QLineEdit* hostnameEdit_;
  QSpinBox* portSpinBox_;
  QLineEdit* usernameEdit_;
  QLineEdit* sshKeyEdit_;
  QPushButton* browseKeyButton_;

  // ROS settings
  QSpinBox* domainIdSpinBox_;
  QComboBox* distroCombo_;
  QCheckBox* useTunnelCheckbox_;

  // Environment
  QTableWidget* envTable_;

  // Actions
  QPushButton* testButton_;
  QPushButton* connectButton_;
  QPushButton* cancelButton_;

  QLabel* statusLabel_;

  RemoteConnectionManager* manager_;
};
```

**Step 4: DDS configuration for remote discovery**

```cpp
void RemoteConnectionManager::setupDdsEnvironment(const RobotProfile& profile) {
  // Set ROS_DOMAIN_ID
  qputenv("ROS_DOMAIN_ID", QString::number(profile.rosDomainId).toUtf8());

  // For Fast-DDS, configure discovery servers
  // Create XML configuration for unicast discovery to remote host
  QString xmlConfig = generateFastDdsConfig(profile.hostname);
  QString configPath = QDir::temp().filePath("ros_weaver_dds_config.xml");

  QFile file(configPath);
  if (file.open(QIODevice::WriteOnly)) {
    file.write(xmlConfig.toUtf8());
    file.close();
  }

  qputenv("FASTRTPS_DEFAULT_PROFILES_FILE", configPath.toUtf8());

  // Set custom environment variables
  for (const QString& envVar : profile.environmentVariables) {
    QStringList parts = envVar.split('=');
    if (parts.size() == 2) {
      qputenv(parts[0].toUtf8(), parts[1].toUtf8());
    }
  }
}

QString RemoteConnectionManager::generateFastDdsConfig(const QString& remoteHost) {
  return QString(R"(
<?xml version="1.0" encoding="UTF-8"?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <participant profile_name="ros_weaver_remote" is_default_profile="true">
    <rtps>
      <builtin>
        <discovery_config>
          <discoveryProtocol>SIMPLE</discoveryProtocol>
          <initialPeersList>
            <locator>
              <udpv4>
                <address>%1</address>
              </udpv4>
            </locator>
          </initialPeersList>
        </discovery_config>
      </builtin>
    </rtps>
  </participant>
</profiles>
)").arg(remoteHost);
}
```

**Step 5: Add to MainWindow menu**

```cpp
void MainWindow::setupConnectionMenu() {
  QMenu* connectionMenu = menuBar()->addMenu(tr("&Connection"));

  QAction* connectAction = connectionMenu->addAction(tr("Connect to Robot..."));
  connectAction->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_R));
  connect(connectAction, &QAction::triggered, this, &MainWindow::showRemoteConnectionDialog);

  QAction* disconnectAction = connectionMenu->addAction(tr("Disconnect"));
  disconnectAction->setEnabled(false);
  connect(disconnectAction, &QAction::triggered, remoteManager_, &RemoteConnectionManager::disconnect);

  connectionMenu->addSeparator();

  // Recent connections submenu
  recentConnectionsMenu_ = connectionMenu->addMenu(tr("Recent Connections"));
  updateRecentConnectionsMenu();
}
```

---

### MEDIUM PRIORITY

---

### Topic Latency Heatmap

**Branch:** `feature/latency-heatmap`

**Overview:** Visualize end-to-end message latency through node chains with color-coded overlays.

#### Implementation Steps

**Step 1: Create LatencyTracker class**

```cpp
struct LatencyMeasurement {
  QString sourceTopic;
  QString targetTopic;
  double latencyMs;
  qint64 timestamp;
};

class LatencyTracker : public QObject {
  Q_OBJECT
public:
  // Track latency by matching message timestamps across topics
  void trackLatency(const QString& inputTopic, const QString& outputTopic,
                    const QString& correlationField);

  // Get statistics
  double getAverageLatency(const QString& connection) const;
  double getP99Latency(const QString& connection) const;
  QList<LatencyMeasurement> getHistory(const QString& connection, int maxSize = 100);

signals:
  void latencyUpdated(const QString& connection, double latencyMs);
  void latencyAlert(const QString& connection, double latencyMs, double threshold);

private:
  // Track pending messages waiting for correlation
  QMap<QString, QList<QPair<qint64, QVariant>>> pendingCorrelations_;
  // Latency history per connection
  QMap<QString, QList<double>> latencyHistory_;
};
```

**Step 2: Add heatmap visualization to ConnectionLine**

```cpp
void ConnectionLine::setLatencyHeatmapEnabled(bool enabled) {
  showLatencyHeatmap_ = enabled;
  update();
}

void ConnectionLine::setLatency(double latencyMs) {
  currentLatency_ = latencyMs;
  update();
}

QColor ConnectionLine::getLatencyColor(double latencyMs) const {
  // Color scale: green (< 10ms) -> yellow (10-50ms) -> red (> 50ms)
  if (latencyMs < 10.0) {
    return QColor(100, 200, 100);  // Green
  } else if (latencyMs < 50.0) {
    // Interpolate green to yellow
    double t = (latencyMs - 10.0) / 40.0;
    return QColor(100 + 155 * t, 200, 100 - 50 * t);
  } else if (latencyMs < 100.0) {
    // Interpolate yellow to red
    double t = (latencyMs - 50.0) / 50.0;
    return QColor(255, 200 - 150 * t, 50 - 50 * t);
  } else {
    return QColor(255, 50, 50);  // Red
  }
}

void ConnectionLine::drawLatencyLabel(QPainter* painter) {
  if (!showLatencyHeatmap_ || currentLatency_ <= 0) return;

  QPointF pos = path_.pointAtPercent(0.7);
  QString label = QString("%1 ms").arg(currentLatency_, 0, 'f', 1);

  // Background
  QFontMetrics fm(painter->font());
  QRect textRect = fm.boundingRect(label);
  textRect.moveCenter(pos.toPoint());

  painter->setBrush(QColor(0, 0, 0, 180));
  painter->setPen(Qt::NoPen);
  painter->drawRoundedRect(textRect.adjusted(-4, -2, 4, 2), 3, 3);

  // Text colored by latency
  painter->setPen(getLatencyColor(currentLatency_));
  painter->drawText(textRect, Qt::AlignCenter, label);
}
```

**Step 3: Add toolbar toggle**

```cpp
void MainWindow::createLatencyHeatmapAction() {
  latencyHeatmapAction_ = new QAction(tr("Show Latency Heatmap"), this);
  latencyHeatmapAction_->setCheckable(true);
  latencyHeatmapAction_->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_L));

  connect(latencyHeatmapAction_, &QAction::toggled, [this](bool enabled) {
    canvas_->setLatencyHeatmapEnabled(enabled);
    if (enabled) {
      latencyTracker_->startTracking();
    } else {
      latencyTracker_->stopTracking();
    }
  });

  viewMenu_->addAction(latencyHeatmapAction_);
}
```

---

### Test Message Scenario Editor

**Branch:** `feature/scenario-editor`

**Overview:** Create and replay scripted message sequences for testing ROS2 systems.

#### New Files

```
src/ros_weaver/include/ros_weaver/widgets/scenario_editor_panel.hpp
src/ros_weaver/src/widgets/scenario_editor_panel.cpp
src/ros_weaver/include/ros_weaver/core/scenario_player.hpp
src/ros_weaver/src/core/scenario_player.cpp
```

#### Data Model

```cpp
struct ScenarioStep {
  enum Type { PublishMessage, WaitForMessage, WaitTime, Conditional };

  Type type;
  QString topicName;
  QString messageType;
  QJsonObject messageData;
  int waitTimeMs;
  QString conditionExpression;  // e.g., "msg.status == 1"
  int timeoutMs;
};

struct TestScenario {
  QString name;
  QString description;
  QList<ScenarioStep> steps;
  bool loopEnabled;
  int loopCount;
};
```

#### UI Components

```cpp
class ScenarioEditorPanel : public QWidget {
  // Timeline view of scenario steps
  QListWidget* stepsListWidget_;

  // Step editor
  QComboBox* stepTypeCombo_;
  QStackedWidget* stepEditorStack_;  // Different editors per step type

  // Publish message editor
  QComboBox* topicCombo_;
  QTextEdit* jsonEditor_;

  // Wait for message editor
  QLineEdit* conditionEdit_;
  QSpinBox* timeoutSpinBox_;

  // Wait time editor
  QSpinBox* waitTimeSpinBox_;

  // Controls
  QPushButton* playButton_;
  QPushButton* pauseButton_;
  QPushButton* stopButton_;
  QPushButton* recordButton_;  // Record from live system
  QProgressBar* progressBar_;

  // Import/Export
  QPushButton* exportPythonButton_;  // Export as pytest script
  QPushButton* saveButton_;
  QPushButton* loadButton_;
};
```

---

### Architecture Documentation Generator

**Branch:** `feature/doc-generator`

**Overview:** Auto-generate comprehensive architecture documentation from canvas.

#### Output Formats

1. **Markdown** - README-style documentation
2. **HTML** - Standalone documentation site
3. **PDF** - Print-ready documentation (via Qt PDF)

#### Template Structure

```markdown
# {PROJECT_NAME} Architecture

## Overview
{AI_GENERATED_SUMMARY}

## System Diagram
![Architecture Diagram]({MERMAID_DIAGRAM})

## Nodes

### {NODE_NAME}
- **Package:** {PACKAGE_NAME}
- **Executable:** {EXECUTABLE_NAME}
- **Namespace:** {NAMESPACE}

#### Subscriptions
| Topic | Message Type | QoS |
|-------|--------------|-----|
{SUBSCRIPTIONS_TABLE}

#### Publications
| Topic | Message Type | QoS |
|-------|--------------|-----|
{PUBLICATIONS_TABLE}

#### Parameters
| Name | Type | Default | Description |
|------|------|---------|-------------|
{PARAMETERS_TABLE}

## Topics
{TOPICS_SECTION}

## Message Types
{MESSAGE_SCHEMAS}

## Launch Configuration
```python
{GENERATED_LAUNCH_FILE}
```
```

---

### ros2 doctor Integration

**Branch:** `feature/ros2-doctor`

**Overview:** Built-in diagnostics panel using ros2 doctor functionality.

#### Implementation

```cpp
class DiagnosticsPanel : public QWidget {
public:
  void runDiagnostics();

private:
  // Diagnostic checks
  void checkRosEnvironment();
  void checkNetworkConfiguration();
  void checkDdsMiddleware();
  void checkTopicConnectivity();
  void checkNodeHealth();

  // UI
  QTreeWidget* resultsTree_;
  QTextEdit* detailsText_;
  QPushButton* runButton_;
  QPushButton* exportButton_;

  // Results
  struct DiagnosticResult {
    enum Level { OK, Warning, Error };
    QString category;
    QString check;
    Level level;
    QString message;
    QString suggestion;
  };
  QList<DiagnosticResult> results_;
};

void DiagnosticsPanel::checkDdsMiddleware() {
  DiagnosticResult result;
  result.category = "Middleware";
  result.check = "DDS Implementation";

  QString rmwImpl = qgetenv("RMW_IMPLEMENTATION");
  if (rmwImpl.isEmpty()) {
    rmwImpl = "rmw_fastrtps_cpp";  // Default
  }

  result.level = DiagnosticResult::OK;
  result.message = QString("Using %1").arg(rmwImpl);

  // Check for known issues
  if (rmwImpl == "rmw_cyclonedds_cpp") {
    // Check Cyclone config
    QString configFile = qgetenv("CYCLONEDDS_URI");
    if (configFile.isEmpty()) {
      result.level = DiagnosticResult::Warning;
      result.suggestion = "Consider setting CYCLONEDDS_URI for custom configuration";
    }
  }

  results_.append(result);
}
```

---

### Message Schema Diff Tool

**Branch:** `feature/schema-diff`

**Overview:** Compare message definitions between ROS2 distributions or package versions.

#### UI Design

```cpp
class SchemaDiffDialog : public QDialog {
private:
  // Left side - "From" version
  QComboBox* fromSourceCombo_;  // Local, File, ROS Distro
  QLineEdit* fromPathEdit_;
  QTreeWidget* fromSchemaTree_;

  // Right side - "To" version
  QComboBox* toSourceCombo_;
  QLineEdit* toPathEdit_;
  QTreeWidget* toSchemaTree_;

  // Diff view
  QTextEdit* diffOutput_;

  // Results summary
  QLabel* addedFieldsLabel_;
  QLabel* removedFieldsLabel_;
  QLabel* changedFieldsLabel_;

  void performDiff();
  void highlightDifferences();
};

struct FieldDiff {
  enum Type { Added, Removed, TypeChanged, DefaultChanged };
  Type type;
  QString fieldPath;  // e.g., "header.stamp.sec"
  QString oldValue;
  QString newValue;
};
```

---

### LOW PRIORITY

---

### Visual Behavior Tree Editor

**Branch:** `feature/behavior-tree-editor`

**Overview:** Drag-and-drop editor for BehaviorTree.CPP with ROS2 action integration.

This is a large feature - consider implementing as a separate dockable panel that integrates with the main canvas.

Key components:
- Custom QGraphicsScene for BT nodes
- Palette of BT node types (Sequence, Fallback, Decorator, Action, Condition)
- Property editor for node configuration
- Live execution visualization (highlight active nodes)
- Export to BehaviorTree.CPP XML format
- Import existing BT XML files

---

### Plugin/Extension System

**Branch:** `feature/plugin-system`

**Overview:** Allow third-party extensions via Qt plugin architecture.

```cpp
// Plugin interface
class RosWeaverPlugin {
public:
  virtual ~RosWeaverPlugin() = default;

  virtual QString name() const = 0;
  virtual QString version() const = 0;
  virtual QString description() const = 0;

  // Optional: Add dock widget
  virtual QWidget* createDockWidget() { return nullptr; }

  // Optional: Add menu items
  virtual QList<QAction*> menuActions() { return {}; }

  // Optional: Add canvas tools
  virtual QList<QAction*> toolbarActions() { return {}; }

  // Optional: Custom node types
  virtual QList<NodeTemplate> nodeTemplates() { return {}; }

  // Lifecycle
  virtual void initialize(MainWindow* mainWindow) = 0;
  virtual void shutdown() = 0;
};

#define RosWeaverPlugin_iid "org.ros_weaver.Plugin/1.0"
Q_DECLARE_INTERFACE(RosWeaverPlugin, RosWeaverPlugin_iid)
```

---

### CI/CD Pipeline Generator

**Branch:** `feature/cicd-generator`

Generates GitHub Actions or GitLab CI configuration:

```yaml
# Generated .github/workflows/ros2_build.yml
name: ROS2 Build and Test

on:
  push:
    branches: [main]
  pull_request:
    branches: [main]

jobs:
  build:
    runs-on: ubuntu-22.04
    container:
      image: ros:{ROS_DISTRO}-ros-base

    steps:
      - uses: actions/checkout@v3

      - name: Install dependencies
        run: |
          apt-get update
          rosdep install --from-paths src --ignore-src -r -y

      - name: Build
        run: |
          source /opt/ros/{ROS_DISTRO}/setup.bash
          colcon build --symlink-install

      - name: Test
        run: |
          source /opt/ros/{ROS_DISTRO}/setup.bash
          source install/setup.bash
          colcon test
          colcon test-result --verbose
```

---

### DDS Network Topology View

**Branch:** `feature/network-topology`

**Overview:** Visualize DDS discovery and network communication patterns.

Shows:
- Which nodes are on which hosts
- DDS domain IDs
- Unicast vs multicast discovery
- QoS compatibility matrix
- Network bandwidth between hosts

---

## Completed Feature Details

### AI-Powered Features ✅

#### Natural Language Architecture Generation
- Describe your robot system in plain language and get a generated architecture
- Uses Ollama integration for AI-powered suggestions
- Supports: robot type, use case, available sensors as input parameters
- Generates nodes, connections, and logical groupings automatically
- Example: "Create a ROS2 architecture for a mobile robot with SLAM and navigation"

#### Automated Error Fixing Suggestions
- "Ask AI" button in the Issue List panel for each detected issue
- Sends issue context to local LLM for targeted fix suggestions
- Provides structured suggestions with confidence scores
- Includes code examples for applicable fixes
- Integrates with static analysis results

#### Architecture Optimization Recommendations
- Tools → Optimize Architecture menu
- Analyzes current canvas architecture for:
  - Unused nodes and connections
  - Suboptimal QoS configurations
  - Missing common patterns
  - Performance bottlenecks
- Provides actionable recommendations with implementation guidance

#### Advanced Node Library Search
- Search tab in package browser with advanced filtering
- Category filters: Navigation, Perception, Manipulation, SLAM, Control, Drivers, Simulation, Utilities
- Message type filtering (common ROS2 message types)
- Feature filters: Has Inputs, Has Outputs, Has Services, Has Actions
- Double-click or "Add to Canvas" button to add nodes

#### Improved Namespace/Remapping Visualization
- **Namespace Badge**: Purple badge displayed above blocks showing their namespace (e.g., `/my_robot`)
- **Remapping Indicator**: Amber arrow icon in block header when topic/service remappings are configured
- **Connection Visualization**: Remapped connections shown with dashed amber lines
- **Remapping Editor**: Full editor for adding/removing namespace and topic remappings per block
- **Tooltip Support**: Connection tooltips show original and remapped topic names
- **Storage Integration**: Namespace and remappings stored on PackageBlock and persisted with projects

### Documentation & Discovery Features ✅

#### Context-Sensitive ROS2 Documentation Lookup
- Hover or right-click on nodes, topics, services, actions, message types, or parameters to open a side panel/pop-up with relevant official documentation.
- Sources:
  - Official ROS2 docs (`https://docs.ros.org`) for core packages.
  - Local parsing of `.msg`, `.srv`, `.action` files for field descriptions.
  - Cached index for offline use.
- Global searchable docs bar (e.g., search "LaserScan fields" or "MoveIt parameters").

#### Inline Tooltips & Message Schema Viewer
- Quick hover tooltips showing message fields, types, and defaults.
- Click-to-expand full interactive schema tree (similar to `rqt_msg` or Foxglove message inspector).

#### Package/Node README & Description Preview
- Automatically load and display `README.md` or `package.xml` description for selected nodes/packages in a dockable panel.

### Export, Import & Interoperability ✅

- Export canvas diagrams to **PlantUML**, **Mermaid**, or **Graphviz** for documentation.
- Import existing architectures from:
  - `rqt_graph` DOT files.
  - CARET `architecture.yaml` files.
- Save/load full projects as `.weaver` JSON/YAML files.

### Simulation & Visualization Enhancements ✅

- One-click launch of **Gazebo** or **Ignition** using the robot configuration wizard output.
- Integrated **ros2 bag** recording/playback with canvas animation during replay.

### Validation & Debugging Tools ✅

- Static analysis:
  - Topic type mismatches.
  - Unused publishers/subscribers.
  - QoS policy incompatibilities.
  - Cyclic dependency detection.
  - Naming convention validation.
  - Security issue detection.
- Live parameter validation dashboard with override capabilities.
- Error highlighting and suggested fixes on the canvas.

### Collaboration & Version Control ✅

- Robust **undo/redo** stack for canvas operations.
- Git-aware project saving with visual diff support for canvas changes.

### Usability & Polish ✅

- Multi-canvas tabs for separate compositions (e.g., navigation stack vs. manipulation).
- Improved namespace and remapping visualization.
- Expanded keyboard shortcuts and customizable themes.

#### Canvas Grid Visualization (Blender-style)
- Configurable grid overlay with major and minor lines for precise node placement.
- Settings:
  - Toggle grid visibility on/off.
  - Snap-to-grid option for automatic alignment when moving nodes.
  - Adjustable granularity for major lines (e.g., every 100px, 200px).
  - Adjustable granularity for minor lines (e.g., subdivisions between major lines).
  - Customizable grid line colors and opacity.
- Visual style similar to Blender's 3D viewport grid for familiarity.

### Canvas Navigation & Layout ✅

#### Mini-map Navigation
- Small overview panel for navigating large architectures (similar to VS Code's minimap).
- Shows entire canvas with viewport indicator.
- Click-to-navigate and drag viewport rectangle.
- Useful for projects with many nodes spread across large canvas areas.

#### Auto-layout Algorithms
- One-click automatic layout options to organize nodes:
  - **Hierarchical** - Top-to-bottom or left-to-right data flow.
  - **Force-directed** - Physics-based spacing for balanced distribution.
  - **Circular** - Arrange nodes in a circle, useful for cyclic architectures.
  - **Grid** - Snap all nodes to a regular grid pattern.
- Option to layout only selected nodes or entire canvas.
- Undo support for layout operations.

#### Canvas Annotations & Sticky Notes
- Add text notes and documentation directly on the canvas.
- Sticky note widgets with customizable colors.
- Support for markdown formatting in notes.
- Link notes to specific nodes or connections.
- Useful for team communication and architecture documentation.

### Live Monitoring & Visualization ✅

#### Connection Bandwidth Visualization
- Show live message rates (Hz) on connection lines.
- Color coding for connection health:
  - Green: Healthy, receiving messages at expected rate.
  - Yellow: Degraded, lower than expected rate.
  - Red: Stale or dropped messages.
- Optional bandwidth display (KB/s, MB/s).
- Click connection to see detailed statistics (latency, jitter, queue depth).

#### Time-Synced Playback Visualization
- During bag playback, animate data flow on canvas connections.
- Visual pulse/glow effect showing which topics are active at each timestamp.
- Synchronized with timeline scrubbing.
- Helps understand temporal relationships between nodes during debugging.

#### Node Health Dashboard
- Real-time monitoring overlay for each node:
  - CPU usage percentage.
  - Memory consumption.
  - Callback execution latency.
  - Dropped message count.
- Heatmap visualization option (color nodes by resource usage).
- Alert thresholds with visual warnings.
- Historical graphs accessible via node context menu.

### Configuration & Code Generation ✅

#### QoS Profile Visual Editor
- Visual QoS configuration per connection/topic.
- Dropdown presets: Sensor Data, Services, Parameters, Default.
- Custom configuration with all QoS policies:
  - Reliability (reliable/best effort).
  - Durability (volatile/transient local).
  - History depth.
  - Deadline, lifespan, liveliness.
- Compatibility checker showing warnings for mismatched publisher/subscriber QoS.
- Generate QoS code snippets for C++/Python.

#### Launch File Generator
- Generate ROS2 launch files directly from canvas configuration.
- Support for Python and XML launch file formats.
- Include:
  - Node declarations with parameters.
  - Remappings from canvas connections.
  - Namespace configurations.
  - Composable node containers where applicable.
- Preview before export with syntax highlighting.

#### Parameter Presets
- Save named parameter configurations for quick switching.
- Example presets: "simulation", "real_robot", "debug", "competition".
- Per-node or global parameter overrides.
- Quick toggle between presets from toolbar.
- Export/import presets as YAML files.
- Diff view between presets.

### Node Library & Templates ✅

#### Node Templates Library
- Pre-configured templates for common ROS2 patterns:
  - **Sensor Fusion Node** - Multi-sensor input with synchronized output.
  - **State Machine** - Template with state transitions and events.
  - **Behavior Tree Executor** - BehaviorTree.CPP integration template.
  - **Bridge Node** - ROS1↔ROS2 bridge configuration.
  - **Lifecycle Node** - Managed node with state callbacks.
  - **Component Node** - Composable node template.
- Customizable templates with placeholder parameters.

#### Workspace Browser Panel
- Browse local ROS2 workspace packages.
- Tree view of installed packages organized by:
  - Workspace (overlay order).
  - Package type (nodes, libraries, messages).
- Drag-and-drop nodes from browser onto canvas.
- Quick search/filter within workspace.
- Show package dependencies and reverse dependencies.

### Comparison & Analysis ✅

#### Architecture Diff View
- Visually compare two project versions side-by-side.
- Highlighting for:
  - Added nodes/connections (green).
  - Removed nodes/connections (red).
  - Modified parameters (yellow).
- Git integration to compare with previous commits.
- Export diff report as HTML or markdown.
- Useful for code reviews and architecture evolution tracking.

---

Feel free to open issues or pull requests for any remaining features — they'd make ROS2Weaver an even more powerful all-in-one ROS2 development environment!

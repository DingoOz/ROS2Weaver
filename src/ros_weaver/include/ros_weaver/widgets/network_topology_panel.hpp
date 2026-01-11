#ifndef ROS_WEAVER_WIDGETS_NETWORK_TOPOLOGY_PANEL_HPP
#define ROS_WEAVER_WIDGETS_NETWORK_TOPOLOGY_PANEL_HPP

#include <QGraphicsItem>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QMap>
#include <QWidget>

#include "ros_weaver/core/network_topology_manager.hpp"

class QCheckBox;
class QComboBox;
class QLabel;
class QPushButton;
class QSpinBox;
class QSplitter;
class QStackedWidget;
class QTableWidget;
class QTableWidgetItem;
class QToolButton;
class QTreeWidget;
class QTreeWidgetItem;

namespace ros_weaver {

class WeaverCanvas;

/**
 * @brief Custom graphics item for host cluster visualization
 */
class HostClusterItem : public QGraphicsItem {
public:
  HostClusterItem(const HostInfo& host, const QColor& color,
                  QGraphicsItem* parent = nullptr);

  QRectF boundingRect() const override;
  void paint(QPainter* painter, const QStyleOptionGraphicsItem* option,
             QWidget* widget) override;

  QString hostName() const { return hostInfo_.hostname; }
  QStringList nodeNames() const { return hostInfo_.nodeNames; }

  void setHighlighted(bool highlighted);

private:
  HostInfo hostInfo_;
  QColor baseColor_;
  QRectF bounds_;
  bool highlighted_ = false;
};

/**
 * @brief Panel for visualizing DDS network topology
 *
 * Features:
 * - Graph view showing hosts as clusters with contained nodes
 * - Table view with sortable host/node listing
 * - Switchable between graph and table views
 * - Bidirectional canvas integration
 * - Auto-refresh capability
 */
class NetworkTopologyPanel : public QWidget {
  Q_OBJECT

public:
  explicit NetworkTopologyPanel(QWidget* parent = nullptr);
  ~NetworkTopologyPanel() override;

  /**
   * @brief Set the main canvas for bidirectional integration
   */
  void setCanvas(WeaverCanvas* canvas);

  /**
   * @brief Set the network topology manager
   */
  void setNetworkTopologyManager(NetworkTopologyManager* manager);

signals:
  /**
   * @brief Request to highlight a node on the main canvas
   */
  void nodeSelectedOnCanvas(const QString& nodeName);

  /**
   * @brief Emitted when a host is highlighted
   */
  void hostHighlighted(const QString& hostname);

public slots:
  /**
   * @brief Trigger a refresh of the topology
   */
  void refresh();

  /**
   * @brief Enable/disable auto-refresh
   */
  void setAutoRefreshEnabled(bool enabled);

  /**
   * @brief Handle canvas selection changes for bidirectional sync
   */
  void onCanvasSelectionChanged();

private slots:
  void onTopologyUpdated(const NetworkTopology& topology);
  void onScanStarted();
  void onScanProgress(int percent, const QString& message);
  void onScanFailed(const QString& error);

  void onViewModeChanged(int index);
  void onRefreshClicked();
  void onAutoRefreshToggled(bool enabled);
  void onRefreshIntervalChanged(int value);

  void onHostClicked(QGraphicsItem* item);
  void onTableItemClicked(QTableWidgetItem* item);
  void onHostTreeItemClicked(QTreeWidgetItem* item, int column);

  void onZoomIn();
  void onZoomOut();
  void onFitView();

private:
  void setupUi();
  void setupConnections();
  void setupToolbar();
  void setupStatusBar();
  void setupGraphView();
  void setupTableView();

  // Graph visualization
  void updateGraphView(const NetworkTopology& topology);
  void drawHostCluster(const HostInfo& host, const QPointF& position);
  void arrangeHosts();
  void clearGraphView();

  // Table view
  void updateTableView(const NetworkTopology& topology);
  void updateHostsTable(const QList<HostInfo>& hosts);
  void updateNodesTree(const QList<ParticipantInfo>& participants);

  // Canvas integration
  void highlightCanvasNodes(const QStringList& nodeNames);
  void highlightHostInGraph(const QString& hostname);
  void highlightNodeInGraph(const QString& nodeName);

  // Utility
  QColor getHostColor(int index) const;

  // UI Components - Toolbar
  QPushButton* refreshButton_ = nullptr;
  QCheckBox* autoRefreshCheck_ = nullptr;
  QSpinBox* refreshIntervalSpin_ = nullptr;
  QComboBox* viewModeCombo_ = nullptr;
  QToolButton* zoomInButton_ = nullptr;
  QToolButton* zoomOutButton_ = nullptr;
  QToolButton* fitViewButton_ = nullptr;

  // UI Components - Status bar
  QLabel* domainIdLabel_ = nullptr;
  QLabel* rmwLabel_ = nullptr;
  QLabel* discoveryTypeLabel_ = nullptr;
  QLabel* hostCountLabel_ = nullptr;
  QLabel* nodeCountLabel_ = nullptr;
  QLabel* statusLabel_ = nullptr;

  // UI Components - Main view
  QStackedWidget* viewStack_ = nullptr;

  // Graph view components
  QGraphicsView* graphView_ = nullptr;
  QGraphicsScene* graphScene_ = nullptr;

  // Table view components
  QSplitter* tableSplitter_ = nullptr;
  QTableWidget* hostsTable_ = nullptr;
  QTreeWidget* nodesTree_ = nullptr;

  // State
  WeaverCanvas* canvas_ = nullptr;
  NetworkTopologyManager* manager_ = nullptr;
  NetworkTopology currentTopology_;

  // Host graphics items map
  QMap<QString, HostClusterItem*> hostItems_;

  // Zoom settings
  double currentZoom_ = 1.0;

  // Constants
  static const QList<QColor> kHostColors;
  static constexpr double kMinZoom = 0.25;
  static constexpr double kMaxZoom = 4.0;
  static constexpr double kZoomStep = 0.25;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_WIDGETS_NETWORK_TOPOLOGY_PANEL_HPP

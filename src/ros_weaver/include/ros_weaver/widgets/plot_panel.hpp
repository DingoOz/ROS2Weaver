#ifndef ROS_WEAVER_PLOT_PANEL_HPP
#define ROS_WEAVER_PLOT_PANEL_HPP

#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QSplitter>
#include <QPushButton>
#include <QToolButton>
#include <QComboBox>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QTimer>
#include <QMenu>
#include <QColorDialog>
#include <QTreeWidget>

#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>
#include <QtCharts/QDateTimeAxis>

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <thread>
#include <atomic>
#include <mutex>
#include <deque>
#include <map>

QT_CHARTS_USE_NAMESPACE

namespace ros_weaver {

class WeaverCanvas;

// Data point for plotting
struct PlotDataPoint {
  qint64 timestamp;  // milliseconds since epoch
  double value;
};

// Information about a plotted series
struct PlotSeriesInfo {
  QString topicName;
  QString fieldPath;      // e.g., "linear.x" for Twist
  QString fullPath;       // topicName + fieldPath
  QColor color;
  QLineSeries* series;
  std::deque<PlotDataPoint> buffer;

  // Statistics
  double minValue = 0.0;
  double maxValue = 0.0;
  double meanValue = 0.0;
  double currentValue = 0.0;
  int messageCount = 0;

  // For rate calculation
  qint64 lastMessageTime = 0;
  double publishRate = 0.0;
};

// Represents a field that can be plotted from a message
struct PlottableField {
  QString fieldPath;    // Path like "linear.x" or "position[0]"
  QString fieldType;    // e.g., "float64", "int32"
  QString fullPath;     // topicName/fieldPath
  int arrayIndex = -1;  // -1 if not array element
};

class PlotPanel : public QWidget {
  Q_OBJECT

public:
  explicit PlotPanel(QWidget* parent = nullptr);
  ~PlotPanel() override;

  // Set canvas for integration
  void setCanvas(WeaverCanvas* canvas) { canvas_ = canvas; }

public slots:
  // Add a topic/field to plot
  void addPlot(const QString& topicName, const QString& fieldPath);
  void removePlot(const QString& fullPath);
  void clearAllPlots();

  // Time window control
  void setTimeWindow(int seconds);

  // Playback control
  void pause();
  void resume();
  void togglePause();

  // Clear data but keep subscriptions
  void clearData();

  // Export data
  void exportToCsv();
  void copyToClipboard();
  void takeScreenshot();

signals:
  // Emitted when data is received (for thread-safe UI updates)
  void dataReceived(const QString& fullPath, double value, qint64 timestamp);

  // Emitted when a topic/field is added or removed
  void plotAdded(const QString& fullPath);
  void plotRemoved(const QString& fullPath);

private slots:
  void onDataReceived(const QString& fullPath, double value, qint64 timestamp);
  void onUpdateTimer();
  void onAddTopicClicked();
  void onSeriesListContextMenu(const QPoint& pos);
  void onSeriesSelectionChanged();
  void onTimeWindowChanged(int index);

private:
  void setupUi();
  QWidget* createToolbar();
  void setupChart();
  void setupSeriesList();
  void setupDetailsPanel();
  void setupConnections();

  void initializeRosNode();
  void shutdownRosNode();

  // Subscribe to a topic for plotting
  void subscribeToTopic(const QString& topicName, const QString& msgType,
                        const QString& fieldPath);
  void unsubscribeFromTopic(const QString& topicName);

  // Extract numeric value from serialized message at field path
  bool extractFieldValue(const std::shared_ptr<rclcpp::SerializedMessage>& msg,
                         const QString& msgType, const QString& fieldPath,
                         double& outValue);

  // Discover plottable fields in a message type
  QList<PlottableField> discoverPlottableFields(const QString& msgType);

  // Update chart with latest data
  void updateChart();
  void updateStatistics(PlotSeriesInfo& info);
  void updateDetailsPanel();

  // Adjust axes based on data
  void adjustAxes();

  // Generate a unique color for new series
  QColor generateSeriesColor();

  // Show topic/field selection dialog
  void showAddTopicDialog();

  // UI Components
  QSplitter* mainSplitter_;

  // Toolbar
  QPushButton* addTopicButton_;
  QPushButton* clearButton_;
  QToolButton* pauseButton_;
  QComboBox* timeWindowCombo_;
  QPushButton* exportButton_;
  QPushButton* screenshotButton_;
  QLabel* statusLabel_;

  // Chart
  QChart* chart_;
  QChartView* chartView_;
  QValueAxis* axisX_;
  QValueAxis* axisY_;

  // Series list (left panel)
  QListWidget* seriesList_;

  // Details panel (bottom)
  QWidget* detailsPanel_;
  QLabel* selectedSeriesLabel_;
  QLabel* statsLabel_;
  QLabel* rateLabel_;

  // Canvas integration
  WeaverCanvas* canvas_;

  // ROS2 components
  std::shared_ptr<rclcpp::Node> rosNode_;
  std::unique_ptr<std::thread> spinThread_;
  std::atomic<bool> spinning_;

  // Plot data
  std::map<std::string, PlotSeriesInfo> plotSeries_;  // fullPath -> info
  std::map<std::string, rclcpp::GenericSubscription::SharedPtr> subscriptions_;
  mutable std::mutex dataMutex_;

  // Time window (seconds)
  int timeWindowSeconds_;
  static constexpr int MAX_BUFFER_SIZE = 10000;  // Max points per series

  // Playback state
  bool paused_;

  // Update timer
  QTimer* updateTimer_;
  static constexpr int UPDATE_INTERVAL_MS = 50;  // 20 Hz update rate

  // Color palette for series
  QList<QColor> colorPalette_;
  int colorIndex_;

  // Selected series for details
  QString selectedSeriesPath_;
};

}  // namespace ros_weaver

Q_DECLARE_METATYPE(ros_weaver::PlotDataPoint)

#endif  // ROS_WEAVER_PLOT_PANEL_HPP

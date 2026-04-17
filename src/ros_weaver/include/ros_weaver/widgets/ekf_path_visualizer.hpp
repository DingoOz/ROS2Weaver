#ifndef ROS_WEAVER_WIDGETS_EKF_PATH_VISUALIZER_HPP
#define ROS_WEAVER_WIDGETS_EKF_PATH_VISUALIZER_HPP

#include <QWidget>
#include <QChartView>
#include <QChart>
#include <QLineSeries>
#include <QScatterSeries>
#include <QValueAxis>
#include <QCheckBox>
#include <QLabel>

#include "ros_weaver/core/ekf_config.hpp"

namespace ros_weaver {

/**
 * @brief 2D trajectory visualization widget using Qt Charts
 *
 * Displays multiple trajectories (raw odom, EKF output, ground truth)
 * with color coding, legend, and interactive zoom/pan.
 */
class EKFPathVisualizer : public QWidget {
  Q_OBJECT

public:
  explicit EKFPathVisualizer(QWidget* parent = nullptr);
  ~EKFPathVisualizer() override;

  // Add/remove trajectories
  void addTrajectory(const Trajectory& trajectory);
  void removeTrajectory(const QString& name);
  void clearTrajectories();

  // Update a specific trajectory
  void updateTrajectory(const Trajectory& trajectory);

  // Set all trajectories at once
  void setTrajectories(const Trajectory& ekf,
                       const Trajectory& rawOdom,
                       const Trajectory& groundTruth);

  // Highlighting
  void highlightPoint(const QString& trajectoryName, int pointIndex);
  void clearHighlight();

  // View control
  void fitToView();
  void resetZoom();

  // Options
  void setShowStartEndMarkers(bool show);
  bool showStartEndMarkers() const;

  void setShowGrid(bool show);
  bool showGrid() const;

  void setEqualAspectRatio(bool equal);
  bool equalAspectRatio() const;

signals:
  void pointClicked(const QString& trajectoryName, int pointIndex, double x, double y);
  void trajectoryVisibilityChanged(const QString& name, bool visible);

private slots:
  void onSeriesClicked(const QPointF& point);
  void onVisibilityCheckToggled(bool checked);

private:
  void setupUi();
  void updateAxes();
  QtCharts::QLineSeries* createSeries(const Trajectory& trajectory);
  void addStartEndMarkers(const Trajectory& trajectory);

  // Chart components
  QtCharts::QChartView* chartView_ = nullptr;
  QtCharts::QChart* chart_ = nullptr;
  QtCharts::QValueAxis* axisX_ = nullptr;
  QtCharts::QValueAxis* axisY_ = nullptr;

  // Trajectories
  QMap<QString, Trajectory> trajectories_;
  QMap<QString, QtCharts::QLineSeries*> series_;
  QMap<QString, QCheckBox*> visibilityChecks_;

  // Markers for start/end points
  QMap<QString, QtCharts::QScatterSeries*> startMarkers_;
  QMap<QString, QtCharts::QScatterSeries*> endMarkers_;

  // Highlight
  QtCharts::QScatterSeries* highlightSeries_ = nullptr;

  // Options
  bool showStartEndMarkers_ = true;
  bool equalAspectRatio_ = true;

  // Legend/controls widget
  QWidget* legendWidget_ = nullptr;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_WIDGETS_EKF_PATH_VISUALIZER_HPP

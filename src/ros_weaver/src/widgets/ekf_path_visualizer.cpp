#include "ros_weaver/widgets/ekf_path_visualizer.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QPushButton>

#include <cmath>
#include <algorithm>

using namespace QtCharts;

namespace ros_weaver {

EKFPathVisualizer::EKFPathVisualizer(QWidget* parent)
  : QWidget(parent)
{
  setupUi();
}

EKFPathVisualizer::~EKFPathVisualizer() = default;

void EKFPathVisualizer::setupUi() {
  auto* mainLayout = new QVBoxLayout(this);
  mainLayout->setContentsMargins(0, 0, 0, 0);

  // Create chart
  chart_ = new QChart();
  chart_->setTitle("Trajectory Comparison");
  chart_->setAnimationOptions(QChart::NoAnimation);
  chart_->legend()->setVisible(true);
  chart_->legend()->setAlignment(Qt::AlignBottom);

  // Create axes
  axisX_ = new QValueAxis();
  axisX_->setTitleText("X (m)");
  axisX_->setGridLineVisible(true);
  chart_->addAxis(axisX_, Qt::AlignBottom);

  axisY_ = new QValueAxis();
  axisY_->setTitleText("Y (m)");
  axisY_->setGridLineVisible(true);
  chart_->addAxis(axisY_, Qt::AlignLeft);

  // Create chart view
  chartView_ = new QChartView(chart_, this);
  chartView_->setRenderHint(QPainter::Antialiasing);
  chartView_->setRubberBand(QChartView::RectangleRubberBand);

  mainLayout->addWidget(chartView_, 1);

  // Legend/controls
  legendWidget_ = new QWidget(this);
  auto* legendLayout = new QHBoxLayout(legendWidget_);
  legendLayout->setContentsMargins(4, 4, 4, 4);

  auto* resetZoomBtn = new QPushButton("Reset Zoom", this);
  connect(resetZoomBtn, &QPushButton::clicked, this, &EKFPathVisualizer::resetZoom);
  legendLayout->addWidget(resetZoomBtn);

  auto* fitViewBtn = new QPushButton("Fit to View", this);
  connect(fitViewBtn, &QPushButton::clicked, this, &EKFPathVisualizer::fitToView);
  legendLayout->addWidget(fitViewBtn);

  legendLayout->addStretch();

  mainLayout->addWidget(legendWidget_);

  // Highlight series
  highlightSeries_ = new QScatterSeries();
  highlightSeries_->setName("Selected");
  highlightSeries_->setMarkerSize(12);
  highlightSeries_->setColor(Qt::red);
  highlightSeries_->setBorderColor(Qt::darkRed);
  chart_->addSeries(highlightSeries_);
  highlightSeries_->attachAxis(axisX_);
  highlightSeries_->attachAxis(axisY_);
  highlightSeries_->setVisible(false);
}

void EKFPathVisualizer::addTrajectory(const Trajectory& trajectory) {
  if (trajectory.poses.isEmpty()) {
    return;
  }

  // Remove existing if present
  if (series_.contains(trajectory.name)) {
    removeTrajectory(trajectory.name);
  }

  trajectories_[trajectory.name] = trajectory;

  // Create and add series
  auto* lineSeries = createSeries(trajectory);
  series_[trajectory.name] = lineSeries;

  chart_->addSeries(lineSeries);
  lineSeries->attachAxis(axisX_);
  lineSeries->attachAxis(axisY_);

  // Connect click handler
  connect(lineSeries, &QLineSeries::clicked, this, &EKFPathVisualizer::onSeriesClicked);

  // Add start/end markers
  if (showStartEndMarkers_) {
    addStartEndMarkers(trajectory);
  }

  // Add visibility checkbox
  auto* check = new QCheckBox(trajectory.name, legendWidget_);
  check->setChecked(true);
  check->setProperty("trajectory_name", trajectory.name);
  connect(check, &QCheckBox::toggled, this, &EKFPathVisualizer::onVisibilityCheckToggled);
  visibilityChecks_[trajectory.name] = check;

  // Insert before stretch
  auto* layout = static_cast<QHBoxLayout*>(legendWidget_->layout());
  layout->insertWidget(layout->count() - 1, check);

  updateAxes();
}

void EKFPathVisualizer::removeTrajectory(const QString& name) {
  if (series_.contains(name)) {
    chart_->removeSeries(series_[name]);
    delete series_[name];
    series_.remove(name);
  }

  if (startMarkers_.contains(name)) {
    chart_->removeSeries(startMarkers_[name]);
    delete startMarkers_[name];
    startMarkers_.remove(name);
  }

  if (endMarkers_.contains(name)) {
    chart_->removeSeries(endMarkers_[name]);
    delete endMarkers_[name];
    endMarkers_.remove(name);
  }

  if (visibilityChecks_.contains(name)) {
    delete visibilityChecks_[name];
    visibilityChecks_.remove(name);
  }

  trajectories_.remove(name);
  updateAxes();
}

void EKFPathVisualizer::clearTrajectories() {
  QStringList names = trajectories_.keys();
  for (const QString& name : names) {
    removeTrajectory(name);
  }
}

void EKFPathVisualizer::updateTrajectory(const Trajectory& trajectory) {
  if (series_.contains(trajectory.name)) {
    auto* lineSeries = series_[trajectory.name];
    lineSeries->clear();

    for (const auto& pose : trajectory.poses) {
      lineSeries->append(pose.x, pose.y);
    }

    trajectories_[trajectory.name] = trajectory;

    // Update markers
    if (showStartEndMarkers_ && startMarkers_.contains(trajectory.name)) {
      startMarkers_[trajectory.name]->clear();
      endMarkers_[trajectory.name]->clear();

      if (!trajectory.poses.isEmpty()) {
        startMarkers_[trajectory.name]->append(trajectory.poses.first().x,
                                                trajectory.poses.first().y);
        endMarkers_[trajectory.name]->append(trajectory.poses.last().x,
                                              trajectory.poses.last().y);
      }
    }

    updateAxes();
  } else {
    addTrajectory(trajectory);
  }
}

void EKFPathVisualizer::setTrajectories(const Trajectory& ekf,
                                         const Trajectory& rawOdom,
                                         const Trajectory& groundTruth) {
  clearTrajectories();

  if (!rawOdom.poses.isEmpty()) {
    Trajectory odom = rawOdom;
    odom.color = QColor(128, 128, 128);  // Gray
    odom.name = "Raw Odometry";
    addTrajectory(odom);
  }

  if (!groundTruth.poses.isEmpty()) {
    Trajectory gt = groundTruth;
    gt.color = QColor(0, 180, 0);  // Green
    gt.name = "Ground Truth";
    addTrajectory(gt);
  }

  if (!ekf.poses.isEmpty()) {
    Trajectory filtered = ekf;
    filtered.color = QColor(0, 120, 255);  // Blue
    filtered.name = "EKF Output";
    addTrajectory(filtered);
  }

  fitToView();
}

void EKFPathVisualizer::highlightPoint(const QString& trajectoryName, int pointIndex) {
  if (!trajectories_.contains(trajectoryName)) {
    return;
  }

  const auto& traj = trajectories_[trajectoryName];
  if (pointIndex < 0 || pointIndex >= traj.poses.size()) {
    return;
  }

  const auto& pose = traj.poses[pointIndex];

  highlightSeries_->clear();
  highlightSeries_->append(pose.x, pose.y);
  highlightSeries_->setVisible(true);
}

void EKFPathVisualizer::clearHighlight() {
  highlightSeries_->clear();
  highlightSeries_->setVisible(false);
}

void EKFPathVisualizer::fitToView() {
  if (trajectories_.isEmpty()) {
    axisX_->setRange(-10, 10);
    axisY_->setRange(-10, 10);
    return;
  }

  double minX = std::numeric_limits<double>::max();
  double maxX = std::numeric_limits<double>::lowest();
  double minY = std::numeric_limits<double>::max();
  double maxY = std::numeric_limits<double>::lowest();

  for (const auto& traj : trajectories_) {
    for (const auto& pose : traj.poses) {
      minX = std::min(minX, pose.x);
      maxX = std::max(maxX, pose.x);
      minY = std::min(minY, pose.y);
      maxY = std::max(maxY, pose.y);
    }
  }

  // Add padding
  double padX = (maxX - minX) * 0.1;
  double padY = (maxY - minY) * 0.1;

  if (padX < 1.0) padX = 1.0;
  if (padY < 1.0) padY = 1.0;

  minX -= padX;
  maxX += padX;
  minY -= padY;
  maxY += padY;

  // Equal aspect ratio
  if (equalAspectRatio_) {
    double rangeX = maxX - minX;
    double rangeY = maxY - minY;
    double range = std::max(rangeX, rangeY);

    double centerX = (minX + maxX) / 2.0;
    double centerY = (minY + maxY) / 2.0;

    minX = centerX - range / 2.0;
    maxX = centerX + range / 2.0;
    minY = centerY - range / 2.0;
    maxY = centerY + range / 2.0;
  }

  axisX_->setRange(minX, maxX);
  axisY_->setRange(minY, maxY);
}

void EKFPathVisualizer::resetZoom() {
  fitToView();
}

void EKFPathVisualizer::setShowStartEndMarkers(bool show) {
  if (showStartEndMarkers_ == show) {
    return;
  }

  showStartEndMarkers_ = show;

  if (show) {
    for (const auto& traj : trajectories_) {
      addStartEndMarkers(traj);
    }
  } else {
    for (auto* marker : startMarkers_) {
      chart_->removeSeries(marker);
      delete marker;
    }
    startMarkers_.clear();

    for (auto* marker : endMarkers_) {
      chart_->removeSeries(marker);
      delete marker;
    }
    endMarkers_.clear();
  }
}

bool EKFPathVisualizer::showStartEndMarkers() const {
  return showStartEndMarkers_;
}

void EKFPathVisualizer::setShowGrid(bool show) {
  axisX_->setGridLineVisible(show);
  axisY_->setGridLineVisible(show);
}

bool EKFPathVisualizer::showGrid() const {
  return axisX_->isGridLineVisible();
}

void EKFPathVisualizer::setEqualAspectRatio(bool equal) {
  equalAspectRatio_ = equal;
  fitToView();
}

bool EKFPathVisualizer::equalAspectRatio() const {
  return equalAspectRatio_;
}

void EKFPathVisualizer::onSeriesClicked(const QPointF& point) {
  auto* series = qobject_cast<QLineSeries*>(sender());
  if (!series) return;

  // Find trajectory name
  QString name;
  for (auto it = series_.begin(); it != series_.end(); ++it) {
    if (it.value() == series) {
      name = it.key();
      break;
    }
  }

  if (name.isEmpty() || !trajectories_.contains(name)) {
    return;
  }

  // Find closest point
  const auto& traj = trajectories_[name];
  int closestIdx = 0;
  double closestDist = std::numeric_limits<double>::max();

  for (int i = 0; i < traj.poses.size(); ++i) {
    double dx = traj.poses[i].x - point.x();
    double dy = traj.poses[i].y - point.y();
    double dist = dx*dx + dy*dy;
    if (dist < closestDist) {
      closestDist = dist;
      closestIdx = i;
    }
  }

  highlightPoint(name, closestIdx);
  emit pointClicked(name, closestIdx, traj.poses[closestIdx].x, traj.poses[closestIdx].y);
}

void EKFPathVisualizer::onVisibilityCheckToggled(bool checked) {
  auto* check = qobject_cast<QCheckBox*>(sender());
  if (!check) return;

  QString name = check->property("trajectory_name").toString();

  if (series_.contains(name)) {
    series_[name]->setVisible(checked);
  }
  if (startMarkers_.contains(name)) {
    startMarkers_[name]->setVisible(checked && showStartEndMarkers_);
  }
  if (endMarkers_.contains(name)) {
    endMarkers_[name]->setVisible(checked && showStartEndMarkers_);
  }

  emit trajectoryVisibilityChanged(name, checked);
}

void EKFPathVisualizer::updateAxes() {
  // Called after trajectory changes to update axis ranges
  // Don't auto-fit - user might have zoomed
}

QLineSeries* EKFPathVisualizer::createSeries(const Trajectory& trajectory) {
  auto* series = new QLineSeries();
  series->setName(trajectory.name);

  QPen pen(trajectory.color);
  pen.setWidth(2);
  series->setPen(pen);

  for (const auto& pose : trajectory.poses) {
    series->append(pose.x, pose.y);
  }

  return series;
}

void EKFPathVisualizer::addStartEndMarkers(const Trajectory& trajectory) {
  if (trajectory.poses.isEmpty()) {
    return;
  }

  // Start marker (green circle)
  auto* startSeries = new QScatterSeries();
  startSeries->setName(trajectory.name + " Start");
  startSeries->setMarkerSize(10);
  startSeries->setColor(Qt::green);
  startSeries->setBorderColor(Qt::darkGreen);
  startSeries->append(trajectory.poses.first().x, trajectory.poses.first().y);

  chart_->addSeries(startSeries);
  startSeries->attachAxis(axisX_);
  startSeries->attachAxis(axisY_);
  startMarkers_[trajectory.name] = startSeries;

  // End marker (red square)
  auto* endSeries = new QScatterSeries();
  endSeries->setName(trajectory.name + " End");
  endSeries->setMarkerShape(QScatterSeries::MarkerShapeRectangle);
  endSeries->setMarkerSize(10);
  endSeries->setColor(Qt::red);
  endSeries->setBorderColor(Qt::darkRed);
  endSeries->append(trajectory.poses.last().x, trajectory.poses.last().y);

  chart_->addSeries(endSeries);
  endSeries->attachAxis(axisX_);
  endSeries->attachAxis(axisY_);
  endMarkers_[trajectory.name] = endSeries;
}

}  // namespace ros_weaver

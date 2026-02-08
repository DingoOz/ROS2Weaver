#ifndef ROS_WEAVER_PLOT_SERIES_CONFIG_DIALOG_HPP
#define ROS_WEAVER_PLOT_SERIES_CONFIG_DIALOG_HPP

#include <QDialog>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QRadioButton>
#include <QPushButton>
#include <QButtonGroup>

namespace ros_weaver {

struct PlotSeriesConfig;
enum class PlotRenderMode;

class PlotSeriesConfigDialog : public QDialog {
  Q_OBJECT

public:
  explicit PlotSeriesConfigDialog(const PlotSeriesConfig& config,
                                   QWidget* parent = nullptr);

  PlotSeriesConfig result() const;

private:
  void setupUi(const PlotSeriesConfig& config);
  void updateSectionVisibility();
  void chooseColor(QPushButton* btn, QColor& target);

  // Appearance
  QPushButton* colorButton_;
  QColor selectedColor_;
  QSpinBox* thicknessSpin_;

  // Decimation
  QCheckBox* decimationCheck_;
  QDoubleSpinBox* sampleRateSpin_;

  // Render mode
  QRadioButton* solidRadio_;
  QRadioButton* thresholdRadio_;
  QRadioButton* gradientRadio_;
  QButtonGroup* modeGroup_;

  // Threshold
  QDoubleSpinBox* thresholdUpperSpin_;
  QDoubleSpinBox* thresholdLowerSpin_;
  QPushButton* alarmColorButton_;
  QColor alarmColor_;
  QWidget* thresholdSection_;

  // Gradient
  QDoubleSpinBox* gradientMinSpin_;
  QDoubleSpinBox* gradientMaxSpin_;
  QPushButton* gradientLowColorButton_;
  QPushButton* gradientHighColorButton_;
  QColor gradientLowColor_;
  QColor gradientHighColor_;
  QSpinBox* gradientBucketsSpin_;
  QWidget* gradientSection_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_PLOT_SERIES_CONFIG_DIALOG_HPP

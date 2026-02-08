#include "ros_weaver/widgets/plot_series_config_dialog.hpp"
#include "ros_weaver/widgets/plot_panel.hpp"
#include "ros_weaver/core/constants.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QLabel>
#include <QDialogButtonBox>
#include <QColorDialog>

namespace ros_weaver {

PlotSeriesConfigDialog::PlotSeriesConfigDialog(const PlotSeriesConfig& config,
                                                 QWidget* parent)
  : QDialog(parent)
{
  setWindowTitle(tr("Configure Series"));
  setMinimumWidth(380);
  setupUi(config);
}

void PlotSeriesConfigDialog::setupUi(const PlotSeriesConfig& config) {
  QVBoxLayout* mainLayout = new QVBoxLayout(this);

  // ---- Appearance ----
  QGroupBox* appearanceGroup = new QGroupBox(tr("Appearance"));
  QHBoxLayout* appearLayout = new QHBoxLayout(appearanceGroup);

  appearLayout->addWidget(new QLabel(tr("Color:")));
  colorButton_ = new QPushButton();
  colorButton_->setFixedSize(40, 24);
  selectedColor_ = config.color;
  colorButton_->setStyleSheet(
    QString("background-color: %1; border: 1px solid #555;").arg(selectedColor_.name()));
  connect(colorButton_, &QPushButton::clicked, [this]() {
    chooseColor(colorButton_, selectedColor_);
  });
  appearLayout->addWidget(colorButton_);

  appearLayout->addSpacing(16);
  appearLayout->addWidget(new QLabel(tr("Thickness:")));
  thicknessSpin_ = new QSpinBox();
  thicknessSpin_->setRange(constants::plot::MIN_LINE_THICKNESS,
                            constants::plot::MAX_LINE_THICKNESS);
  thicknessSpin_->setValue(config.lineThickness);
  appearLayout->addWidget(thicknessSpin_);
  appearLayout->addStretch();

  mainLayout->addWidget(appearanceGroup);

  // ---- Sample Rate ----
  QGroupBox* sampleGroup = new QGroupBox(tr("Sample Rate"));
  QHBoxLayout* sampleLayout = new QHBoxLayout(sampleGroup);

  decimationCheck_ = new QCheckBox(tr("Limit rate"));
  decimationCheck_->setChecked(config.decimationEnabled);
  sampleLayout->addWidget(decimationCheck_);

  sampleRateSpin_ = new QDoubleSpinBox();
  sampleRateSpin_->setRange(constants::plot::MIN_SAMPLE_RATE_HZ,
                             constants::plot::MAX_SAMPLE_RATE_HZ);
  sampleRateSpin_->setDecimals(1);
  sampleRateSpin_->setSuffix(tr(" Hz"));
  sampleRateSpin_->setValue(config.maxSampleRateHz > 0 ? config.maxSampleRateHz : 10.0);
  sampleRateSpin_->setEnabled(config.decimationEnabled);
  sampleLayout->addWidget(sampleRateSpin_);
  sampleLayout->addStretch();

  connect(decimationCheck_, &QCheckBox::toggled, sampleRateSpin_, &QWidget::setEnabled);

  mainLayout->addWidget(sampleGroup);

  // ---- Render Mode ----
  QGroupBox* modeGroup = new QGroupBox(tr("Render Mode"));
  QHBoxLayout* modeLayout = new QHBoxLayout(modeGroup);

  solidRadio_ = new QRadioButton(tr("Solid"));
  thresholdRadio_ = new QRadioButton(tr("Threshold"));
  gradientRadio_ = new QRadioButton(tr("Gradient"));

  modeGroup_ = new QButtonGroup(this);
  modeGroup_->addButton(solidRadio_, 0);
  modeGroup_->addButton(thresholdRadio_, 1);
  modeGroup_->addButton(gradientRadio_, 2);

  switch (config.renderMode) {
    case PlotRenderMode::Threshold: thresholdRadio_->setChecked(true); break;
    case PlotRenderMode::Gradient: gradientRadio_->setChecked(true); break;
    default: solidRadio_->setChecked(true); break;
  }

  modeLayout->addWidget(solidRadio_);
  modeLayout->addWidget(thresholdRadio_);
  modeLayout->addWidget(gradientRadio_);
  modeLayout->addStretch();

  mainLayout->addWidget(modeGroup);

  // ---- Threshold Section ----
  thresholdSection_ = new QWidget();
  QGroupBox* thresholdGroup = new QGroupBox(tr("Threshold Bounds"));
  QVBoxLayout* thresholdOuterLayout = new QVBoxLayout(thresholdSection_);
  thresholdOuterLayout->setContentsMargins(0, 0, 0, 0);

  QGridLayout* threshLayout = new QGridLayout(thresholdGroup);
  threshLayout->addWidget(new QLabel(tr("Upper:")), 0, 0);
  thresholdUpperSpin_ = new QDoubleSpinBox();
  thresholdUpperSpin_->setRange(-1e9, 1e9);
  thresholdUpperSpin_->setDecimals(3);
  thresholdUpperSpin_->setValue(config.thresholdUpper);
  threshLayout->addWidget(thresholdUpperSpin_, 0, 1);

  threshLayout->addWidget(new QLabel(tr("Lower:")), 1, 0);
  thresholdLowerSpin_ = new QDoubleSpinBox();
  thresholdLowerSpin_->setRange(-1e9, 1e9);
  thresholdLowerSpin_->setDecimals(3);
  thresholdLowerSpin_->setValue(config.thresholdLower);
  threshLayout->addWidget(thresholdLowerSpin_, 1, 1);

  threshLayout->addWidget(new QLabel(tr("Alarm color:")), 2, 0);
  alarmColorButton_ = new QPushButton();
  alarmColorButton_->setFixedSize(40, 24);
  alarmColor_ = config.thresholdAlarmColor;
  alarmColorButton_->setStyleSheet(
    QString("background-color: %1; border: 1px solid #555;").arg(alarmColor_.name()));
  connect(alarmColorButton_, &QPushButton::clicked, [this]() {
    chooseColor(alarmColorButton_, alarmColor_);
  });
  threshLayout->addWidget(alarmColorButton_, 2, 1);

  thresholdOuterLayout->addWidget(thresholdGroup);
  mainLayout->addWidget(thresholdSection_);

  // ---- Gradient Section ----
  gradientSection_ = new QWidget();
  QGroupBox* gradientGroup = new QGroupBox(tr("Value Gradient"));
  QVBoxLayout* gradientOuterLayout = new QVBoxLayout(gradientSection_);
  gradientOuterLayout->setContentsMargins(0, 0, 0, 0);

  QGridLayout* gradLayout = new QGridLayout(gradientGroup);
  gradLayout->addWidget(new QLabel(tr("Min value:")), 0, 0);
  gradientMinSpin_ = new QDoubleSpinBox();
  gradientMinSpin_->setRange(-1e9, 1e9);
  gradientMinSpin_->setDecimals(3);
  gradientMinSpin_->setValue(config.gradientMinValue);
  gradLayout->addWidget(gradientMinSpin_, 0, 1);

  gradLayout->addWidget(new QLabel(tr("Max value:")), 1, 0);
  gradientMaxSpin_ = new QDoubleSpinBox();
  gradientMaxSpin_->setRange(-1e9, 1e9);
  gradientMaxSpin_->setDecimals(3);
  gradientMaxSpin_->setValue(config.gradientMaxValue);
  gradLayout->addWidget(gradientMaxSpin_, 1, 1);

  gradLayout->addWidget(new QLabel(tr("Low color:")), 2, 0);
  gradientLowColorButton_ = new QPushButton();
  gradientLowColorButton_->setFixedSize(40, 24);
  gradientLowColor_ = config.gradientColorLow;
  gradientLowColorButton_->setStyleSheet(
    QString("background-color: %1; border: 1px solid #555;").arg(gradientLowColor_.name()));
  connect(gradientLowColorButton_, &QPushButton::clicked, [this]() {
    chooseColor(gradientLowColorButton_, gradientLowColor_);
  });
  gradLayout->addWidget(gradientLowColorButton_, 2, 1);

  gradLayout->addWidget(new QLabel(tr("High color:")), 3, 0);
  gradientHighColorButton_ = new QPushButton();
  gradientHighColorButton_->setFixedSize(40, 24);
  gradientHighColor_ = config.gradientColorHigh;
  gradientHighColorButton_->setStyleSheet(
    QString("background-color: %1; border: 1px solid #555;").arg(gradientHighColor_.name()));
  connect(gradientHighColorButton_, &QPushButton::clicked, [this]() {
    chooseColor(gradientHighColorButton_, gradientHighColor_);
  });
  gradLayout->addWidget(gradientHighColorButton_, 3, 1);

  gradLayout->addWidget(new QLabel(tr("Steps:")), 4, 0);
  gradientBucketsSpin_ = new QSpinBox();
  gradientBucketsSpin_->setRange(constants::plot::MIN_GRADIENT_BUCKETS,
                                  constants::plot::MAX_GRADIENT_BUCKETS);
  gradientBucketsSpin_->setValue(config.gradientBuckets);
  gradLayout->addWidget(gradientBucketsSpin_, 4, 1);

  gradientOuterLayout->addWidget(gradientGroup);
  mainLayout->addWidget(gradientSection_);

  // ---- Buttons ----
  QDialogButtonBox* buttons = new QDialogButtonBox(
    QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
  connect(buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
  connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
  mainLayout->addWidget(buttons);

  // Wire mode toggles to section visibility
  connect(modeGroup_, QOverload<int>::of(&QButtonGroup::buttonClicked),
          this, [this](int) { updateSectionVisibility(); });

  updateSectionVisibility();
}

void PlotSeriesConfigDialog::updateSectionVisibility() {
  thresholdSection_->setVisible(thresholdRadio_->isChecked());
  gradientSection_->setVisible(gradientRadio_->isChecked());
  adjustSize();
}

void PlotSeriesConfigDialog::chooseColor(QPushButton* btn, QColor& target) {
  QColor c = QColorDialog::getColor(target, this, tr("Select Color"));
  if (c.isValid()) {
    target = c;
    btn->setStyleSheet(
      QString("background-color: %1; border: 1px solid #555;").arg(c.name()));
  }
}

PlotSeriesConfig PlotSeriesConfigDialog::result() const {
  PlotSeriesConfig cfg;
  cfg.color = selectedColor_;
  cfg.lineThickness = thicknessSpin_->value();

  cfg.decimationEnabled = decimationCheck_->isChecked();
  cfg.maxSampleRateHz = decimationCheck_->isChecked() ? sampleRateSpin_->value() : 0.0;

  if (thresholdRadio_->isChecked()) {
    cfg.renderMode = PlotRenderMode::Threshold;
  } else if (gradientRadio_->isChecked()) {
    cfg.renderMode = PlotRenderMode::Gradient;
  } else {
    cfg.renderMode = PlotRenderMode::Solid;
  }

  cfg.thresholdUpper = thresholdUpperSpin_->value();
  cfg.thresholdLower = thresholdLowerSpin_->value();
  cfg.thresholdAlarmColor = alarmColor_;

  cfg.gradientMinValue = gradientMinSpin_->value();
  cfg.gradientMaxValue = gradientMaxSpin_->value();
  cfg.gradientColorLow = gradientLowColor_;
  cfg.gradientColorHigh = gradientHighColor_;
  cfg.gradientBuckets = gradientBucketsSpin_->value();

  return cfg;
}

}  // namespace ros_weaver

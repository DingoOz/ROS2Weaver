#include "ros_weaver/widgets/typing_indicator.hpp"
#include "ros_weaver/core/theme_manager.hpp"

#include <QPainter>
#include <QPainterPath>
#include <cmath>

namespace ros_weaver {

// =============================================================================
// BouncingDot
// =============================================================================

BouncingDot::BouncingDot(QWidget* parent)
    : QWidget(parent)
    , bounceOffset_(0.0)
    , color_(QColor(76, 175, 80))  // Green by default
    , dotSize_(8) {
  setFixedSize(dotSize_ + 4, dotSize_ * 3);  // Extra height for bounce
}

void BouncingDot::setBounceOffset(qreal offset) {
  bounceOffset_ = offset;
  update();
}

void BouncingDot::setColor(const QColor& color) {
  color_ = color;
  update();
}

void BouncingDot::setDotSize(int size) {
  dotSize_ = size;
  setFixedSize(size + 4, size * 3);
  update();
}

void BouncingDot::paintEvent(QPaintEvent* event) {
  Q_UNUSED(event)

  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);

  // Calculate vertical position with bounce
  qreal centerY = height() / 2.0 + bounceOffset_;
  qreal centerX = width() / 2.0;

  // Draw dot with slight glow
  painter.setPen(Qt::NoPen);

  // Glow effect
  QColor glowColor = color_;
  glowColor.setAlpha(50);
  painter.setBrush(glowColor);
  painter.drawEllipse(QPointF(centerX, centerY), dotSize_ / 2.0 + 2, dotSize_ / 2.0 + 2);

  // Main dot
  painter.setBrush(color_);
  painter.drawEllipse(QPointF(centerX, centerY), dotSize_ / 2.0, dotSize_ / 2.0);
}

// =============================================================================
// TypingIndicator
// =============================================================================

TypingIndicator::TypingIndicator(QWidget* parent)
    : QWidget(parent)
    , textLabel_(nullptr)
    , elapsedLabel_(nullptr)
    , stopButton_(nullptr)
    , dotsContainer_(nullptr)
    , animationTimer_(nullptr)
    , elapsedTimer_(nullptr)
    , isAnimating_(false)
    , animationPhase_(0)
    , showElapsedTime_(true)
    , showStopButton_(true) {
  setupUi();

  animationTimer_ = new QTimer(this);
  connect(animationTimer_, &QTimer::timeout, this, &TypingIndicator::updateAnimation);

  elapsedTimer_ = new QTimer(this);
  connect(elapsedTimer_, &QTimer::timeout, this, &TypingIndicator::updateElapsedTime);
}

TypingIndicator::~TypingIndicator() {
  stop();
}

void TypingIndicator::setupUi() {
  auto& theme = ThemeManager::instance();

  QHBoxLayout* mainLayout = new QHBoxLayout(this);
  mainLayout->setContentsMargins(12, 8, 12, 8);
  mainLayout->setSpacing(8);

  // Container widget with background using theme colors
  QWidget* container = new QWidget(this);
  container->setStyleSheet(QString(
      "background-color: %1;"
      "border-radius: 12px;"
      "border: 1px solid %2;")
      .arg(theme.elevatedSurfaceColor().name())
      .arg(theme.borderColor().name()));

  QHBoxLayout* containerLayout = new QHBoxLayout(container);
  containerLayout->setContentsMargins(16, 10, 16, 10);
  containerLayout->setSpacing(12);

  // AI icon
  QLabel* iconLabel = new QLabel(container);
  iconLabel->setText(QString::fromUtf8("\xF0\x9F\xA4\x96")); // Robot emoji
  iconLabel->setStyleSheet("font-size: 16px; background: transparent;");
  containerLayout->addWidget(iconLabel);

  // Text label using theme colors
  textLabel_ = new QLabel(tr("AI is thinking"), container);
  textLabel_->setStyleSheet(QString(
      "color: %1;"
      "font-size: 13px;"
      "font-weight: 500;"
      "background: transparent;")
      .arg(theme.successColor().name()));
  containerLayout->addWidget(textLabel_);

  // Bouncing dots container
  dotsContainer_ = new QWidget(container);
  dotsContainer_->setStyleSheet("background: transparent;");
  QHBoxLayout* dotsLayout = new QHBoxLayout(dotsContainer_);
  dotsLayout->setContentsMargins(0, 0, 0, 0);
  dotsLayout->setSpacing(4);

  for (int i = 0; i < DOT_COUNT; ++i) {
    BouncingDot* dot = new BouncingDot(dotsContainer_);
    dot->setColor(theme.successColor());
    dots_.append(dot);
    dotsLayout->addWidget(dot);
  }

  containerLayout->addWidget(dotsContainer_);

  // Elapsed time label
  elapsedLabel_ = new QLabel(container);
  elapsedLabel_->setStyleSheet(QString(
      "color: %1;"
      "font-size: 11px;"
      "background: transparent;")
      .arg(theme.textSecondaryColor().name()));
  elapsedLabel_->setVisible(showElapsedTime_);
  containerLayout->addWidget(elapsedLabel_);

  containerLayout->addStretch();

  // Stop button
  stopButton_ = new QPushButton(tr("Stop"), container);
  stopButton_->setFixedHeight(24);
  stopButton_->setCursor(Qt::PointingHandCursor);
  stopButton_->setStyleSheet(
      "QPushButton {"
      "  background-color: #5a3d3d;"
      "  border: 1px solid #6a4d4d;"
      "  border-radius: 4px;"
      "  color: #f0b0b0;"
      "  font-size: 11px;"
      "  padding: 2px 12px;"
      "}"
      "QPushButton:hover {"
      "  background-color: #6a4d4d;"
      "  border-color: #7a5d5d;"
      "}");
  stopButton_->setVisible(showStopButton_);
  connect(stopButton_, &QPushButton::clicked, this, &TypingIndicator::stopRequested);
  containerLayout->addWidget(stopButton_);

  mainLayout->addWidget(container);
}

void TypingIndicator::start() {
  if (isAnimating_) return;

  isAnimating_ = true;
  animationPhase_ = 0;
  elapsedClock_.start();

  // Reset elapsed label
  elapsedLabel_->setText("");
  elapsedLabel_->setVisible(false);

  animationTimer_->start(ANIMATION_INTERVAL_MS);
  elapsedTimer_->start(ELAPSED_UPDATE_INTERVAL_MS);

  show();
}

void TypingIndicator::stop() {
  if (!isAnimating_) return;

  isAnimating_ = false;
  animationTimer_->stop();
  elapsedTimer_->stop();

  // Reset dots
  for (BouncingDot* dot : dots_) {
    dot->setBounceOffset(0);
  }

  hide();
}

void TypingIndicator::setLabel(const QString& text) {
  textLabel_->setText(text);
}

void TypingIndicator::setShowElapsedTime(bool show) {
  showElapsedTime_ = show;
  elapsedLabel_->setVisible(show && elapsedClock_.elapsed() >= 5000);
}

void TypingIndicator::setShowStopButton(bool show) {
  showStopButton_ = show;
  stopButton_->setVisible(show);
}

void TypingIndicator::updateAnimation() {
  // Animate each dot with a phase offset
  for (int i = 0; i < dots_.size(); ++i) {
    // Calculate bounce using sine wave with phase offset
    int phase = (animationPhase_ + i * 2) % 12;
    qreal bounce = 0;

    if (phase < 6) {
      // Bounce up and down
      bounce = -6.0 * std::sin(phase * M_PI / 6.0);
    }

    dots_[i]->setBounceOffset(bounce);
  }

  animationPhase_ = (animationPhase_ + 1) % 12;
}

void TypingIndicator::updateElapsedTime() {
  if (!showElapsedTime_) return;

  qint64 elapsed = elapsedClock_.elapsed();

  // Only show after 5 seconds
  if (elapsed >= 5000) {
    elapsedLabel_->setVisible(true);

    int seconds = static_cast<int>(elapsed / 1000);
    if (seconds < 60) {
      elapsedLabel_->setText(QString("%1s").arg(seconds));
    } else {
      int minutes = seconds / 60;
      seconds = seconds % 60;
      elapsedLabel_->setText(QString("%1m %2s").arg(minutes).arg(seconds));
    }
  }
}

}  // namespace ros_weaver

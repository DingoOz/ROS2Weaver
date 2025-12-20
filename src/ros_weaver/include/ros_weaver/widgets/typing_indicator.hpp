#ifndef ROS_WEAVER_TYPING_INDICATOR_HPP
#define ROS_WEAVER_TYPING_INDICATOR_HPP

#include <QWidget>
#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <QElapsedTimer>
#include <QHBoxLayout>
#include <QVBoxLayout>

namespace ros_weaver {

/**
 * @brief Animated dot used in the typing indicator
 */
class BouncingDot : public QWidget {
  Q_OBJECT
  Q_PROPERTY(qreal bounceOffset READ bounceOffset WRITE setBounceOffset)

public:
  explicit BouncingDot(QWidget* parent = nullptr);

  qreal bounceOffset() const { return bounceOffset_; }
  void setBounceOffset(qreal offset);

  void setColor(const QColor& color);
  void setDotSize(int size);

protected:
  void paintEvent(QPaintEvent* event) override;

private:
  qreal bounceOffset_;
  QColor color_;
  int dotSize_;
};

/**
 * @brief Animated typing indicator with bouncing dots
 *
 * Shows "AI is thinking" with animated bouncing dots,
 * elapsed time display, and stop button.
 */
class TypingIndicator : public QWidget {
  Q_OBJECT

public:
  explicit TypingIndicator(QWidget* parent = nullptr);
  ~TypingIndicator() override;

  void start();
  void stop();
  bool isAnimating() const { return isAnimating_; }

  void setLabel(const QString& text);
  void setShowElapsedTime(bool show);
  void setShowStopButton(bool show);

signals:
  void stopRequested();

private slots:
  void updateAnimation();
  void updateElapsedTime();

private:
  void setupUi();

  QLabel* textLabel_;
  QLabel* elapsedLabel_;
  QPushButton* stopButton_;
  QWidget* dotsContainer_;
  QList<BouncingDot*> dots_;

  QTimer* animationTimer_;
  QTimer* elapsedTimer_;
  QElapsedTimer elapsedClock_;

  bool isAnimating_;
  int animationPhase_;
  bool showElapsedTime_;
  bool showStopButton_;

  static constexpr int DOT_COUNT = 3;
  static constexpr int ANIMATION_INTERVAL_MS = 150;
  static constexpr int ELAPSED_UPDATE_INTERVAL_MS = 1000;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_TYPING_INDICATOR_HPP

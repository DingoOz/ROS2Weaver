#ifndef ROS_WEAVER_WIDGETS_ROLLING_DIGIT_LABEL_HPP
#define ROS_WEAVER_WIDGETS_ROLLING_DIGIT_LABEL_HPP

#include <QWidget>
#include <QPropertyAnimation>
#include <QFont>
#include <QColor>

namespace ros_weaver {

/**
 * @brief A label widget that animates digit transitions with a rolling/odometer effect
 *
 * When the displayed value changes, digits "roll" through intermediate values
 * instead of jumping instantly, creating a slot-machine/odometer visual effect.
 * Non-digit characters (decimal points, units) change instantly.
 */
class RollingDigitLabel : public QWidget {
  Q_OBJECT
  Q_PROPERTY(qreal animationProgress READ animationProgress WRITE setAnimationProgress)

public:
  explicit RollingDigitLabel(QWidget* parent = nullptr);
  ~RollingDigitLabel() override;

  /**
   * @brief Set the value to display (e.g., "1.25 MB/s", "45.2 Hz")
   * Triggers animation from current value to new value
   */
  void setValue(const QString& formattedValue);

  /**
   * @brief Get the current displayed value
   */
  QString value() const { return targetValue_; }

  /**
   * @brief Set the text color
   */
  void setTextColor(const QColor& color);
  QColor textColor() const { return textColor_; }

  /**
   * @brief Set the font for rendering
   */
  void setFont(const QFont& font);

  /**
   * @brief Set animation duration in milliseconds (default 300ms)
   */
  void setAnimationDuration(int ms);
  int animationDuration() const { return animationDuration_; }

  /**
   * @brief Enable/disable animations (useful for rapid updates)
   */
  void setAnimationsEnabled(bool enabled);
  bool animationsEnabled() const { return animationsEnabled_; }

  // Animation property accessors
  qreal animationProgress() const { return animationProgress_; }
  void setAnimationProgress(qreal progress);

  QSize sizeHint() const override;
  QSize minimumSizeHint() const override;

protected:
  void paintEvent(QPaintEvent* event) override;

private:
  void startAnimation();
  void stopAnimation();

  /**
   * @brief Calculate the shortest scroll distance between two digits
   * e.g., 3→7 = +4, 7→3 = -4 (not +6), 9→1 = +2 (wrap around)
   */
  int digitScrollDistance(int fromDigit, int toDigit) const;

  /**
   * @brief Get the digit width using the current font (monospace-style)
   */
  qreal digitWidth() const;

  QString currentValue_;      // Value at animation start
  QString targetValue_;       // Value we're animating to
  QString displayValue_;      // Currently displayed (for size calculations)
  QColor textColor_;
  QFont font_;
  int animationDuration_;
  qreal animationProgress_;   // 0.0 to 1.0
  bool animationsEnabled_;
  QPropertyAnimation* animation_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_WIDGETS_ROLLING_DIGIT_LABEL_HPP

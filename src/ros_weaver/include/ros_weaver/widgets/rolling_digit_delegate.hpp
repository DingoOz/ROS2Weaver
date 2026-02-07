#ifndef ROS_WEAVER_WIDGETS_ROLLING_DIGIT_DELEGATE_HPP
#define ROS_WEAVER_WIDGETS_ROLLING_DIGIT_DELEGATE_HPP

#include <QStyledItemDelegate>
#include <QPropertyAnimation>
#include <QMap>
#include <QTimer>

namespace ros_weaver {

/**
 * @brief Custom delegate that renders rate values with rolling digit animation
 *
 * When rate values change in the model, this delegate animates the transition
 * with an odometer-style rolling effect on the digits.
 */
class RollingDigitDelegate : public QStyledItemDelegate {
  Q_OBJECT

public:
  explicit RollingDigitDelegate(QObject* parent = nullptr);
  ~RollingDigitDelegate() override;

  void paint(QPainter* painter, const QStyleOptionViewItem& option,
             const QModelIndex& index) const override;

  QSize sizeHint(const QStyleOptionViewItem& option,
                 const QModelIndex& index) const override;

  /**
   * @brief Notify the delegate that a rate value has changed
   * This triggers the rolling animation for that row
   */
  void notifyRateChanged(int row, double oldRate, double newRate);

  /**
   * @brief Set animation duration in milliseconds (default 300ms)
   */
  void setAnimationDuration(int ms) { animationDuration_ = ms; }

  /**
   * @brief Clear all animation state (call when model is reset)
   */
  void clearAnimations();

signals:
  /**
   * @brief Emitted when a cell needs repainting during animation
   */
  void needsRepaint(int row) const;

private slots:
  void onAnimationValueChanged(const QVariant& value);
  void onAnimationFinished();

private:
  struct AnimationState {
    QString oldValue;
    QString newValue;
    qreal progress = 1.0;  // 0.0 to 1.0
    QPropertyAnimation* animation = nullptr;
  };

  QString formatRate(double rate) const;
  int digitScrollDistance(int fromDigit, int toDigit) const;
  void drawRollingText(QPainter* painter, const QRect& rect,
                       const QString& oldValue, const QString& newValue,
                       qreal progress, const QColor& textColor) const;

  mutable QMap<int, AnimationState> animations_;  // row -> state
  int animationDuration_ = 300;
  QColor activeColor_ = QColor(100, 200, 100);
  QColor inactiveColor_ = QColor(128, 128, 128);
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_WIDGETS_ROLLING_DIGIT_DELEGATE_HPP

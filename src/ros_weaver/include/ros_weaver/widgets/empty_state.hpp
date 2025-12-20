#ifndef ROS_WEAVER_EMPTY_STATE_HPP
#define ROS_WEAVER_EMPTY_STATE_HPP

#include <QWidget>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <functional>

namespace ros_weaver {

/**
 * @brief Widget displayed when a panel or view has no content
 *
 * Provides helpful guidance and optional action buttons
 * to help users understand what to do.
 */
class EmptyState : public QWidget {
  Q_OBJECT

public:
  explicit EmptyState(QWidget* parent = nullptr);
  ~EmptyState() override = default;

  // Configuration
  void setIcon(const QString& iconText);  // Emoji or unicode character
  void setTitle(const QString& title);
  void setDescription(const QString& description);

  // Add action buttons
  void addAction(const QString& text, std::function<void()> callback);
  void clearActions();

  // Predefined empty states
  static EmptyState* createCanvasEmpty(QWidget* parent = nullptr);
  static EmptyState* createParamsEmpty(QWidget* parent = nullptr);
  static EmptyState* createLogsEmpty(QWidget* parent = nullptr);
  static EmptyState* createTopicsEmpty(QWidget* parent = nullptr);
  static EmptyState* createPlotEmpty(QWidget* parent = nullptr);
  static EmptyState* createSearchEmpty(QWidget* parent = nullptr);

private:
  void setupUi();
  void updateStyle();

  QLabel* iconLabel_;
  QLabel* titleLabel_;
  QLabel* descriptionLabel_;
  QWidget* actionsContainer_;
  QVBoxLayout* actionsLayout_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_EMPTY_STATE_HPP

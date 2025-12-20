#include "ros_weaver/widgets/empty_state.hpp"
#include "ros_weaver/core/theme_manager.hpp"

namespace ros_weaver {

EmptyState::EmptyState(QWidget* parent)
    : QWidget(parent)
    , iconLabel_(nullptr)
    , titleLabel_(nullptr)
    , descriptionLabel_(nullptr)
    , actionsContainer_(nullptr)
    , actionsLayout_(nullptr) {
  setupUi();
}

void EmptyState::setupUi() {
  QVBoxLayout* layout = new QVBoxLayout(this);
  layout->setAlignment(Qt::AlignCenter);
  layout->setSpacing(12);
  layout->setContentsMargins(40, 40, 40, 40);

  // Icon (emoji)
  iconLabel_ = new QLabel(this);
  iconLabel_->setAlignment(Qt::AlignCenter);
  iconLabel_->setStyleSheet("font-size: 48px; background: transparent;");
  layout->addWidget(iconLabel_);

  // Title
  titleLabel_ = new QLabel(this);
  titleLabel_->setAlignment(Qt::AlignCenter);
  titleLabel_->setWordWrap(true);
  layout->addWidget(titleLabel_);

  // Description
  descriptionLabel_ = new QLabel(this);
  descriptionLabel_->setAlignment(Qt::AlignCenter);
  descriptionLabel_->setWordWrap(true);
  layout->addWidget(descriptionLabel_);

  // Actions container
  actionsContainer_ = new QWidget(this);
  actionsLayout_ = new QVBoxLayout(actionsContainer_);
  actionsLayout_->setAlignment(Qt::AlignCenter);
  actionsLayout_->setSpacing(8);
  actionsLayout_->setContentsMargins(0, 12, 0, 0);
  layout->addWidget(actionsContainer_);

  updateStyle();
}

void EmptyState::updateStyle() {
  auto& theme = ThemeManager::instance();

  titleLabel_->setStyleSheet(QString(
      "font-size: 16px;"
      "font-weight: bold;"
      "color: %1;"
      "background: transparent;")
      .arg(theme.textPrimaryColor().name()));

  descriptionLabel_->setStyleSheet(QString(
      "font-size: 13px;"
      "color: %1;"
      "background: transparent;")
      .arg(theme.textSecondaryColor().name()));
}

void EmptyState::setIcon(const QString& iconText) {
  iconLabel_->setText(iconText);
}

void EmptyState::setTitle(const QString& title) {
  titleLabel_->setText(title);
}

void EmptyState::setDescription(const QString& description) {
  descriptionLabel_->setText(description);
}

void EmptyState::addAction(const QString& text, std::function<void()> callback) {
  auto& theme = ThemeManager::instance();

  QPushButton* button = new QPushButton(text, actionsContainer_);
  button->setStyleSheet(theme.primaryButtonStyle());
  button->setCursor(Qt::PointingHandCursor);

  connect(button, &QPushButton::clicked, this, [callback]() {
    if (callback) callback();
  });

  actionsLayout_->addWidget(button);
}

void EmptyState::clearActions() {
  QLayoutItem* item;
  while ((item = actionsLayout_->takeAt(0)) != nullptr) {
    delete item->widget();
    delete item;
  }
}

// =============================================================================
// Predefined Empty States
// =============================================================================

EmptyState* EmptyState::createCanvasEmpty(QWidget* parent) {
  EmptyState* state = new EmptyState(parent);
  state->setIcon(QString::fromUtf8("\xF0\x9F\x93\xA6")); // Package emoji
  state->setTitle(QObject::tr("Start Building Your ROS2 System"));
  state->setDescription(QObject::tr(
      "Drag packages from the browser panel on the left,\n"
      "or use the templates to get started quickly."));
  return state;
}

EmptyState* EmptyState::createParamsEmpty(QWidget* parent) {
  EmptyState* state = new EmptyState(parent);
  state->setIcon(QString::fromUtf8("\xE2\x9A\x99\xEF\xB8\x8F")); // Gear emoji
  state->setTitle(QObject::tr("No Node Selected"));
  state->setDescription(QObject::tr(
      "Select a node on the canvas to view and edit its parameters."));
  return state;
}

EmptyState* EmptyState::createLogsEmpty(QWidget* parent) {
  EmptyState* state = new EmptyState(parent);
  state->setIcon(QString::fromUtf8("\xF0\x9F\x93\x9C")); // Scroll emoji
  state->setTitle(QObject::tr("No Logs Yet"));
  state->setDescription(QObject::tr(
      "Click 'Start Listening' to begin capturing ROS2 logs\n"
      "from running nodes."));
  return state;
}

EmptyState* EmptyState::createTopicsEmpty(QWidget* parent) {
  EmptyState* state = new EmptyState(parent);
  state->setIcon(QString::fromUtf8("\xF0\x9F\x93\xA1")); // Satellite antenna
  state->setTitle(QObject::tr("No Topics Found"));
  state->setDescription(QObject::tr(
      "No ROS2 topics are currently active.\n"
      "Start some nodes to see their topics here."));
  return state;
}

EmptyState* EmptyState::createPlotEmpty(QWidget* parent) {
  EmptyState* state = new EmptyState(parent);
  state->setIcon(QString::fromUtf8("\xF0\x9F\x93\x88")); // Chart emoji
  state->setTitle(QObject::tr("No Data to Plot"));
  state->setDescription(QObject::tr(
      "Add topics to visualize their data over time.\n"
      "Click '+ Add Topic' to get started."));
  return state;
}

EmptyState* EmptyState::createSearchEmpty(QWidget* parent) {
  EmptyState* state = new EmptyState(parent);
  state->setIcon(QString::fromUtf8("\xF0\x9F\x94\x8D")); // Magnifying glass
  state->setTitle(QObject::tr("No Results Found"));
  state->setDescription(QObject::tr(
      "Try adjusting your search terms or filters."));
  return state;
}

}  // namespace ros_weaver

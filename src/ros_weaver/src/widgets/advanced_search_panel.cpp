#include "ros_weaver/widgets/advanced_search_panel.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QTimer>

namespace ros_weaver {

AdvancedSearchPanel::AdvancedSearchPanel(QWidget* parent)
    : QWidget(parent)
{
  setupUI();
  populateCategories();
  populateMessageTypes();
}

void AdvancedSearchPanel::setupUI() {
  QVBoxLayout* mainLayout = new QVBoxLayout(this);
  mainLayout->setContentsMargins(8, 8, 8, 8);
  mainLayout->setSpacing(8);

  // Search box
  searchEdit_ = new QLineEdit(this);
  searchEdit_->setPlaceholderText(tr("Search packages, nodes, message types..."));
  searchEdit_->setClearButtonEnabled(true);
  connect(searchEdit_, &QLineEdit::textChanged,
          this, &AdvancedSearchPanel::onSearchTextChanged);
  mainLayout->addWidget(searchEdit_);

  // Filters group
  QGroupBox* filtersGroup = new QGroupBox(tr("Filters"), this);
  QFormLayout* filtersLayout = new QFormLayout(filtersGroup);
  filtersLayout->setContentsMargins(8, 12, 8, 8);

  categoryCombo_ = new QComboBox(this);
  connect(categoryCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &AdvancedSearchPanel::onCategoryChanged);
  filtersLayout->addRow(tr("Category:"), categoryCombo_);

  messageTypeCombo_ = new QComboBox(this);
  connect(messageTypeCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &AdvancedSearchPanel::onMessageTypeChanged);
  filtersLayout->addRow(tr("Message Type:"), messageTypeCombo_);

  // Feature checkboxes
  QHBoxLayout* featuresLayout = new QHBoxLayout();
  hasInputsCheck_ = new QCheckBox(tr("Has Inputs"), this);
  hasOutputsCheck_ = new QCheckBox(tr("Has Outputs"), this);
  hasServicesCheck_ = new QCheckBox(tr("Has Services"), this);
  hasActionsCheck_ = new QCheckBox(tr("Has Actions"), this);

  connect(hasInputsCheck_, &QCheckBox::toggled, this, &AdvancedSearchPanel::updateFilters);
  connect(hasOutputsCheck_, &QCheckBox::toggled, this, &AdvancedSearchPanel::updateFilters);
  connect(hasServicesCheck_, &QCheckBox::toggled, this, &AdvancedSearchPanel::updateFilters);
  connect(hasActionsCheck_, &QCheckBox::toggled, this, &AdvancedSearchPanel::updateFilters);

  featuresLayout->addWidget(hasInputsCheck_);
  featuresLayout->addWidget(hasOutputsCheck_);
  featuresLayout->addWidget(hasServicesCheck_);
  featuresLayout->addWidget(hasActionsCheck_);
  featuresLayout->addStretch();

  filtersLayout->addRow(tr("Features:"), featuresLayout);

  mainLayout->addWidget(filtersGroup);

  // Results list
  resultsList_ = new QListWidget(this);
  resultsList_->setSelectionMode(QAbstractItemView::SingleSelection);
  resultsList_->setAlternatingRowColors(true);
  connect(resultsList_, &QListWidget::itemClicked,
          this, &AdvancedSearchPanel::onResultClicked);
  connect(resultsList_, &QListWidget::itemDoubleClicked,
          this, &AdvancedSearchPanel::onResultDoubleClicked);
  mainLayout->addWidget(resultsList_, 1);

  // Bottom bar with add button and status
  QHBoxLayout* bottomLayout = new QHBoxLayout();

  statusLabel_ = new QLabel(tr("0 results"), this);
  bottomLayout->addWidget(statusLabel_);

  bottomLayout->addStretch();

  addButton_ = new QPushButton(tr("Add to Canvas"), this);
  addButton_->setEnabled(false);
  connect(addButton_, &QPushButton::clicked, this, &AdvancedSearchPanel::onAddClicked);
  bottomLayout->addWidget(addButton_);

  mainLayout->addLayout(bottomLayout);
}

void AdvancedSearchPanel::populateCategories() {
  categoryCombo_->clear();
  categoryCombo_->addItem(tr("All Categories"), "");
  categoryCombo_->addItem(tr("Navigation"), "navigation");
  categoryCombo_->addItem(tr("Perception"), "perception");
  categoryCombo_->addItem(tr("Manipulation"), "manipulation");
  categoryCombo_->addItem(tr("SLAM"), "slam");
  categoryCombo_->addItem(tr("Control"), "control");
  categoryCombo_->addItem(tr("Drivers"), "drivers");
  categoryCombo_->addItem(tr("Simulation"), "simulation");
  categoryCombo_->addItem(tr("Utilities"), "utilities");
  categoryCombo_->addItem(tr("Custom"), "custom");
}

void AdvancedSearchPanel::populateMessageTypes() {
  messageTypeCombo_->clear();
  messageTypeCombo_->addItem(tr("All Message Types"), "");
  messageTypeCombo_->addItem("geometry_msgs/Twist", "geometry_msgs/Twist");
  messageTypeCombo_->addItem("geometry_msgs/PoseStamped", "geometry_msgs/PoseStamped");
  messageTypeCombo_->addItem("sensor_msgs/LaserScan", "sensor_msgs/LaserScan");
  messageTypeCombo_->addItem("sensor_msgs/Image", "sensor_msgs/Image");
  messageTypeCombo_->addItem("sensor_msgs/PointCloud2", "sensor_msgs/PointCloud2");
  messageTypeCombo_->addItem("sensor_msgs/JointState", "sensor_msgs/JointState");
  messageTypeCombo_->addItem("nav_msgs/Odometry", "nav_msgs/Odometry");
  messageTypeCombo_->addItem("nav_msgs/Path", "nav_msgs/Path");
  messageTypeCombo_->addItem("nav_msgs/OccupancyGrid", "nav_msgs/OccupancyGrid");
  messageTypeCombo_->addItem("tf2_msgs/TFMessage", "tf2_msgs/TFMessage");
  messageTypeCombo_->addItem("std_msgs/String", "std_msgs/String");
}

void AdvancedSearchPanel::setPackages(const QStringList& packages) {
  allPackages_ = packages;

  // Build search results from packages
  allResults_.clear();
  for (const QString& pkg : packages) {
    SearchResult result;
    result.name = pkg;
    result.packageName = pkg;

    // Categorize based on name
    if (pkg.contains("nav", Qt::CaseInsensitive)) {
      result.category = "navigation";
    } else if (pkg.contains("slam", Qt::CaseInsensitive) ||
               pkg.contains("map", Qt::CaseInsensitive)) {
      result.category = "slam";
    } else if (pkg.contains("camera", Qt::CaseInsensitive) ||
               pkg.contains("image", Qt::CaseInsensitive) ||
               pkg.contains("vision", Qt::CaseInsensitive)) {
      result.category = "perception";
    } else if (pkg.contains("arm", Qt::CaseInsensitive) ||
               pkg.contains("moveit", Qt::CaseInsensitive) ||
               pkg.contains("gripper", Qt::CaseInsensitive)) {
      result.category = "manipulation";
    } else if (pkg.contains("driver", Qt::CaseInsensitive) ||
               pkg.contains("lidar", Qt::CaseInsensitive) ||
               pkg.contains("sensor", Qt::CaseInsensitive)) {
      result.category = "drivers";
    } else if (pkg.contains("gazebo", Qt::CaseInsensitive) ||
               pkg.contains("sim", Qt::CaseInsensitive)) {
      result.category = "simulation";
    } else if (pkg.contains("control", Qt::CaseInsensitive)) {
      result.category = "control";
    } else {
      result.category = "utilities";
    }

    allResults_.append(result);
  }

  performSearch();
}

void AdvancedSearchPanel::clear() {
  searchEdit_->clear();
  categoryCombo_->setCurrentIndex(0);
  messageTypeCombo_->setCurrentIndex(0);
  hasInputsCheck_->setChecked(false);
  hasOutputsCheck_->setChecked(false);
  hasServicesCheck_->setChecked(false);
  hasActionsCheck_->setChecked(false);
  resultsList_->clear();
  results_.clear();
  addButton_->setEnabled(false);
  statusLabel_->setText(tr("0 results"));
}

void AdvancedSearchPanel::onSearchTextChanged(const QString& text) {
  currentQuery_ = text;
  // Use a small delay to avoid searching on every keystroke
  QTimer::singleShot(200, this, &AdvancedSearchPanel::performSearch);
}

void AdvancedSearchPanel::onCategoryChanged(int index) {
  currentCategory_ = categoryCombo_->itemData(index).toString();
  performSearch();
}

void AdvancedSearchPanel::onMessageTypeChanged(int index) {
  currentMessageType_ = messageTypeCombo_->itemData(index).toString();
  performSearch();
}

void AdvancedSearchPanel::updateFilters() {
  performSearch();
}

void AdvancedSearchPanel::performSearch() {
  results_.clear();
  resultsList_->clear();

  QString query = currentQuery_.toLower();

  for (const SearchResult& result : allResults_) {
    // Text search
    if (!query.isEmpty()) {
      bool matches = result.name.toLower().contains(query) ||
                     result.packageName.toLower().contains(query) ||
                     result.description.toLower().contains(query);
      if (!matches) {
        // Check message types
        for (const QString& msgType : result.messageTypes) {
          if (msgType.toLower().contains(query)) {
            matches = true;
            break;
          }
        }
      }
      if (!matches) continue;
    }

    // Category filter
    if (!currentCategory_.isEmpty() && result.category != currentCategory_) {
      continue;
    }

    // Message type filter
    if (!currentMessageType_.isEmpty()) {
      bool hasType = result.messageTypes.contains(currentMessageType_);
      if (!hasType) continue;
    }

    // Feature filters (if any checked, result must have that feature)
    if (!matchesFilters(result)) {
      continue;
    }

    results_.append(result);
    addResultToList(result);
  }

  statusLabel_->setText(tr("%1 results").arg(results_.size()));
  emit searchChanged(currentQuery_);
}

bool AdvancedSearchPanel::matchesFilters(const SearchResult& result) const {
  // If no feature filters are checked, everything passes
  if (!hasInputsCheck_->isChecked() && !hasOutputsCheck_->isChecked() &&
      !hasServicesCheck_->isChecked() && !hasActionsCheck_->isChecked()) {
    return true;
  }

  // Check each enabled filter
  if (hasInputsCheck_->isChecked() && !result.features.contains("inputs")) {
    return false;
  }
  if (hasOutputsCheck_->isChecked() && !result.features.contains("outputs")) {
    return false;
  }
  if (hasServicesCheck_->isChecked() && !result.features.contains("services")) {
    return false;
  }
  if (hasActionsCheck_->isChecked() && !result.features.contains("actions")) {
    return false;
  }

  return true;
}

void AdvancedSearchPanel::addResultToList(const SearchResult& result) {
  QListWidgetItem* item = new QListWidgetItem(resultsList_);

  QString text = result.name;
  if (!result.category.isEmpty()) {
    text += QString(" [%1]").arg(result.category);
  }
  if (!result.description.isEmpty()) {
    text += "\n" + result.description;
  }

  item->setText(text);
  item->setData(Qt::UserRole, result.name);
  item->setData(Qt::UserRole + 1, result.packageName);
  item->setData(Qt::UserRole + 2, result.category);

  resultsList_->addItem(item);
}

void AdvancedSearchPanel::onResultClicked(QListWidgetItem* item) {
  addButton_->setEnabled(item != nullptr);

  if (item) {
    // Find the result
    QString name = item->data(Qt::UserRole).toString();
    for (const SearchResult& result : results_) {
      if (result.name == name) {
        emit resultSelected(result);
        break;
      }
    }
  }
}

void AdvancedSearchPanel::onResultDoubleClicked(QListWidgetItem* item) {
  if (item) {
    QString name = item->data(Qt::UserRole).toString();
    for (const SearchResult& result : results_) {
      if (result.name == name) {
        emit resultDoubleClicked(result);
        emit addToCanvasRequested(result);
        break;
      }
    }
  }
}

void AdvancedSearchPanel::onAddClicked() {
  QListWidgetItem* item = resultsList_->currentItem();
  if (item) {
    QString name = item->data(Qt::UserRole).toString();
    for (const SearchResult& result : results_) {
      if (result.name == name) {
        emit addToCanvasRequested(result);
        break;
      }
    }
  }
}

}  // namespace ros_weaver

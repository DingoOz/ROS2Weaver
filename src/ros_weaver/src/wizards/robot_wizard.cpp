#include "ros_weaver/wizards/robot_wizard.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QFormLayout>
#include <QFileDialog>
#include <QMessageBox>
#include <QStandardPaths>
#include <QRegularExpression>
#include <QDir>
#include <QFileInfo>
#include <QHeaderView>
#include <QFont>
#include <QAbstractButton>
#include <QTimer>
#include <QFile>
#include <QTextStream>
#include <QSplitter>

namespace ros_weaver {

// =============================================================================
// RobotWizardProgressWidget
// =============================================================================

RobotWizardProgressWidget::RobotWizardProgressWidget(QWidget* parent)
  : QFrame(parent)
  , currentStep_(0)
{
  setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
  setAutoFillBackground(true);

  QPalette pal = palette();
  pal.setColor(QPalette::Window, pal.color(QPalette::Window).darker(105));
  setPalette(pal);

  QVBoxLayout* mainLayout = new QVBoxLayout(this);
  mainLayout->setContentsMargins(12, 8, 12, 8);
  mainLayout->setSpacing(8);

  // Top row: step indicators (circles)
  indicatorContainer_ = new QWidget();
  QHBoxLayout* indicatorLayout = new QHBoxLayout(indicatorContainer_);
  indicatorLayout->setContentsMargins(0, 0, 0, 0);
  indicatorLayout->setSpacing(0);
  mainLayout->addWidget(indicatorContainer_);

  // Progress bar
  progressBar_ = new QProgressBar();
  progressBar_->setTextVisible(false);
  progressBar_->setFixedHeight(6);
  progressBar_->setStyleSheet(
    "QProgressBar { border: none; background-color: #404040; border-radius: 3px; }"
    "QProgressBar::chunk { background-color: #3daee9; border-radius: 3px; }"
  );
  mainLayout->addWidget(progressBar_);

  // Bottom row: step number and title
  QHBoxLayout* labelLayout = new QHBoxLayout();
  labelLayout->setContentsMargins(0, 4, 0, 0);
  labelLayout->setSpacing(8);

  stepLabel_ = new QLabel();
  stepLabel_->setStyleSheet("font-weight: bold; color: #3daee9;");
  labelLayout->addWidget(stepLabel_);

  titleLabel_ = new QLabel();
  titleLabel_->setStyleSheet("color: #888888;");
  labelLayout->addWidget(titleLabel_);

  labelLayout->addStretch();
  mainLayout->addLayout(labelLayout);
}

void RobotWizardProgressWidget::setSteps(const QStringList& stepNames) {
  stepNames_ = stepNames;

  // Clear old indicators
  qDeleteAll(stepIndicators_);
  stepIndicators_.clear();

  QHBoxLayout* layout = qobject_cast<QHBoxLayout*>(indicatorContainer_->layout());
  if (!layout) return;

  // Clear layout
  QLayoutItem* item;
  while ((item = layout->takeAt(0)) != nullptr) {
    delete item;
  }

  // Add stretch at start
  layout->addStretch();

  // Create step indicators - use smaller connectors for 8 steps
  for (int i = 0; i < stepNames_.size(); ++i) {
    if (i > 0) {
      // Add connector line between circles
      QFrame* line = new QFrame();
      line->setFrameShape(QFrame::HLine);
      line->setFixedWidth(24);
      line->setFixedHeight(2);
      line->setStyleSheet("background-color: #404040;");
      layout->addWidget(line);
    }

    QLabel* indicator = new QLabel(QString::number(i + 1));
    indicator->setAlignment(Qt::AlignCenter);
    indicator->setFixedSize(26, 26);
    indicator->setToolTip(stepNames_[i]);
    stepIndicators_.append(indicator);
    layout->addWidget(indicator);
  }

  // Add stretch at end
  layout->addStretch();

  progressBar_->setMaximum(stepNames_.size() > 0 ? stepNames_.size() - 1 : 1);
  updateDisplay();
}

void RobotWizardProgressWidget::setCurrentStep(int step) {
  currentStep_ = step;
  updateDisplay();
}

void RobotWizardProgressWidget::updateDisplay() {
  if (stepNames_.isEmpty()) return;

  // Update progress bar
  progressBar_->setValue(currentStep_);

  // Update step label
  stepLabel_->setText(tr("Step %1 of %2:").arg(currentStep_ + 1).arg(stepNames_.size()));

  // Update title
  if (currentStep_ >= 0 && currentStep_ < stepNames_.size()) {
    titleLabel_->setText(stepNames_[currentStep_]);
  }

  // Update step indicators
  for (int i = 0; i < stepIndicators_.size(); ++i) {
    QLabel* indicator = stepIndicators_[i];
    if (i < currentStep_) {
      // Completed step - green checkmark
      indicator->setText(QString::fromUtf8("\u2713")); // checkmark
      indicator->setStyleSheet(
        "QLabel { background-color: #27ae60; color: white; font-weight: bold; "
        "border-radius: 13px; font-size: 13px; }"
      );
    } else if (i == currentStep_) {
      // Current step - blue highlight
      indicator->setText(QString::number(i + 1));
      indicator->setStyleSheet(
        "QLabel { background-color: #3daee9; color: white; font-weight: bold; "
        "border-radius: 13px; font-size: 11px; }"
      );
    } else {
      // Future step - gray
      indicator->setText(QString::number(i + 1));
      indicator->setStyleSheet(
        "QLabel { background-color: #404040; color: #888888; font-weight: bold; "
        "border-radius: 13px; font-size: 11px; }"
      );
    }
  }
}

// =============================================================================
// RobotWizard
// =============================================================================

RobotWizard::RobotWizard(QWidget* parent)
  : QWizard(parent)
  , progressWidget_(nullptr)
{
  setWindowTitle(tr("Robot Configuration Wizard"));
  setWizardStyle(QWizard::ModernStyle);
  setMinimumSize(900, 700);

  robotTypeSelectionPage_ = new RobotTypeSelectionPage(this);
  robotConfigurationPage_ = new RobotConfigurationPage(this);
  hardwareInterfacesPage_ = new HardwareInterfacesPage(this);
  controllersSelectionPage_ = new ControllersSelectionPage(this);
  sensorsConfigurationPage_ = new SensorsConfigurationPage(this);
  teleopSetupPage_ = new TeleopSetupPage(this);
  visualizationSetupPage_ = new VisualizationSetupPage(this);
  reviewGeneratePage_ = new RobotReviewGeneratePage(this);

  setPage(Page_RobotTypeSelection, robotTypeSelectionPage_);
  setPage(Page_RobotConfiguration, robotConfigurationPage_);
  setPage(Page_HardwareInterfaces, hardwareInterfacesPage_);
  setPage(Page_ControllersSelection, controllersSelectionPage_);
  setPage(Page_SensorsConfiguration, sensorsConfigurationPage_);
  setPage(Page_TeleopSetup, teleopSetupPage_);
  setPage(Page_VisualizationSetup, visualizationSetupPage_);
  setPage(Page_ReviewGenerate, reviewGeneratePage_);

  connect(robotTypeSelectionPage_, &RobotTypeSelectionPage::robotTypeChanged,
          this, &RobotWizard::onRobotTypeChanged);

  setupProgressWidget();
  connect(this, &QWizard::currentIdChanged, this, &RobotWizard::onCurrentPageChanged);
  setOption(QWizard::NoBackButtonOnStartPage, true);
}

RobotWizard::~RobotWizard() = default;

void RobotWizard::setupProgressWidget() {
  progressWidget_ = new RobotWizardProgressWidget(this);

  QStringList stepNames = {
    tr("Robot Type"),
    tr("Configuration"),
    tr("Hardware"),
    tr("Controllers"),
    tr("Sensors"),
    tr("Teleop"),
    tr("Visualization"),
    tr("Generate")
  };
  progressWidget_->setSteps(stepNames);
  progressWidget_->setCurrentStep(0);

  QTimer::singleShot(0, this, [this]() {
    if (QVBoxLayout* vboxLayout = qobject_cast<QVBoxLayout*>(layout())) {
      vboxLayout->insertWidget(0, progressWidget_);
    }
  });
}

void RobotWizard::onCurrentPageChanged(int id) {
  int stepIndex = 0;
  switch (id) {
    case Page_RobotTypeSelection:  stepIndex = 0; break;
    case Page_RobotConfiguration:  stepIndex = 1; break;
    case Page_HardwareInterfaces:  stepIndex = 2; break;
    case Page_ControllersSelection: stepIndex = 3; break;
    case Page_SensorsConfiguration: stepIndex = 4; break;
    case Page_TeleopSetup:         stepIndex = 5; break;
    case Page_VisualizationSetup:  stepIndex = 6; break;
    case Page_ReviewGenerate:      stepIndex = 7; break;
  }
  progressWidget_->setCurrentStep(stepIndex);
}

void RobotWizard::onRobotTypeChanged(RobotType type) {
  config_.robotType = type;
  robotConfigurationPage_->setRobotType(type);
  hardwareInterfacesPage_->setRobotType(type);
  controllersSelectionPage_->setRobotType(type);
  sensorsConfigurationPage_->setRobotType(type);
  teleopSetupPage_->setRobotType(type);
  visualizationSetupPage_->setRobotType(type);
}

void RobotWizard::accept() {
  config_.robotName = robotTypeSelectionPage_->robotName();
  config_.robotDescription = robotTypeSelectionPage_->robotDescription();
  config_.namespace_ = robotTypeSelectionPage_->robotNamespace();
  config_.joints = robotConfigurationPage_->joints();
  config_.wheels = robotConfigurationPage_->wheels();
  config_.baseFrameName = robotConfigurationPage_->baseFrameName();
  config_.robotMass = robotConfigurationPage_->robotMass();
  config_.commandInterface = hardwareInterfacesPage_->commandInterface();
  config_.stateInterface = hardwareInterfacesPage_->stateInterface();
  config_.hardwarePlugin = hardwareInterfacesPage_->hardwarePlugin();
  config_.useSimHardware = hardwareInterfacesPage_->useSimHardware();
  config_.controllers = controllersSelectionPage_->controllers();
  config_.controlFrequency = controllersSelectionPage_->controlFrequency();
  config_.sensors = sensorsConfigurationPage_->sensors();
  config_.teleop = teleopSetupPage_->teleopConfig();
  config_.visualization = visualizationSetupPage_->visualizationConfig();

  QWizard::accept();
}

// =============================================================================
// Step 1: RobotTypeSelectionPage
// =============================================================================

RobotTypeSelectionPage::RobotTypeSelectionPage(QWidget* parent)
  : QWizardPage(parent)
{
  setTitle(tr("Select Robot Type"));
  setSubTitle(tr("Choose a robot type that matches your hardware. Each type comes with "
                 "pre-configured joints, controllers, and recommended sensors."));
  setupUi();
}

void RobotTypeSelectionPage::setupUi() {
  QVBoxLayout* mainLayout = new QVBoxLayout(this);

  QSplitter* splitter = new QSplitter(Qt::Horizontal);

  // Left: Robot type list
  QWidget* leftWidget = new QWidget();
  QVBoxLayout* leftLayout = new QVBoxLayout(leftWidget);
  leftLayout->setContentsMargins(0, 0, 0, 0);

  QLabel* typeLabel = new QLabel(tr("Available Robot Types:"));
  typeLabel->setStyleSheet("font-weight: bold;");
  leftLayout->addWidget(typeLabel);

  robotTypeList_ = new QListWidget();
  robotTypeList_->setMinimumWidth(280);
  robotTypeList_->setIconSize(QSize(48, 48));
  leftLayout->addWidget(robotTypeList_);

  splitter->addWidget(leftWidget);

  // Right: Description and form
  QWidget* rightWidget = new QWidget();
  QVBoxLayout* rightLayout = new QVBoxLayout(rightWidget);
  rightLayout->setContentsMargins(10, 0, 0, 0);

  robotDescriptionView_ = new QTextEdit();
  robotDescriptionView_->setReadOnly(true);
  robotDescriptionView_->setMaximumHeight(200);
  rightLayout->addWidget(robotDescriptionView_);

  // Robot name form
  QGroupBox* nameGroup = new QGroupBox(tr("Robot Identity"));
  QFormLayout* formLayout = new QFormLayout(nameGroup);

  robotNameEdit_ = new QLineEdit();
  robotNameEdit_->setPlaceholderText(tr("my_robot"));
  nameValidationLabel_ = new QLabel();
  nameValidationLabel_->setStyleSheet("color: red; font-size: 11px;");

  QVBoxLayout* nameLayout = new QVBoxLayout();
  nameLayout->setSpacing(2);
  nameLayout->addWidget(robotNameEdit_);
  nameLayout->addWidget(nameValidationLabel_);
  formLayout->addRow(tr("Robot Name*:"), nameLayout);

  namespaceEdit_ = new QLineEdit();
  namespaceEdit_->setPlaceholderText(tr("(optional) e.g., robot1"));
  formLayout->addRow(tr("Namespace:"), namespaceEdit_);

  descriptionEdit_ = new QTextEdit();
  descriptionEdit_->setMaximumHeight(60);
  descriptionEdit_->setPlaceholderText(tr("Brief description of your robot..."));
  formLayout->addRow(tr("Description:"), descriptionEdit_);

  rightLayout->addWidget(nameGroup);
  rightLayout->addStretch();

  splitter->addWidget(rightWidget);
  splitter->setSizes({350, 500});

  mainLayout->addWidget(splitter);

  populateRobotTypes();

  connect(robotTypeList_, &QListWidget::currentRowChanged,
          this, &RobotTypeSelectionPage::onRobotTypeSelected);
  connect(robotNameEdit_, &QLineEdit::textChanged,
          this, &RobotTypeSelectionPage::onRobotNameChanged);

  registerField("robotName*", robotNameEdit_);
}

void RobotTypeSelectionPage::populateRobotTypes() {
  robotTypeList_->clear();

  struct RobotTypeInfo {
    RobotType type;
    QString name;
    QString shortDesc;
  };

  QList<RobotTypeInfo> types = {
    {RobotType::DifferentialDrive, tr("Differential Drive Robot"),
     tr("Two-wheeled mobile base (TurtleBot, DiffBot)")},
    {RobotType::RRBot, tr("RRBot (2-DOF Arm)"),
     tr("Two revolute joint manipulator arm")},
    {RobotType::SixDofArm, tr("6-DOF Industrial Arm"),
     tr("Six-axis industrial robot manipulator")},
    {RobotType::CarLikeAckermann, tr("Car-like (Ackermann)"),
     tr("Four-wheeled vehicle with steering")},
    {RobotType::Tricycle, tr("Tricycle Robot"),
     tr("Three-wheeled vehicle with front steering")},
    {RobotType::Gripper, tr("Parallel Gripper"),
     tr("Two-finger parallel gripper end-effector")},
    {RobotType::CustomRobot, tr("Custom Robot"),
     tr("Define your own robot from scratch")}
  };

  for (const auto& info : types) {
    QListWidgetItem* item = new QListWidgetItem(info.name);
    item->setData(Qt::UserRole, static_cast<int>(info.type));
    item->setToolTip(info.shortDesc);
    robotTypeList_->addItem(item);
  }

  robotTypeList_->setCurrentRow(0);
}

void RobotTypeSelectionPage::updateRobotDescription(RobotType type) {
  QString html;

  switch (type) {
    case RobotType::DifferentialDrive:
      html = tr(
        "<h3>Differential Drive Robot</h3>"
        "<p>A two-wheeled mobile base where each wheel is independently driven. "
        "Direction is controlled by varying the speed of each wheel.</p>"
        "<h4>Typical Examples:</h4>"
        "<ul>"
        "<li>TurtleBot3 Burger/Waffle</li>"
        "<li>ros2_control DiffBot</li>"
        "<li>Create 2 / Roomba</li>"
        "</ul>"
        "<h4>Pre-configured Components:</h4>"
        "<ul>"
        "<li><b>Joints:</b> 2 continuous wheel joints</li>"
        "<li><b>Controller:</b> diff_drive_controller</li>"
        "<li><b>Interfaces:</b> Velocity command, Position/Velocity state</li>"
        "</ul>"
        "<h4>Common Sensors:</h4>"
        "<ul>"
        "<li>2D LiDAR for navigation</li>"
        "<li>IMU for orientation</li>"
        "<li>Wheel encoders (built-in)</li>"
        "</ul>"
      );
      break;

    case RobotType::RRBot:
      html = tr(
        "<h3>RRBot - 2-DOF Revolute Manipulator</h3>"
        "<p>A simple two-joint robot arm with revolute (rotational) joints. "
        "Ideal for learning ros2_control concepts.</p>"
        "<h4>Typical Examples:</h4>"
        "<ul>"
        "<li>ros2_control_demos RRBot</li>"
        "<li>Educational robot arms</li>"
        "<li>Simple pick-and-place setups</li>"
        "</ul>"
        "<h4>Pre-configured Components:</h4>"
        "<ul>"
        "<li><b>Joints:</b> 2 revolute joints (joint1, joint2)</li>"
        "<li><b>Controller:</b> joint_trajectory_controller or forward_command_controller</li>"
        "<li><b>Interfaces:</b> Position command, Position/Velocity state</li>"
        "</ul>"
        "<h4>Joint Limits:</h4>"
        "<ul>"
        "<li>Position: -π to +π radians</li>"
        "<li>Velocity: 1.0 rad/s</li>"
        "<li>Effort: 100 Nm</li>"
        "</ul>"
      );
      break;

    case RobotType::SixDofArm:
      html = tr(
        "<h3>6-DOF Industrial Robot Arm</h3>"
        "<p>A full six-axis manipulator providing complete spatial positioning "
        "and orientation control of the end-effector.</p>"
        "<h4>Typical Examples:</h4>"
        "<ul>"
        "<li>Universal Robots UR5/UR10</li>"
        "<li>KUKA LBR iiwa</li>"
        "<li>Franka Emika Panda</li>"
        "</ul>"
        "<h4>Pre-configured Components:</h4>"
        "<ul>"
        "<li><b>Joints:</b> 6 revolute joints (shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3)</li>"
        "<li><b>Controller:</b> joint_trajectory_controller</li>"
        "<li><b>Interfaces:</b> Position command, Position/Velocity/Effort state</li>"
        "</ul>"
        "<h4>Common Additions:</h4>"
        "<ul>"
        "<li>Gripper end-effector</li>"
        "<li>Force/torque sensor at wrist</li>"
        "<li>Camera for vision-guided manipulation</li>"
        "</ul>"
      );
      break;

    case RobotType::CarLikeAckermann:
      html = tr(
        "<h3>Car-like Robot (Ackermann Steering)</h3>"
        "<p>A four-wheeled vehicle using Ackermann steering geometry, similar to "
        "a standard automobile. Front wheels steer while rear wheels drive.</p>"
        "<h4>Typical Examples:</h4>"
        "<ul>"
        "<li>Autonomous vehicles</li>"
        "<li>Golf carts / Utility vehicles</li>"
        "<li>RC cars with Ackermann steering</li>"
        "</ul>"
        "<h4>Pre-configured Components:</h4>"
        "<ul>"
        "<li><b>Joints:</b> 2 steering joints (front), 2-4 drive wheels</li>"
        "<li><b>Controller:</b> ackermann_steering_controller</li>"
        "<li><b>Interfaces:</b> Velocity/Position command</li>"
        "</ul>"
        "<h4>Key Parameters:</h4>"
        "<ul>"
        "<li>Wheelbase: Distance between front and rear axles</li>"
        "<li>Track width: Distance between left and right wheels</li>"
        "<li>Steering angle limits</li>"
        "</ul>"
      );
      break;

    case RobotType::Tricycle:
      html = tr(
        "<h3>Tricycle Robot</h3>"
        "<p>A three-wheeled vehicle with a single steerable front wheel and "
        "two fixed rear wheels. Simpler than Ackermann but with good maneuverability.</p>"
        "<h4>Typical Examples:</h4>"
        "<ul>"
        "<li>Warehouse robots</li>"
        "<li>Material handling vehicles</li>"
        "<li>Simple AGVs</li>"
        "</ul>"
        "<h4>Pre-configured Components:</h4>"
        "<ul>"
        "<li><b>Joints:</b> 1 steering joint, 1 front drive, 2 rear wheels</li>"
        "<li><b>Controller:</b> tricycle_controller</li>"
        "<li><b>Interfaces:</b> Velocity command for drive, Position for steering</li>"
        "</ul>"
      );
      break;

    case RobotType::Gripper:
      html = tr(
        "<h3>Parallel Gripper</h3>"
        "<p>A two-finger parallel gripper, typically mounted as an end-effector "
        "on a robot arm. Fingers move in parallel to grasp objects.</p>"
        "<h4>Typical Examples:</h4>"
        "<ul>"
        "<li>Robotiq 2F-85/2F-140</li>"
        "<li>OnRobot RG2/RG6</li>"
        "<li>Custom pneumatic grippers</li>"
        "</ul>"
        "<h4>Pre-configured Components:</h4>"
        "<ul>"
        "<li><b>Joints:</b> 1-2 prismatic joints (finger movement)</li>"
        "<li><b>Controller:</b> gripper_action_controller or forward_command_controller</li>"
        "<li><b>Interfaces:</b> Position command (grip width)</li>"
        "</ul>"
        "<h4>Common Features:</h4>"
        "<ul>"
        "<li>Force feedback for grasp detection</li>"
        "<li>Configurable grip force</li>"
        "<li>Open/close position limits</li>"
        "</ul>"
      );
      break;

    case RobotType::CustomRobot:
      html = tr(
        "<h3>Custom Robot</h3>"
        "<p>Start from scratch and define your own robot configuration. "
        "You'll manually specify all joints, interfaces, and controllers.</p>"
        "<h4>Use this when:</h4>"
        "<ul>"
        "<li>Your robot doesn't fit standard categories</li>"
        "<li>You have a unique kinematic structure</li>"
        "<li>You want full control over all parameters</li>"
        "</ul>"
        "<h4>You will configure:</h4>"
        "<ul>"
        "<li><b>Joints:</b> Add revolute, continuous, prismatic, or fixed joints</li>"
        "<li><b>Hardware Interfaces:</b> Select command and state interfaces per joint</li>"
        "<li><b>Controllers:</b> Choose from available ros2_control controllers</li>"
        "<li><b>Sensors:</b> Add any combination of sensors</li>"
        "</ul>"
      );
      break;
  }

  robotDescriptionView_->setHtml(html);
}

void RobotTypeSelectionPage::onRobotTypeSelected(int index) {
  if (index < 0) return;

  QListWidgetItem* item = robotTypeList_->item(index);
  RobotType type = static_cast<RobotType>(item->data(Qt::UserRole).toInt());

  updateRobotDescription(type);
  emit robotTypeChanged(type);
  emit completeChanged();
}

void RobotTypeSelectionPage::onRobotNameChanged(const QString& name) {
  QRegularExpression validName("^[a-z][a-z0-9_]*$");
  if (name.isEmpty()) {
    nameValidationLabel_->setText("");
  } else if (!validName.match(name).hasMatch()) {
    nameValidationLabel_->setText(tr("Must start with lowercase letter, contain only lowercase letters, numbers, underscores"));
  } else {
    nameValidationLabel_->setText("");
  }
  emit completeChanged();
}

void RobotTypeSelectionPage::initializePage() {
  robotNameEdit_->setFocus();
}

bool RobotTypeSelectionPage::validatePage() {
  QRegularExpression validName("^[a-z][a-z0-9_]*$");
  if (!validName.match(robotNameEdit_->text()).hasMatch()) {
    QMessageBox::warning(this, tr("Invalid Robot Name"),
      tr("Robot name must start with a lowercase letter and contain only "
         "lowercase letters, numbers, and underscores."));
    return false;
  }
  return true;
}

bool RobotTypeSelectionPage::isComplete() const {
  return !robotNameEdit_->text().isEmpty() &&
         robotTypeList_->currentRow() >= 0 &&
         getIncompleteReason().isEmpty();
}

QString RobotTypeSelectionPage::getIncompleteReason() const {
  if (robotNameEdit_->text().isEmpty()) {
    return tr("Enter a robot name");
  }
  QRegularExpression validName("^[a-z][a-z0-9_]*$");
  if (!validName.match(robotNameEdit_->text()).hasMatch()) {
    return tr("Invalid robot name format");
  }
  if (robotTypeList_->currentRow() < 0) {
    return tr("Select a robot type");
  }
  return QString();
}

RobotType RobotTypeSelectionPage::selectedRobotType() const {
  if (robotTypeList_->currentRow() < 0) return RobotType::DifferentialDrive;
  return static_cast<RobotType>(robotTypeList_->currentItem()->data(Qt::UserRole).toInt());
}

QString RobotTypeSelectionPage::robotName() const {
  return robotNameEdit_->text();
}

QString RobotTypeSelectionPage::robotDescription() const {
  return descriptionEdit_->toPlainText();
}

QString RobotTypeSelectionPage::robotNamespace() const {
  return namespaceEdit_->text();
}

// =============================================================================
// Step 2: RobotConfigurationPage
// =============================================================================

RobotConfigurationPage::RobotConfigurationPage(QWidget* parent)
  : QWizardPage(parent)
  , currentRobotType_(RobotType::DifferentialDrive)
{
  setTitle(tr("Robot Configuration"));
  setSubTitle(tr("Configure the physical structure of your robot including joints, "
                 "wheels, and mechanical parameters."));
  setupUi();
}

void RobotConfigurationPage::setupUi() {
  QVBoxLayout* mainLayout = new QVBoxLayout(this);

  // General parameters
  QGroupBox* generalGroup = new QGroupBox(tr("General Parameters"));
  QFormLayout* generalForm = new QFormLayout(generalGroup);

  baseFrameEdit_ = new QLineEdit("base_link");
  generalForm->addRow(tr("Base Frame:"), baseFrameEdit_);

  robotMassSpin_ = new QDoubleSpinBox();
  robotMassSpin_->setRange(0.1, 1000.0);
  robotMassSpin_->setValue(10.0);
  robotMassSpin_->setSuffix(" kg");
  generalForm->addRow(tr("Robot Mass:"), robotMassSpin_);

  mainLayout->addWidget(generalGroup);

  // Wheel configuration (for mobile robots)
  wheelGroup_ = new QGroupBox(tr("Wheel Configuration"));
  QFormLayout* wheelForm = new QFormLayout(wheelGroup_);

  wheelRadiusSpin_ = new QDoubleSpinBox();
  wheelRadiusSpin_->setRange(0.01, 1.0);
  wheelRadiusSpin_->setValue(0.05);
  wheelRadiusSpin_->setSuffix(" m");
  wheelRadiusSpin_->setDecimals(3);
  wheelForm->addRow(tr("Wheel Radius:"), wheelRadiusSpin_);

  wheelSeparationSpin_ = new QDoubleSpinBox();
  wheelSeparationSpin_->setRange(0.1, 3.0);
  wheelSeparationSpin_->setValue(0.3);
  wheelSeparationSpin_->setSuffix(" m");
  wheelSeparationSpin_->setDecimals(3);
  wheelForm->addRow(tr("Wheel Separation:"), wheelSeparationSpin_);

  mainLayout->addWidget(wheelGroup_);

  // Joint configuration
  jointGroup_ = new QGroupBox(tr("Joint Configuration"));
  QVBoxLayout* jointLayout = new QVBoxLayout(jointGroup_);

  QHBoxLayout* jointButtonLayout = new QHBoxLayout();
  addJointButton_ = new QPushButton(tr("Add Joint"));
  removeJointButton_ = new QPushButton(tr("Remove Joint"));
  jointButtonLayout->addWidget(addJointButton_);
  jointButtonLayout->addWidget(removeJointButton_);
  jointButtonLayout->addStretch();
  jointLayout->addLayout(jointButtonLayout);

  jointTree_ = new QTreeWidget();
  jointTree_->setHeaderLabels({tr("Joint Name"), tr("Type"), tr("Lower Limit"),
                               tr("Upper Limit"), tr("Vel Limit"), tr("Effort Limit")});
  jointTree_->setColumnWidth(0, 150);
  jointTree_->setColumnWidth(1, 100);
  jointLayout->addWidget(jointTree_);

  mainLayout->addWidget(jointGroup_);

  connect(addJointButton_, &QPushButton::clicked, this, &RobotConfigurationPage::onAddJoint);
  connect(removeJointButton_, &QPushButton::clicked, this, &RobotConfigurationPage::onRemoveJoint);
  connect(wheelRadiusSpin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &RobotConfigurationPage::onWheelParameterChanged);
  connect(wheelSeparationSpin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &RobotConfigurationPage::onWheelParameterChanged);
}

void RobotConfigurationPage::setRobotType(RobotType type) {
  currentRobotType_ = type;
  updateUiForRobotType(type);
}

void RobotConfigurationPage::updateUiForRobotType(RobotType type) {
  bool isMobile = (type == RobotType::DifferentialDrive ||
                   type == RobotType::CarLikeAckermann ||
                   type == RobotType::Tricycle);
  wheelGroup_->setVisible(isMobile);
  jointGroup_->setVisible(true);

  populateDefaultJoints(type);
}

void RobotConfigurationPage::populateDefaultJoints(RobotType type) {
  jointTree_->clear();

  QList<JointConfig> defaults;

  switch (type) {
    case RobotType::DifferentialDrive:
      defaults = {
        {"left_wheel_joint", "continuous", -1e9, 1e9, 10.0, 100.0, true, true, false},
        {"right_wheel_joint", "continuous", -1e9, 1e9, 10.0, 100.0, true, true, false}
      };
      break;

    case RobotType::RRBot:
      defaults = {
        {"joint1", "revolute", -3.14159, 3.14159, 1.0, 100.0, true, true, false},
        {"joint2", "revolute", -3.14159, 3.14159, 1.0, 100.0, true, true, false}
      };
      break;

    case RobotType::SixDofArm:
      defaults = {
        {"shoulder_pan_joint", "revolute", -3.14159, 3.14159, 2.0, 150.0, true, true, true},
        {"shoulder_lift_joint", "revolute", -1.57, 1.57, 2.0, 150.0, true, true, true},
        {"elbow_joint", "revolute", -2.35, 2.35, 2.0, 100.0, true, true, true},
        {"wrist_1_joint", "revolute", -3.14159, 3.14159, 3.0, 50.0, true, true, true},
        {"wrist_2_joint", "revolute", -3.14159, 3.14159, 3.0, 50.0, true, true, true},
        {"wrist_3_joint", "revolute", -3.14159, 3.14159, 3.0, 50.0, true, true, true}
      };
      break;

    case RobotType::CarLikeAckermann:
      defaults = {
        {"front_left_steer_joint", "revolute", -0.7, 0.7, 2.0, 50.0, true, true, false},
        {"front_right_steer_joint", "revolute", -0.7, 0.7, 2.0, 50.0, true, true, false},
        {"front_left_wheel_joint", "continuous", -1e9, 1e9, 10.0, 100.0, true, true, false},
        {"front_right_wheel_joint", "continuous", -1e9, 1e9, 10.0, 100.0, true, true, false},
        {"rear_left_wheel_joint", "continuous", -1e9, 1e9, 10.0, 100.0, true, true, false},
        {"rear_right_wheel_joint", "continuous", -1e9, 1e9, 10.0, 100.0, true, true, false}
      };
      break;

    case RobotType::Tricycle:
      defaults = {
        {"front_steer_joint", "revolute", -0.7, 0.7, 2.0, 50.0, true, true, false},
        {"front_wheel_joint", "continuous", -1e9, 1e9, 10.0, 100.0, true, true, false},
        {"rear_left_wheel_joint", "continuous", -1e9, 1e9, 10.0, 100.0, true, true, false},
        {"rear_right_wheel_joint", "continuous", -1e9, 1e9, 10.0, 100.0, true, true, false}
      };
      break;

    case RobotType::Gripper:
      defaults = {
        {"finger_left_joint", "prismatic", 0.0, 0.04, 0.1, 20.0, true, true, true},
        {"finger_right_joint", "prismatic", 0.0, 0.04, 0.1, 20.0, true, true, true}
      };
      break;

    case RobotType::CustomRobot:
      break;
  }

  for (const auto& joint : defaults) {
    QTreeWidgetItem* item = new QTreeWidgetItem();
    item->setText(0, joint.name);
    item->setText(1, joint.type);
    item->setText(2, QString::number(joint.lowerLimit, 'f', 4));
    item->setText(3, QString::number(joint.upperLimit, 'f', 4));
    item->setText(4, QString::number(joint.velocityLimit, 'f', 2));
    item->setText(5, QString::number(joint.effortLimit, 'f', 1));
    item->setFlags(item->flags() | Qt::ItemIsEditable);
    jointTree_->addTopLevelItem(item);
  }
}

void RobotConfigurationPage::onAddJoint() {
  QTreeWidgetItem* item = new QTreeWidgetItem();
  item->setText(0, "new_joint");
  item->setText(1, "revolute");
  item->setText(2, "-3.14159");
  item->setText(3, "3.14159");
  item->setText(4, "1.0");
  item->setText(5, "100.0");
  item->setFlags(item->flags() | Qt::ItemIsEditable);
  jointTree_->addTopLevelItem(item);
}

void RobotConfigurationPage::onRemoveJoint() {
  QTreeWidgetItem* current = jointTree_->currentItem();
  if (current) {
    delete current;
  }
}

void RobotConfigurationPage::onJointItemChanged(QTreeWidgetItem*, int) {
  // Could add validation here
}

void RobotConfigurationPage::onWheelParameterChanged() {
  // Update preview or validate
}

void RobotConfigurationPage::initializePage() {
  RobotWizard* wiz = qobject_cast<RobotWizard*>(wizard());
  if (wiz) {
    RobotTypeSelectionPage* typePage = qobject_cast<RobotTypeSelectionPage*>(
      wiz->page(RobotWizard::Page_RobotTypeSelection));
    if (typePage) {
      setRobotType(typePage->selectedRobotType());
    }
  }
}

bool RobotConfigurationPage::validatePage() {
  return true;
}

QList<JointConfig> RobotConfigurationPage::joints() const {
  QList<JointConfig> result;
  for (int i = 0; i < jointTree_->topLevelItemCount(); ++i) {
    QTreeWidgetItem* item = jointTree_->topLevelItem(i);
    JointConfig joint;
    joint.name = item->text(0);
    joint.type = item->text(1);
    joint.lowerLimit = item->text(2).toDouble();
    joint.upperLimit = item->text(3).toDouble();
    joint.velocityLimit = item->text(4).toDouble();
    joint.effortLimit = item->text(5).toDouble();
    result.append(joint);
  }
  return result;
}

QList<WheelConfig> RobotConfigurationPage::wheels() const {
  QList<WheelConfig> result;
  if (wheelGroup_->isVisible()) {
    WheelConfig left, right;
    left.name = "left_wheel";
    left.radius = wheelRadiusSpin_->value();
    left.separation = wheelSeparationSpin_->value();
    right.name = "right_wheel";
    right.radius = wheelRadiusSpin_->value();
    right.separation = wheelSeparationSpin_->value();
    result << left << right;
  }
  return result;
}

QString RobotConfigurationPage::baseFrameName() const {
  return baseFrameEdit_->text();
}

double RobotConfigurationPage::robotMass() const {
  return robotMassSpin_->value();
}

void RobotConfigurationPage::populateDefaultWheels(RobotType) {
  // Set defaults based on type
}

// =============================================================================
// Step 3: HardwareInterfacesPage
// =============================================================================

HardwareInterfacesPage::HardwareInterfacesPage(QWidget* parent)
  : QWizardPage(parent)
  , currentRobotType_(RobotType::DifferentialDrive)
{
  setTitle(tr("Hardware Interfaces"));
  setSubTitle(tr("Configure the ros2_control hardware interfaces for commanding and "
                 "reading state from your robot's actuators."));
  setupUi();
}

void HardwareInterfacesPage::setupUi() {
  QVBoxLayout* mainLayout = new QVBoxLayout(this);

  QHBoxLayout* interfaceLayout = new QHBoxLayout();

  // Command interfaces
  commandGroup_ = new QGroupBox(tr("Command Interfaces (Write to Hardware)"));
  QVBoxLayout* cmdLayout = new QVBoxLayout(commandGroup_);

  cmdPositionCheck_ = new QCheckBox(tr("Position - Send target positions"));
  cmdVelocityCheck_ = new QCheckBox(tr("Velocity - Send target velocities"));
  cmdEffortCheck_ = new QCheckBox(tr("Effort - Send force/torque commands"));

  cmdLayout->addWidget(cmdPositionCheck_);
  cmdLayout->addWidget(cmdVelocityCheck_);
  cmdLayout->addWidget(cmdEffortCheck_);
  cmdLayout->addStretch();

  interfaceLayout->addWidget(commandGroup_);

  // State interfaces
  stateGroup_ = new QGroupBox(tr("State Interfaces (Read from Hardware)"));
  QVBoxLayout* stateLayout = new QVBoxLayout(stateGroup_);

  statePositionCheck_ = new QCheckBox(tr("Position - Read joint positions"));
  stateVelocityCheck_ = new QCheckBox(tr("Velocity - Read joint velocities"));
  stateEffortCheck_ = new QCheckBox(tr("Effort - Read force/torque feedback"));

  stateLayout->addWidget(statePositionCheck_);
  stateLayout->addWidget(stateVelocityCheck_);
  stateLayout->addWidget(stateEffortCheck_);
  stateLayout->addStretch();

  interfaceLayout->addWidget(stateGroup_);
  mainLayout->addLayout(interfaceLayout);

  // Hardware plugin
  QGroupBox* pluginGroup = new QGroupBox(tr("Hardware Plugin"));
  QVBoxLayout* pluginLayout = new QVBoxLayout(pluginGroup);

  QFormLayout* pluginForm = new QFormLayout();
  hardwarePluginCombo_ = new QComboBox();
  hardwarePluginCombo_->addItems({
    "mock_components/GenericSystem",
    "gazebo_ros2_control/GazeboSystem",
    "ros2_control_demo_hardware/RRBotSystemPositionOnlyHardware",
    "ros2_control_demo_hardware/DiffBotSystemHardware",
    "Custom..."
  });
  pluginForm->addRow(tr("Hardware Plugin:"), hardwarePluginCombo_);
  pluginLayout->addLayout(pluginForm);

  simHardwareCheck_ = new QCheckBox(tr("Use simulation hardware (mock_components)"));
  simHardwareCheck_->setChecked(true);
  pluginLayout->addWidget(simHardwareCheck_);

  pluginDescriptionText_ = new QTextEdit();
  pluginDescriptionText_->setReadOnly(true);
  pluginDescriptionText_->setMaximumHeight(80);
  pluginLayout->addWidget(pluginDescriptionText_);

  mainLayout->addWidget(pluginGroup);

  // Interface preview
  QGroupBox* previewGroup = new QGroupBox(tr("Interface Preview"));
  QVBoxLayout* previewLayout = new QVBoxLayout(previewGroup);
  interfacePreview_ = new QTreeWidget();
  interfacePreview_->setHeaderLabels({tr("Joint"), tr("Command Interfaces"), tr("State Interfaces")});
  interfacePreview_->setMaximumHeight(150);
  previewLayout->addWidget(interfacePreview_);
  mainLayout->addWidget(previewGroup);

  connect(hardwarePluginCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &HardwareInterfacesPage::onHardwarePluginChanged);

  // Set defaults
  cmdVelocityCheck_->setChecked(true);
  statePositionCheck_->setChecked(true);
  stateVelocityCheck_->setChecked(true);
}

void HardwareInterfacesPage::setRobotType(RobotType type) {
  currentRobotType_ = type;

  // Set appropriate defaults based on robot type
  switch (type) {
    case RobotType::DifferentialDrive:
      cmdPositionCheck_->setChecked(false);
      cmdVelocityCheck_->setChecked(true);
      cmdEffortCheck_->setChecked(false);
      statePositionCheck_->setChecked(true);
      stateVelocityCheck_->setChecked(true);
      break;

    case RobotType::RRBot:
    case RobotType::SixDofArm:
      cmdPositionCheck_->setChecked(true);
      cmdVelocityCheck_->setChecked(false);
      cmdEffortCheck_->setChecked(false);
      statePositionCheck_->setChecked(true);
      stateVelocityCheck_->setChecked(true);
      break;

    case RobotType::Gripper:
      cmdPositionCheck_->setChecked(true);
      cmdVelocityCheck_->setChecked(false);
      cmdEffortCheck_->setChecked(true);
      statePositionCheck_->setChecked(true);
      stateEffortCheck_->setChecked(true);
      break;

    default:
      break;
  }
}

void HardwareInterfacesPage::onHardwarePluginChanged(int index) {
  QString desc;
  switch (index) {
    case 0:
      desc = tr("Generic mock hardware for testing. Simulates perfect hardware response.");
      break;
    case 1:
      desc = tr("Gazebo simulation hardware interface. Connects to Gazebo physics engine.");
      break;
    case 2:
      desc = tr("Demo hardware for RRBot position-only control.");
      break;
    case 3:
      desc = tr("Demo hardware for differential drive robots.");
      break;
    default:
      desc = tr("Custom hardware plugin - you'll need to provide the full plugin name.");
  }
  pluginDescriptionText_->setText(desc);
}

void HardwareInterfacesPage::updateInterfaceDescription() {
  // Update based on selections
}

void HardwareInterfacesPage::initializePage() {
  onHardwarePluginChanged(hardwarePluginCombo_->currentIndex());
}

bool HardwareInterfacesPage::validatePage() {
  if (!cmdPositionCheck_->isChecked() && !cmdVelocityCheck_->isChecked() && !cmdEffortCheck_->isChecked()) {
    QMessageBox::warning(this, tr("No Command Interface"),
      tr("Please select at least one command interface."));
    return false;
  }
  if (!statePositionCheck_->isChecked() && !stateVelocityCheck_->isChecked() && !stateEffortCheck_->isChecked()) {
    QMessageBox::warning(this, tr("No State Interface"),
      tr("Please select at least one state interface."));
    return false;
  }
  return true;
}

ControlInterface HardwareInterfacesPage::commandInterface() const {
  bool pos = cmdPositionCheck_->isChecked();
  bool vel = cmdVelocityCheck_->isChecked();
  bool eff = cmdEffortCheck_->isChecked();

  if (pos && vel && eff) return ControlInterface::All;
  if (pos && vel) return ControlInterface::PositionVelocity;
  if (vel && eff) return ControlInterface::VelocityEffort;
  if (pos) return ControlInterface::Position;
  if (vel) return ControlInterface::Velocity;
  return ControlInterface::Effort;
}

ControlInterface HardwareInterfacesPage::stateInterface() const {
  bool pos = statePositionCheck_->isChecked();
  bool vel = stateVelocityCheck_->isChecked();
  bool eff = stateEffortCheck_->isChecked();

  if (pos && vel && eff) return ControlInterface::All;
  if (pos && vel) return ControlInterface::PositionVelocity;
  if (vel && eff) return ControlInterface::VelocityEffort;
  if (pos) return ControlInterface::Position;
  if (vel) return ControlInterface::Velocity;
  return ControlInterface::Effort;
}

QString HardwareInterfacesPage::hardwarePlugin() const {
  return hardwarePluginCombo_->currentText();
}

bool HardwareInterfacesPage::useSimHardware() const {
  return simHardwareCheck_->isChecked();
}

// =============================================================================
// Step 4: ControllersSelectionPage
// =============================================================================

ControllersSelectionPage::ControllersSelectionPage(QWidget* parent)
  : QWizardPage(parent)
  , currentRobotType_(RobotType::DifferentialDrive)
{
  setTitle(tr("Controller Selection"));
  setSubTitle(tr("Select and configure ros2_control controllers for your robot."));
  setupUi();
}

void ControllersSelectionPage::setupUi() {
  QVBoxLayout* mainLayout = new QVBoxLayout(this);

  QSplitter* splitter = new QSplitter(Qt::Horizontal);

  // Left: Controller list
  QWidget* leftWidget = new QWidget();
  QVBoxLayout* leftLayout = new QVBoxLayout(leftWidget);
  leftLayout->setContentsMargins(0, 0, 0, 0);

  QLabel* listLabel = new QLabel(tr("Available Controllers:"));
  listLabel->setStyleSheet("font-weight: bold;");
  leftLayout->addWidget(listLabel);

  controllerTree_ = new QTreeWidget();
  controllerTree_->setHeaderLabels({tr("Controller"), tr("Type")});
  controllerTree_->setColumnWidth(0, 200);
  leftLayout->addWidget(controllerTree_);

  QFormLayout* freqForm = new QFormLayout();
  controlFrequencySpin_ = new QDoubleSpinBox();
  controlFrequencySpin_->setRange(10, 1000);
  controlFrequencySpin_->setValue(100);
  controlFrequencySpin_->setSuffix(" Hz");
  freqForm->addRow(tr("Control Frequency:"), controlFrequencySpin_);
  leftLayout->addLayout(freqForm);

  splitter->addWidget(leftWidget);

  // Right: Description
  QWidget* rightWidget = new QWidget();
  QVBoxLayout* rightLayout = new QVBoxLayout(rightWidget);
  rightLayout->setContentsMargins(10, 0, 0, 0);

  QLabel* descLabel = new QLabel(tr("Controller Description:"));
  descLabel->setStyleSheet("font-weight: bold;");
  rightLayout->addWidget(descLabel);

  controllerDescriptionText_ = new QTextEdit();
  controllerDescriptionText_->setReadOnly(true);
  rightLayout->addWidget(controllerDescriptionText_);

  splitter->addWidget(rightWidget);
  splitter->setSizes({400, 400});

  mainLayout->addWidget(splitter);

  // Initialize controller descriptions
  controllerDescriptions_ = {
    {"diff_drive_controller", tr(
      "<h4>Differential Drive Controller</h4>"
      "<p>Controls a differential drive mobile base. Accepts Twist messages on cmd_vel "
      "and publishes odometry.</p>"
      "<b>Features:</b><ul>"
      "<li>Velocity smoothing and limits</li>"
      "<li>Odometry calculation and publishing</li>"
      "<li>TF broadcasting (odom -> base_link)</li>"
      "</ul>")},
    {"joint_state_broadcaster", tr(
      "<h4>Joint State Broadcaster</h4>"
      "<p>Publishes joint states (position, velocity, effort) from all joints to "
      "/joint_states topic.</p>"
      "<b>Required for:</b><ul>"
      "<li>Robot state publisher</li>"
      "<li>TF tree generation</li>"
      "<li>Visualization in RViz</li>"
      "</ul>")},
    {"joint_trajectory_controller", tr(
      "<h4>Joint Trajectory Controller</h4>"
      "<p>Executes joint-space trajectories. Commonly used with MoveIt for motion planning.</p>"
      "<b>Features:</b><ul>"
      "<li>Trajectory interpolation</li>"
      "<li>Velocity/acceleration limits</li>"
      "<li>Action server interface</li>"
      "</ul>")},
    {"forward_command_controller", tr(
      "<h4>Forward Command Controller</h4>"
      "<p>Simple controller that forwards commands directly to hardware interfaces.</p>"
      "<b>Use cases:</b><ul>"
      "<li>Direct position/velocity control</li>"
      "<li>Testing hardware interfaces</li>"
      "<li>Simple setpoint control</li>"
      "</ul>")},
    {"ackermann_steering_controller", tr(
      "<h4>Ackermann Steering Controller</h4>"
      "<p>Controls a car-like robot with Ackermann steering geometry.</p>"
      "<b>Features:</b><ul>"
      "<li>Proper Ackermann angle calculation</li>"
      "<li>Odometry for car-like robots</li>"
      "<li>Accepts Twist or AckermannDrive</li>"
      "</ul>")},
    {"tricycle_controller", tr(
      "<h4>Tricycle Controller</h4>"
      "<p>Controls a tricycle robot with front steering wheel.</p>"
      "<b>Features:</b><ul>"
      "<li>Single front wheel steering</li>"
      "<li>Odometry calculation</li>"
      "<li>Velocity and steering limits</li>"
      "</ul>")},
    {"gripper_action_controller", tr(
      "<h4>Gripper Action Controller</h4>"
      "<p>Controls a gripper using GripperCommand action interface.</p>"
      "<b>Features:</b><ul>"
      "<li>Position-based grip control</li>"
      "<li>Force limiting (if effort interface)</li>"
      "<li>Stall detection</li>"
      "</ul>")}
  };

  connect(controllerTree_, &QTreeWidget::itemChanged,
          this, &ControllersSelectionPage::onControllerSelectionChanged);
  connect(controllerTree_, &QTreeWidget::currentItemChanged,
          this, &ControllersSelectionPage::onControllerSelected);
}

void ControllersSelectionPage::setRobotType(RobotType type) {
  currentRobotType_ = type;
  populateControllersForType(type);
}

void ControllersSelectionPage::populateControllersForType(RobotType type) {
  controllerTree_->clear();

  struct ControllerInfo {
    QString name;
    QString type;
    bool defaultEnabled;
  };

  QList<ControllerInfo> controllers;

  // Joint state broadcaster is always recommended
  controllers.append({"joint_state_broadcaster", "joint_state_broadcaster/JointStateBroadcaster", true});

  switch (type) {
    case RobotType::DifferentialDrive:
      controllers.append({"diff_drive_controller", "diff_drive_controller/DiffDriveController", true});
      break;

    case RobotType::RRBot:
      controllers.append({"forward_position_controller", "forward_command_controller/ForwardCommandController", true});
      controllers.append({"joint_trajectory_controller", "joint_trajectory_controller/JointTrajectoryController", false});
      break;

    case RobotType::SixDofArm:
      controllers.append({"joint_trajectory_controller", "joint_trajectory_controller/JointTrajectoryController", true});
      controllers.append({"forward_position_controller", "forward_command_controller/ForwardCommandController", false});
      break;

    case RobotType::CarLikeAckermann:
      controllers.append({"ackermann_steering_controller", "ackermann_steering_controller/AckermannSteeringController", true});
      break;

    case RobotType::Tricycle:
      controllers.append({"tricycle_controller", "tricycle_controller/TricycleController", true});
      break;

    case RobotType::Gripper:
      controllers.append({"gripper_action_controller", "position_controllers/GripperActionController", true});
      break;

    case RobotType::CustomRobot:
      controllers.append({"forward_position_controller", "forward_command_controller/ForwardCommandController", false});
      controllers.append({"joint_trajectory_controller", "joint_trajectory_controller/JointTrajectoryController", false});
      break;
  }

  for (const auto& ctrl : controllers) {
    QTreeWidgetItem* item = new QTreeWidgetItem();
    item->setText(0, ctrl.name);
    item->setText(1, ctrl.type);
    item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
    item->setCheckState(0, ctrl.defaultEnabled ? Qt::Checked : Qt::Unchecked);
    controllerTree_->addTopLevelItem(item);
  }

  if (controllerTree_->topLevelItemCount() > 0) {
    controllerTree_->setCurrentItem(controllerTree_->topLevelItem(0));
  }
}

void ControllersSelectionPage::onControllerSelectionChanged(QTreeWidgetItem*, int) {
  emit completeChanged();
}

void ControllersSelectionPage::onControllerSelected(QTreeWidgetItem* current, QTreeWidgetItem*) {
  if (!current) return;

  QString name = current->text(0);
  QString key = name;
  if (key.contains("_controller") && !controllerDescriptions_.contains(key)) {
    // Try base name
    key = key.split("_").first() + "_controller";
  }

  if (controllerDescriptions_.contains(key)) {
    controllerDescriptionText_->setHtml(controllerDescriptions_[key]);
  } else {
    controllerDescriptionText_->setHtml(tr("<p>No description available for this controller.</p>"));
  }
}

void ControllersSelectionPage::updateControllerDetails(const QString&) {
  // Could show parameters
}

void ControllersSelectionPage::initializePage() {
  RobotWizard* wiz = qobject_cast<RobotWizard*>(wizard());
  if (wiz) {
    RobotTypeSelectionPage* typePage = qobject_cast<RobotTypeSelectionPage*>(
      wiz->page(RobotWizard::Page_RobotTypeSelection));
    if (typePage) {
      populateControllersForType(typePage->selectedRobotType());
    }
  }
}

bool ControllersSelectionPage::validatePage() {
  return isComplete();
}

bool ControllersSelectionPage::isComplete() const {
  for (int i = 0; i < controllerTree_->topLevelItemCount(); ++i) {
    if (controllerTree_->topLevelItem(i)->checkState(0) == Qt::Checked) {
      return true;
    }
  }
  return false;
}

QString ControllersSelectionPage::getIncompleteReason() const {
  if (!isComplete()) {
    return tr("Select at least one controller");
  }
  return QString();
}

QList<ControllerConfig> ControllersSelectionPage::controllers() const {
  QList<ControllerConfig> result;
  for (int i = 0; i < controllerTree_->topLevelItemCount(); ++i) {
    QTreeWidgetItem* item = controllerTree_->topLevelItem(i);
    if (item->checkState(0) == Qt::Checked) {
      ControllerConfig ctrl;
      ctrl.name = item->text(0);
      ctrl.pluginType = item->text(1);
      ctrl.enabled = true;
      result.append(ctrl);
    }
  }
  return result;
}

double ControllersSelectionPage::controlFrequency() const {
  return controlFrequencySpin_->value();
}

// =============================================================================
// Step 5: SensorsConfigurationPage
// =============================================================================

SensorsConfigurationPage::SensorsConfigurationPage(QWidget* parent)
  : QWizardPage(parent)
  , currentRobotType_(RobotType::DifferentialDrive)
{
  setTitle(tr("Sensors Configuration"));
  setSubTitle(tr("Add and configure sensors for your robot. Common sensors include "
                 "cameras, LiDAR, and IMU."));
  setupUi();
}

void SensorsConfigurationPage::setupUi() {
  QHBoxLayout* mainLayout = new QHBoxLayout(this);

  // Left: Sensor list
  QWidget* leftWidget = new QWidget();
  QVBoxLayout* leftLayout = new QVBoxLayout(leftWidget);
  leftLayout->setContentsMargins(0, 0, 0, 0);

  QLabel* listLabel = new QLabel(tr("Configured Sensors:"));
  listLabel->setStyleSheet("font-weight: bold;");
  leftLayout->addWidget(listLabel);

  sensorList_ = new QListWidget();
  leftLayout->addWidget(sensorList_);

  QHBoxLayout* buttonLayout = new QHBoxLayout();
  addSensorButton_ = new QPushButton(tr("Add Sensor"));
  removeSensorButton_ = new QPushButton(tr("Remove"));
  buttonLayout->addWidget(addSensorButton_);
  buttonLayout->addWidget(removeSensorButton_);
  leftLayout->addLayout(buttonLayout);

  mainLayout->addWidget(leftWidget);

  // Right: Sensor config
  QWidget* rightWidget = new QWidget();
  QVBoxLayout* rightLayout = new QVBoxLayout(rightWidget);
  rightLayout->setContentsMargins(10, 0, 0, 0);

  QGroupBox* configGroup = new QGroupBox(tr("Sensor Configuration"));
  QFormLayout* configForm = new QFormLayout(configGroup);

  sensorTypeCombo_ = new QComboBox();
  sensorTypeCombo_->addItems({"2D LiDAR", "3D LiDAR", "Camera", "Depth Camera", "IMU", "GPS"});
  configForm->addRow(tr("Sensor Type:"), sensorTypeCombo_);

  sensorNameEdit_ = new QLineEdit();
  sensorNameEdit_->setPlaceholderText("laser_scanner");
  configForm->addRow(tr("Sensor Name:"), sensorNameEdit_);

  sensorFrameEdit_ = new QLineEdit();
  sensorFrameEdit_->setPlaceholderText("laser_frame");
  configForm->addRow(tr("Frame Name:"), sensorFrameEdit_);

  sensorTopicEdit_ = new QLineEdit();
  sensorTopicEdit_->setPlaceholderText("/scan");
  configForm->addRow(tr("Topic Name:"), sensorTopicEdit_);

  sensorRateSpin_ = new QDoubleSpinBox();
  sensorRateSpin_->setRange(1, 100);
  sensorRateSpin_->setValue(10);
  sensorRateSpin_->setSuffix(" Hz");
  configForm->addRow(tr("Update Rate:"), sensorRateSpin_);

  rightLayout->addWidget(configGroup);

  // LiDAR specific config
  lidarConfigWidget_ = new QGroupBox(tr("LiDAR Parameters"));
  QFormLayout* lidarForm = new QFormLayout(lidarConfigWidget_);

  lidarRaysSpin_ = new QSpinBox();
  lidarRaysSpin_->setRange(1, 2048);
  lidarRaysSpin_->setValue(360);
  lidarForm->addRow(tr("Number of Rays:"), lidarRaysSpin_);

  lidarMinRangeSpin_ = new QDoubleSpinBox();
  lidarMinRangeSpin_->setRange(0.01, 10);
  lidarMinRangeSpin_->setValue(0.1);
  lidarMinRangeSpin_->setSuffix(" m");
  lidarForm->addRow(tr("Min Range:"), lidarMinRangeSpin_);

  lidarMaxRangeSpin_ = new QDoubleSpinBox();
  lidarMaxRangeSpin_->setRange(1, 100);
  lidarMaxRangeSpin_->setValue(10);
  lidarMaxRangeSpin_->setSuffix(" m");
  lidarForm->addRow(tr("Max Range:"), lidarMaxRangeSpin_);

  rightLayout->addWidget(lidarConfigWidget_);

  // Camera specific config
  cameraConfigWidget_ = new QGroupBox(tr("Camera Parameters"));
  QFormLayout* cameraForm = new QFormLayout(cameraConfigWidget_);

  cameraWidthSpin_ = new QSpinBox();
  cameraWidthSpin_->setRange(160, 4096);
  cameraWidthSpin_->setValue(640);
  cameraForm->addRow(tr("Image Width:"), cameraWidthSpin_);

  cameraHeightSpin_ = new QSpinBox();
  cameraHeightSpin_->setRange(120, 2160);
  cameraHeightSpin_->setValue(480);
  cameraForm->addRow(tr("Image Height:"), cameraHeightSpin_);

  cameraFovSpin_ = new QDoubleSpinBox();
  cameraFovSpin_->setRange(10, 180);
  cameraFovSpin_->setValue(60);
  cameraFovSpin_->setSuffix(" deg");
  cameraForm->addRow(tr("Field of View:"), cameraFovSpin_);

  rightLayout->addWidget(cameraConfigWidget_);

  // IMU config (simple)
  imuConfigWidget_ = new QGroupBox(tr("IMU Parameters"));
  QFormLayout* imuForm = new QFormLayout(imuConfigWidget_);
  QLabel* imuNote = new QLabel(tr("IMU provides orientation, angular velocity, and linear acceleration."));
  imuNote->setWordWrap(true);
  imuForm->addRow(imuNote);
  rightLayout->addWidget(imuConfigWidget_);

  rightLayout->addStretch();
  mainLayout->addWidget(rightWidget);

  connect(addSensorButton_, &QPushButton::clicked, this, &SensorsConfigurationPage::onAddSensor);
  connect(removeSensorButton_, &QPushButton::clicked, this, &SensorsConfigurationPage::onRemoveSensor);
  connect(sensorList_, &QListWidget::currentRowChanged, this, &SensorsConfigurationPage::onSensorSelectionChanged);
  connect(sensorTypeCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &SensorsConfigurationPage::onSensorTypeChanged);

  // Initial visibility
  onSensorTypeChanged(0);
}

void SensorsConfigurationPage::setRobotType(RobotType type) {
  currentRobotType_ = type;
  populateDefaultSensors(type);
}

void SensorsConfigurationPage::populateDefaultSensors(RobotType type) {
  sensorList_->clear();

  switch (type) {
    case RobotType::DifferentialDrive:
      sensorList_->addItem("2D LiDAR - laser_scanner");
      sensorList_->addItem("IMU - imu_sensor");
      break;

    case RobotType::CarLikeAckermann:
    case RobotType::Tricycle:
      sensorList_->addItem("2D LiDAR - laser_scanner");
      sensorList_->addItem("Camera - front_camera");
      sensorList_->addItem("IMU - imu_sensor");
      break;

    case RobotType::SixDofArm:
      sensorList_->addItem("Depth Camera - wrist_camera");
      break;

    default:
      break;
  }
}

void SensorsConfigurationPage::onAddSensor() {
  QString type = sensorTypeCombo_->currentText();
  QString name = sensorNameEdit_->text().isEmpty() ? "new_sensor" : sensorNameEdit_->text();
  sensorList_->addItem(type + " - " + name);
}

void SensorsConfigurationPage::onRemoveSensor() {
  QListWidgetItem* current = sensorList_->currentItem();
  if (current) {
    delete current;
  }
}

void SensorsConfigurationPage::onSensorSelectionChanged() {
  // Could load selected sensor's config
}

void SensorsConfigurationPage::onSensorTypeChanged(int /*index*/) {
  QString type = sensorTypeCombo_->currentText();

  lidarConfigWidget_->setVisible(type.contains("LiDAR"));
  cameraConfigWidget_->setVisible(type.contains("Camera"));
  imuConfigWidget_->setVisible(type == "IMU");

  // Update default topic names
  if (type.contains("LiDAR")) {
    sensorTopicEdit_->setPlaceholderText("/scan");
    sensorFrameEdit_->setPlaceholderText("laser_frame");
  } else if (type.contains("Camera")) {
    sensorTopicEdit_->setPlaceholderText("/camera/image_raw");
    sensorFrameEdit_->setPlaceholderText("camera_link");
  } else if (type == "IMU") {
    sensorTopicEdit_->setPlaceholderText("/imu/data");
    sensorFrameEdit_->setPlaceholderText("imu_link");
  } else if (type == "GPS") {
    sensorTopicEdit_->setPlaceholderText("/gps/fix");
    sensorFrameEdit_->setPlaceholderText("gps_link");
  }
}

void SensorsConfigurationPage::updateSensorConfigPanel(const QString&) {
  // Update panel based on type
}

void SensorsConfigurationPage::initializePage() {
  RobotWizard* wiz = qobject_cast<RobotWizard*>(wizard());
  if (wiz) {
    RobotTypeSelectionPage* typePage = qobject_cast<RobotTypeSelectionPage*>(
      wiz->page(RobotWizard::Page_RobotTypeSelection));
    if (typePage) {
      populateDefaultSensors(typePage->selectedRobotType());
    }
  }
}

bool SensorsConfigurationPage::validatePage() {
  return true;  // Sensors are optional
}

QList<SensorConfig> SensorsConfigurationPage::sensors() const {
  QList<SensorConfig> result;
  for (int i = 0; i < sensorList_->count(); ++i) {
    QString text = sensorList_->item(i)->text();
    QStringList parts = text.split(" - ");
    SensorConfig sensor;
    sensor.type = parts.value(0);
    sensor.name = parts.value(1);
    result.append(sensor);
  }
  return result;
}

// =============================================================================
// Step 6: TeleopSetupPage
// =============================================================================

TeleopSetupPage::TeleopSetupPage(QWidget* parent)
  : QWizardPage(parent)
  , currentRobotType_(RobotType::DifferentialDrive)
{
  setTitle(tr("Teleoperation Setup"));
  setSubTitle(tr("Configure how you will manually control your robot for testing "
                 "and operation."));
  setupUi();
}

void TeleopSetupPage::setupUi() {
  QVBoxLayout* mainLayout = new QVBoxLayout(this);

  // Teleop methods
  QGroupBox* methodGroup = new QGroupBox(tr("Teleoperation Methods"));
  QVBoxLayout* methodLayout = new QVBoxLayout(methodGroup);

  keyboardTeleopCheck_ = new QCheckBox(tr("Keyboard Teleop (teleop_twist_keyboard)"));
  keyboardTeleopCheck_->setChecked(true);
  methodLayout->addWidget(keyboardTeleopCheck_);

  joystickTeleopCheck_ = new QCheckBox(tr("Joystick/Gamepad (teleop_twist_joy)"));
  methodLayout->addWidget(joystickTeleopCheck_);

  spaceMouseCheck_ = new QCheckBox(tr("SpaceMouse 3D (spacenav_node)"));
  methodLayout->addWidget(spaceMouseCheck_);

  mainLayout->addWidget(methodGroup);

  // Velocity configuration
  velocityGroup_ = new QGroupBox(tr("Velocity Limits"));
  QFormLayout* velForm = new QFormLayout(velocityGroup_);

  linearSpeedSpin_ = new QDoubleSpinBox();
  linearSpeedSpin_->setRange(0.1, 10.0);
  linearSpeedSpin_->setValue(0.5);
  linearSpeedSpin_->setSuffix(" m/s");
  velForm->addRow(tr("Max Linear Speed:"), linearSpeedSpin_);

  angularSpeedSpin_ = new QDoubleSpinBox();
  angularSpeedSpin_->setRange(0.1, 5.0);
  angularSpeedSpin_->setValue(1.0);
  angularSpeedSpin_->setSuffix(" rad/s");
  velForm->addRow(tr("Max Angular Speed:"), angularSpeedSpin_);

  linearAccelSpin_ = new QDoubleSpinBox();
  linearAccelSpin_->setRange(0.1, 10.0);
  linearAccelSpin_->setValue(0.5);
  linearAccelSpin_->setSuffix(" m/s²");
  velForm->addRow(tr("Linear Acceleration:"), linearAccelSpin_);

  angularAccelSpin_ = new QDoubleSpinBox();
  angularAccelSpin_->setRange(0.1, 5.0);
  angularAccelSpin_->setValue(1.0);
  angularAccelSpin_->setSuffix(" rad/s²");
  velForm->addRow(tr("Angular Acceleration:"), angularAccelSpin_);

  mainLayout->addWidget(velocityGroup_);

  // Topic configuration
  QGroupBox* topicGroup = new QGroupBox(tr("Topic Configuration"));
  QFormLayout* topicForm = new QFormLayout(topicGroup);

  cmdVelTopicEdit_ = new QLineEdit("/cmd_vel");
  topicForm->addRow(tr("Command Topic:"), cmdVelTopicEdit_);

  twistTypeCombo_ = new QComboBox();
  twistTypeCombo_->addItems({"geometry_msgs/Twist", "geometry_msgs/TwistStamped"});
  topicForm->addRow(tr("Message Type:"), twistTypeCombo_);

  mainLayout->addWidget(topicGroup);

  // Joystick configuration
  joystickGroup_ = new QGroupBox(tr("Joystick Configuration"));
  QFormLayout* joyForm = new QFormLayout(joystickGroup_);

  joystickTypeCombo_ = new QComboBox();
  joystickTypeCombo_->addItems({"Xbox Controller", "PS4 Controller", "Logitech F710", "Generic"});
  joyForm->addRow(tr("Controller Type:"), joystickTypeCombo_);

  linearAxisSpin_ = new QSpinBox();
  linearAxisSpin_->setRange(0, 7);
  linearAxisSpin_->setValue(1);
  joyForm->addRow(tr("Linear Axis:"), linearAxisSpin_);

  angularAxisSpin_ = new QSpinBox();
  angularAxisSpin_->setRange(0, 7);
  angularAxisSpin_->setValue(0);
  joyForm->addRow(tr("Angular Axis:"), angularAxisSpin_);

  deadmanButtonSpin_ = new QSpinBox();
  deadmanButtonSpin_->setRange(0, 15);
  deadmanButtonSpin_->setValue(4);
  joyForm->addRow(tr("Deadman Button:"), deadmanButtonSpin_);

  mainLayout->addWidget(joystickGroup_);
  joystickGroup_->setVisible(false);

  mainLayout->addStretch();

  connect(joystickTeleopCheck_, &QCheckBox::toggled, this, &TeleopSetupPage::onTeleopMethodChanged);
}

void TeleopSetupPage::setRobotType(RobotType type) {
  currentRobotType_ = type;
  updateTeleopOptions(type);
}

void TeleopSetupPage::updateTeleopOptions(RobotType type) {
  bool isMobile = (type == RobotType::DifferentialDrive ||
                   type == RobotType::CarLikeAckermann ||
                   type == RobotType::Tricycle);

  velocityGroup_->setVisible(isMobile);
  keyboardTeleopCheck_->setVisible(isMobile);

  if (type == RobotType::SixDofArm || type == RobotType::RRBot) {
    cmdVelTopicEdit_->setText("/joint_commands");
  } else {
    cmdVelTopicEdit_->setText("/cmd_vel");
  }
}

void TeleopSetupPage::onTeleopMethodChanged() {
  joystickGroup_->setVisible(joystickTeleopCheck_->isChecked());
}

void TeleopSetupPage::initializePage() {
  RobotWizard* wiz = qobject_cast<RobotWizard*>(wizard());
  if (wiz) {
    RobotTypeSelectionPage* typePage = qobject_cast<RobotTypeSelectionPage*>(
      wiz->page(RobotWizard::Page_RobotTypeSelection));
    if (typePage) {
      updateTeleopOptions(typePage->selectedRobotType());
    }
  }
}

bool TeleopSetupPage::validatePage() {
  return true;
}

TeleopConfig TeleopSetupPage::teleopConfig() const {
  TeleopConfig config;
  config.enableKeyboard = keyboardTeleopCheck_->isChecked();
  config.enableJoystick = joystickTeleopCheck_->isChecked();
  config.enableSpaceMouse = spaceMouseCheck_->isChecked();
  config.cmdVelTopic = cmdVelTopicEdit_->text();
  config.linearSpeed = linearSpeedSpin_->value();
  config.angularSpeed = angularSpeedSpin_->value();
  config.linearAccel = linearAccelSpin_->value();
  config.angularAccel = angularAccelSpin_->value();
  return config;
}

// =============================================================================
// Step 7: VisualizationSetupPage
// =============================================================================

VisualizationSetupPage::VisualizationSetupPage(QWidget* parent)
  : QWizardPage(parent)
  , currentRobotType_(RobotType::DifferentialDrive)
{
  setTitle(tr("Visualization Setup"));
  setSubTitle(tr("Configure RViz2 and Gazebo visualization for your robot."));
  setupUi();
}

void VisualizationSetupPage::setupUi() {
  QVBoxLayout* mainLayout = new QVBoxLayout(this);

  QHBoxLayout* topLayout = new QHBoxLayout();

  // RViz configuration
  rvizGroup_ = new QGroupBox(tr("RViz2 Configuration"));
  QVBoxLayout* rvizLayout = new QVBoxLayout(rvizGroup_);

  generateRvizCheck_ = new QCheckBox(tr("Generate RViz config file"));
  generateRvizCheck_->setChecked(true);
  rvizLayout->addWidget(generateRvizCheck_);

  QGroupBox* displaysGroup = new QGroupBox(tr("Default Displays"));
  QVBoxLayout* displaysLayout = new QVBoxLayout(displaysGroup);

  showRobotModelCheck_ = new QCheckBox(tr("Robot Model (URDF visualization)"));
  showRobotModelCheck_->setChecked(true);
  displaysLayout->addWidget(showRobotModelCheck_);

  showTfCheck_ = new QCheckBox(tr("TF Frames"));
  showTfCheck_->setChecked(true);
  displaysLayout->addWidget(showTfCheck_);

  showLaserScanCheck_ = new QCheckBox(tr("LaserScan (if LiDAR configured)"));
  showLaserScanCheck_->setChecked(true);
  displaysLayout->addWidget(showLaserScanCheck_);

  showCameraCheck_ = new QCheckBox(tr("Camera Image (if camera configured)"));
  displaysLayout->addWidget(showCameraCheck_);

  showPointCloudCheck_ = new QCheckBox(tr("PointCloud2 (if depth camera)"));
  displaysLayout->addWidget(showPointCloudCheck_);

  rvizLayout->addWidget(displaysGroup);

  QFormLayout* rvizForm = new QFormLayout();
  fixedFrameEdit_ = new QLineEdit("base_link");
  rvizForm->addRow(tr("Fixed Frame:"), fixedFrameEdit_);
  rvizLayout->addLayout(rvizForm);

  topLayout->addWidget(rvizGroup_);

  // Gazebo configuration
  gazeboGroup_ = new QGroupBox(tr("Gazebo Configuration"));
  QVBoxLayout* gazeboLayout = new QVBoxLayout(gazeboGroup_);

  generateGazeboCheck_ = new QCheckBox(tr("Generate Gazebo world file"));
  generateGazeboCheck_->setChecked(true);
  gazeboLayout->addWidget(generateGazeboCheck_);

  QFormLayout* gazeboForm = new QFormLayout();
  worldTemplateCombo_ = new QComboBox();
  worldTemplateCombo_->addItems({"Empty World", "Warehouse", "House", "Outdoor", "Custom"});
  gazeboForm->addRow(tr("World Template:"), worldTemplateCombo_);
  gazeboLayout->addLayout(gazeboForm);

  QGroupBox* worldGroup = new QGroupBox(tr("World Elements"));
  QVBoxLayout* worldLayout = new QVBoxLayout(worldGroup);

  addGroundPlaneCheck_ = new QCheckBox(tr("Ground Plane"));
  addGroundPlaneCheck_->setChecked(true);
  worldLayout->addWidget(addGroundPlaneCheck_);

  addSunCheck_ = new QCheckBox(tr("Sun Light"));
  addSunCheck_->setChecked(true);
  worldLayout->addWidget(addSunCheck_);

  addWallsCheck_ = new QCheckBox(tr("Boundary Walls"));
  worldLayout->addWidget(addWallsCheck_);

  gazeboLayout->addWidget(worldGroup);
  topLayout->addWidget(gazeboGroup_);

  mainLayout->addLayout(topLayout);

  // Preview
  QGroupBox* previewGroup = new QGroupBox(tr("Configuration Preview"));
  QVBoxLayout* previewLayout = new QVBoxLayout(previewGroup);
  configPreviewText_ = new QTextEdit();
  configPreviewText_->setReadOnly(true);
  configPreviewText_->setMaximumHeight(120);
  previewLayout->addWidget(configPreviewText_);
  mainLayout->addWidget(previewGroup);

  connect(generateRvizCheck_, &QCheckBox::toggled, this, &VisualizationSetupPage::onRvizOptionChanged);
  connect(generateGazeboCheck_, &QCheckBox::toggled, this, &VisualizationSetupPage::onGazeboOptionChanged);
}

void VisualizationSetupPage::setRobotType(RobotType type) {
  currentRobotType_ = type;

  if (type == RobotType::SixDofArm || type == RobotType::RRBot || type == RobotType::Gripper) {
    fixedFrameEdit_->setText("world");
  } else {
    fixedFrameEdit_->setText("odom");
  }
}

void VisualizationSetupPage::onRvizOptionChanged() {
  bool enabled = generateRvizCheck_->isChecked();
  showRobotModelCheck_->setEnabled(enabled);
  showTfCheck_->setEnabled(enabled);
  showLaserScanCheck_->setEnabled(enabled);
  showCameraCheck_->setEnabled(enabled);
  showPointCloudCheck_->setEnabled(enabled);
  fixedFrameEdit_->setEnabled(enabled);
}

void VisualizationSetupPage::onGazeboOptionChanged() {
  bool enabled = generateGazeboCheck_->isChecked();
  worldTemplateCombo_->setEnabled(enabled);
  addGroundPlaneCheck_->setEnabled(enabled);
  addSunCheck_->setEnabled(enabled);
  addWallsCheck_->setEnabled(enabled);
}

void VisualizationSetupPage::initializePage() {
  RobotWizard* wiz = qobject_cast<RobotWizard*>(wizard());
  if (wiz) {
    RobotTypeSelectionPage* typePage = qobject_cast<RobotTypeSelectionPage*>(
      wiz->page(RobotWizard::Page_RobotTypeSelection));
    if (typePage) {
      setRobotType(typePage->selectedRobotType());
    }
  }
}

bool VisualizationSetupPage::validatePage() {
  return true;
}

VisualizationConfig VisualizationSetupPage::visualizationConfig() const {
  VisualizationConfig config;
  config.generateRvizConfig = generateRvizCheck_->isChecked();
  config.generateGazeboWorld = generateGazeboCheck_->isChecked();
  config.showRobotModel = showRobotModelCheck_->isChecked();
  config.showTF = showTfCheck_->isChecked();
  config.showLaserScan = showLaserScanCheck_->isChecked();
  config.showCamera = showCameraCheck_->isChecked();
  config.showPointCloud = showPointCloudCheck_->isChecked();
  config.defaultFixedFrame = fixedFrameEdit_->text();
  return config;
}

// =============================================================================
// Step 8: RobotReviewGeneratePage
// =============================================================================

RobotReviewGeneratePage::RobotReviewGeneratePage(QWidget* parent)
  : QWizardPage(parent)
  , generationComplete_(false)
  , generationSuccess_(false)
{
  setTitle(tr("Review & Generate"));
  setSubTitle(tr("Review your robot configuration and generate the package files."));
  setCommitPage(true);
  setupUi();
}

void RobotReviewGeneratePage::setupUi() {
  QVBoxLayout* mainLayout = new QVBoxLayout(this);

  // Output path
  QGroupBox* outputGroup = new QGroupBox(tr("Output Location"));
  QVBoxLayout* outputLayout = new QVBoxLayout(outputGroup);

  QHBoxLayout* pathLayout = new QHBoxLayout();
  outputPathEdit_ = new QLineEdit();
  outputPathEdit_->setPlaceholderText(tr("Select output directory..."));
  browseButton_ = new QPushButton(tr("Browse..."));
  pathLayout->addWidget(outputPathEdit_);
  pathLayout->addWidget(browseButton_);
  outputLayout->addLayout(pathLayout);

  QFormLayout* outputForm = new QFormLayout();
  packageNameEdit_ = new QLineEdit();
  packageNameEdit_->setPlaceholderText(tr("my_robot_description"));
  outputForm->addRow(tr("Package Name:"), packageNameEdit_);
  outputLayout->addLayout(outputForm);

  createSubdirCheck_ = new QCheckBox(tr("Create subdirectory with package name"));
  createSubdirCheck_->setChecked(true);
  outputLayout->addWidget(createSubdirCheck_);

  mainLayout->addWidget(outputGroup);

  // Generation options
  QGroupBox* filesGroup = new QGroupBox(tr("Files to Generate"));
  QHBoxLayout* filesLayout = new QHBoxLayout(filesGroup);

  generateUrdfCheck_ = new QCheckBox(tr("URDF/Xacro"));
  generateUrdfCheck_->setChecked(true);
  filesLayout->addWidget(generateUrdfCheck_);

  generateControllerCheck_ = new QCheckBox(tr("Controller Config"));
  generateControllerCheck_->setChecked(true);
  filesLayout->addWidget(generateControllerCheck_);

  generateLaunchCheck_ = new QCheckBox(tr("Launch Files"));
  generateLaunchCheck_->setChecked(true);
  filesLayout->addWidget(generateLaunchCheck_);

  generateRvizCheck_ = new QCheckBox(tr("RViz Config"));
  generateRvizCheck_->setChecked(true);
  filesLayout->addWidget(generateRvizCheck_);

  generateGazeboCheck_ = new QCheckBox(tr("Gazebo World"));
  generateGazeboCheck_->setChecked(true);
  filesLayout->addWidget(generateGazeboCheck_);

  mainLayout->addWidget(filesGroup);

  // Summary
  QLabel* summaryLabel = new QLabel(tr("Configuration Summary:"));
  summaryLabel->setStyleSheet("font-weight: bold;");
  mainLayout->addWidget(summaryLabel);

  summaryText_ = new QTextEdit();
  summaryText_->setReadOnly(true);
  summaryText_->setMaximumHeight(150);
  mainLayout->addWidget(summaryText_);

  // Files preview
  filesPreview_ = new QTreeWidget();
  filesPreview_->setHeaderHidden(true);
  filesPreview_->setMaximumHeight(120);
  mainLayout->addWidget(filesPreview_);

  // Progress
  progressBar_ = new QProgressBar();
  progressBar_->setVisible(false);
  mainLayout->addWidget(progressBar_);

  statusLabel_ = new QLabel();
  mainLayout->addWidget(statusLabel_);

  // Generate button
  QHBoxLayout* buttonLayout = new QHBoxLayout();
  buttonLayout->addStretch();
  generateButton_ = new QPushButton(tr("Generate Robot Package"));
  generateButton_->setMinimumWidth(180);
  buttonLayout->addWidget(generateButton_);
  buttonLayout->addStretch();
  mainLayout->addLayout(buttonLayout);

  connect(browseButton_, &QPushButton::clicked, this, &RobotReviewGeneratePage::onBrowseOutput);
  connect(generateButton_, &QPushButton::clicked, this, &RobotReviewGeneratePage::onGenerate);
}

void RobotReviewGeneratePage::onBrowseOutput() {
  QString dir = QFileDialog::getExistingDirectory(
    this, tr("Select Output Directory"),
    outputPathEdit_->text().isEmpty() ?
      QStandardPaths::writableLocation(QStandardPaths::HomeLocation) :
      outputPathEdit_->text()
  );
  if (!dir.isEmpty()) {
    outputPathEdit_->setText(dir);
  }
}

void RobotReviewGeneratePage::initializePage() {
  generationComplete_ = false;
  generationSuccess_ = false;
  generateButton_->setEnabled(true);
  generateButton_->setText(tr("Generate Robot Package"));
  progressBar_->setVisible(false);
  statusLabel_->clear();

  // Set defaults from wizard
  RobotWizard* wiz = qobject_cast<RobotWizard*>(wizard());
  if (wiz) {
    RobotTypeSelectionPage* typePage = qobject_cast<RobotTypeSelectionPage*>(
      wiz->page(RobotWizard::Page_RobotTypeSelection));
    if (typePage) {
      QString robotName = typePage->robotName();
      packageNameEdit_->setText(robotName + "_description");
    }
  }

  if (outputPathEdit_->text().isEmpty()) {
    outputPathEdit_->setText(QStandardPaths::writableLocation(QStandardPaths::HomeLocation));
  }

  updateSummary();
}

void RobotReviewGeneratePage::updateSummary() {
  RobotWizard* wiz = qobject_cast<RobotWizard*>(wizard());
  if (!wiz) return;

  QString html;
  QTextStream stream(&html);

  RobotTypeSelectionPage* typePage = qobject_cast<RobotTypeSelectionPage*>(
    wiz->page(RobotWizard::Page_RobotTypeSelection));
  RobotConfigurationPage* configPage = qobject_cast<RobotConfigurationPage*>(
    wiz->page(RobotWizard::Page_RobotConfiguration));
  ControllersSelectionPage* ctrlPage = qobject_cast<ControllersSelectionPage*>(
    wiz->page(RobotWizard::Page_ControllersSelection));
  SensorsConfigurationPage* sensorPage = qobject_cast<SensorsConfigurationPage*>(
    wiz->page(RobotWizard::Page_SensorsConfiguration));

  if (typePage) {
    stream << "<b>Robot:</b> " << typePage->robotName() << "<br>";
  }
  if (configPage) {
    stream << "<b>Joints:</b> " << configPage->joints().size() << "<br>";
    stream << "<b>Base Frame:</b> " << configPage->baseFrameName() << "<br>";
  }
  if (ctrlPage) {
    stream << "<b>Controllers:</b> " << ctrlPage->controllers().size() << "<br>";
  }
  if (sensorPage) {
    stream << "<b>Sensors:</b> " << sensorPage->sensors().size() << "<br>";
  }

  summaryText_->setHtml(html);

  // Update file preview
  filesPreview_->clear();
  QString pkgName = packageNameEdit_->text().isEmpty() ? "robot_description" : packageNameEdit_->text();
  QTreeWidgetItem* root = new QTreeWidgetItem(filesPreview_);
  root->setText(0, pkgName + "/");
  root->setExpanded(true);

  if (generateUrdfCheck_->isChecked()) {
    new QTreeWidgetItem(root, QStringList("urdf/" + pkgName + ".urdf.xacro"));
  }
  if (generateControllerCheck_->isChecked()) {
    new QTreeWidgetItem(root, QStringList("config/controllers.yaml"));
  }
  if (generateLaunchCheck_->isChecked()) {
    new QTreeWidgetItem(root, QStringList("launch/display.launch.py"));
    new QTreeWidgetItem(root, QStringList("launch/gazebo.launch.py"));
  }
  if (generateRvizCheck_->isChecked()) {
    new QTreeWidgetItem(root, QStringList("rviz/config.rviz"));
  }
  new QTreeWidgetItem(root, QStringList("package.xml"));
  new QTreeWidgetItem(root, QStringList("CMakeLists.txt"));
}

void RobotReviewGeneratePage::onGenerate() {
  if (outputPathEdit_->text().isEmpty()) {
    QMessageBox::warning(this, tr("No Output Path"), tr("Please select an output directory."));
    return;
  }

  generateButton_->setEnabled(false);
  progressBar_->setVisible(true);
  progressBar_->setValue(0);
  statusLabel_->setText(tr("Generating..."));

  emit generationStarted();

  bool success = performGeneration();
  onGenerationFinished(success);
}

bool RobotReviewGeneratePage::performGeneration() {
  QString outputDir = outputPathEdit_->text();
  QString pkgName = packageNameEdit_->text();

  if (createSubdirCheck_->isChecked()) {
    outputDir += "/" + pkgName;
  }

  QDir dir(outputDir);
  if (!dir.exists() && !dir.mkpath(".")) {
    statusLabel_->setText(tr("Failed to create output directory"));
    return false;
  }

  // Create subdirectories
  dir.mkpath("urdf");
  dir.mkpath("config");
  dir.mkpath("launch");
  dir.mkpath("rviz");
  dir.mkpath("meshes");
  dir.mkpath("worlds");

  progressBar_->setValue(10);

  if (generateUrdfCheck_->isChecked()) {
    if (!generateUrdfFile(outputDir)) return false;
  }
  progressBar_->setValue(30);

  if (generateControllerCheck_->isChecked()) {
    if (!generateControllerConfig(outputDir)) return false;
  }
  progressBar_->setValue(50);

  if (generateLaunchCheck_->isChecked()) {
    if (!generateLaunchFile(outputDir)) return false;
  }
  progressBar_->setValue(70);

  if (!generatePackageFiles(outputDir)) return false;
  progressBar_->setValue(100);

  generatedPath_ = outputDir;
  return true;
}

bool RobotReviewGeneratePage::generateUrdfFile(const QString& outputDir) {
  RobotWizard* wiz = qobject_cast<RobotWizard*>(wizard());
  if (!wiz) return false;

  RobotTypeSelectionPage* typePage = qobject_cast<RobotTypeSelectionPage*>(
    wiz->page(RobotWizard::Page_RobotTypeSelection));
  RobotConfigurationPage* configPage = qobject_cast<RobotConfigurationPage*>(
    wiz->page(RobotWizard::Page_RobotConfiguration));

  QString robotName = typePage ? typePage->robotName() : "robot";
  QString pkgName = packageNameEdit_->text();

  QFile file(outputDir + "/urdf/" + pkgName + ".urdf.xacro");
  if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) return false;

  QTextStream out(&file);
  out << "<?xml version=\"1.0\"?>\n";
  out << "<robot xmlns:xacro=\"http://www.ros.org/wiki/xacro\" name=\"" << robotName << "\">\n\n";
  out << "  <!-- Generated by ROS Weaver Robot Wizard -->\n\n";
  out << "  <!-- Include ros2_control -->\n";
  out << "  <xacro:include filename=\"$(find " << pkgName << ")/urdf/ros2_control.xacro\"/>\n\n";
  out << "  <!-- Base link -->\n";
  out << "  <link name=\"" << (configPage ? configPage->baseFrameName() : "base_link") << "\">\n";
  out << "    <visual>\n";
  out << "      <geometry>\n";
  out << "        <box size=\"0.3 0.2 0.1\"/>\n";
  out << "      </geometry>\n";
  out << "    </visual>\n";
  out << "    <collision>\n";
  out << "      <geometry>\n";
  out << "        <box size=\"0.3 0.2 0.1\"/>\n";
  out << "      </geometry>\n";
  out << "    </collision>\n";
  out << "    <inertial>\n";
  out << "      <mass value=\"" << (configPage ? configPage->robotMass() : 10.0) << "\"/>\n";
  out << "      <inertia ixx=\"0.1\" ixy=\"0\" ixz=\"0\" iyy=\"0.1\" iyz=\"0\" izz=\"0.1\"/>\n";
  out << "    </inertial>\n";
  out << "  </link>\n\n";

  // Add joints
  if (configPage) {
    for (const auto& joint : configPage->joints()) {
      out << "  <joint name=\"" << joint.name << "\" type=\"" << joint.type << "\">\n";
      out << "    <parent link=\"" << configPage->baseFrameName() << "\"/>\n";
      out << "    <child link=\"" << joint.name << "_link\"/>\n";
      out << "    <axis xyz=\"0 0 1\"/>\n";
      if (joint.type != "continuous") {
        out << "    <limit lower=\"" << joint.lowerLimit << "\" upper=\"" << joint.upperLimit
            << "\" velocity=\"" << joint.velocityLimit << "\" effort=\"" << joint.effortLimit << "\"/>\n";
      }
      out << "  </joint>\n\n";

      out << "  <link name=\"" << joint.name << "_link\">\n";
      out << "    <visual><geometry><cylinder radius=\"0.02\" length=\"0.1\"/></geometry></visual>\n";
      out << "    <collision><geometry><cylinder radius=\"0.02\" length=\"0.1\"/></geometry></collision>\n";
      out << "    <inertial><mass value=\"0.5\"/><inertia ixx=\"0.01\" ixy=\"0\" ixz=\"0\" iyy=\"0.01\" iyz=\"0\" izz=\"0.01\"/></inertial>\n";
      out << "  </link>\n\n";
    }
  }

  out << "</robot>\n";
  file.close();

  // Generate ros2_control.xacro
  QFile ctrlFile(outputDir + "/urdf/ros2_control.xacro");
  if (ctrlFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
    QTextStream ctrlOut(&ctrlFile);
    ctrlOut << "<?xml version=\"1.0\"?>\n";
    ctrlOut << "<robot xmlns:xacro=\"http://www.ros.org/wiki/xacro\">\n\n";
    ctrlOut << "  <ros2_control name=\"" << robotName << "_system\" type=\"system\">\n";
    ctrlOut << "    <hardware>\n";
    ctrlOut << "      <plugin>mock_components/GenericSystem</plugin>\n";
    ctrlOut << "    </hardware>\n";

    if (configPage) {
      for (const auto& joint : configPage->joints()) {
        ctrlOut << "    <joint name=\"" << joint.name << "\">\n";
        ctrlOut << "      <command_interface name=\"velocity\"/>\n";
        ctrlOut << "      <state_interface name=\"position\"/>\n";
        ctrlOut << "      <state_interface name=\"velocity\"/>\n";
        ctrlOut << "    </joint>\n";
      }
    }

    ctrlOut << "  </ros2_control>\n";
    ctrlOut << "</robot>\n";
    ctrlFile.close();
  }

  return true;
}

bool RobotReviewGeneratePage::generateControllerConfig(const QString& outputDir) {
  RobotWizard* wiz = qobject_cast<RobotWizard*>(wizard());
  ControllersSelectionPage* ctrlPage = wiz ? qobject_cast<ControllersSelectionPage*>(
    wiz->page(RobotWizard::Page_ControllersSelection)) : nullptr;

  QFile file(outputDir + "/config/controllers.yaml");
  if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) return false;

  QTextStream out(&file);
  out << "# Generated by ROS Weaver Robot Wizard\n\n";
  out << "controller_manager:\n";
  out << "  ros__parameters:\n";
  out << "    update_rate: " << (ctrlPage ? static_cast<int>(ctrlPage->controlFrequency()) : 100) << "\n\n";

  if (ctrlPage) {
    for (const auto& ctrl : ctrlPage->controllers()) {
      out << "    " << ctrl.name << ":\n";
      out << "      type: " << ctrl.pluginType << "\n";
    }
  }

  out << "\n# Controller configurations\n";
  if (ctrlPage) {
    for (const auto& ctrl : ctrlPage->controllers()) {
      out << ctrl.name << ":\n";
      out << "  ros__parameters:\n";
      if (ctrl.name.contains("joint_state")) {
        out << "    joints: []\n";
      } else if (ctrl.name.contains("diff_drive")) {
        out << "    left_wheel_names: [\"left_wheel_joint\"]\n";
        out << "    right_wheel_names: [\"right_wheel_joint\"]\n";
        out << "    wheel_separation: 0.3\n";
        out << "    wheel_radius: 0.05\n";
      }
      out << "\n";
    }
  }

  file.close();
  return true;
}

bool RobotReviewGeneratePage::generateLaunchFile(const QString& outputDir) {
  QString pkgName = packageNameEdit_->text();

  QFile file(outputDir + "/launch/display.launch.py");
  if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) return false;

  QTextStream out(&file);
  out << "# Generated by ROS Weaver Robot Wizard\n\n";
  out << "from launch import LaunchDescription\n";
  out << "from launch_ros.actions import Node\n";
  out << "from launch.substitutions import Command, PathJoinSubstitution\n";
  out << "from launch_ros.substitutions import FindPackageShare\n\n";
  out << "def generate_launch_description():\n";
  out << "    pkg_share = FindPackageShare('" << pkgName << "')\n\n";
  out << "    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', '" << pkgName << ".urdf.xacro'])\n";
  out << "    robot_description = Command(['xacro ', urdf_file])\n\n";
  out << "    return LaunchDescription([\n";
  out << "        Node(\n";
  out << "            package='robot_state_publisher',\n";
  out << "            executable='robot_state_publisher',\n";
  out << "            parameters=[{'robot_description': robot_description}],\n";
  out << "        ),\n";
  out << "        Node(\n";
  out << "            package='rviz2',\n";
  out << "            executable='rviz2',\n";
  out << "            arguments=['-d', PathJoinSubstitution([pkg_share, 'rviz', 'config.rviz'])],\n";
  out << "        ),\n";
  out << "    ])\n";

  file.close();
  return true;
}

bool RobotReviewGeneratePage::generateRvizConfig(const QString&) {
  return true;
}

bool RobotReviewGeneratePage::generateGazeboWorld(const QString&) {
  return true;
}

bool RobotReviewGeneratePage::generatePackageFiles(const QString& outputDir) {
  QString pkgName = packageNameEdit_->text();

  // package.xml
  QFile pkgFile(outputDir + "/package.xml");
  if (pkgFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
    QTextStream out(&pkgFile);
    out << "<?xml version=\"1.0\"?>\n";
    out << "<package format=\"3\">\n";
    out << "  <name>" << pkgName << "</name>\n";
    out << "  <version>0.1.0</version>\n";
    out << "  <description>Robot description generated by ROS Weaver</description>\n";
    out << "  <maintainer email=\"user@example.com\">User</maintainer>\n";
    out << "  <license>Apache-2.0</license>\n\n";
    out << "  <buildtool_depend>ament_cmake</buildtool_depend>\n";
    out << "  <exec_depend>robot_state_publisher</exec_depend>\n";
    out << "  <exec_depend>xacro</exec_depend>\n";
    out << "  <exec_depend>ros2_control</exec_depend>\n";
    out << "  <exec_depend>controller_manager</exec_depend>\n\n";
    out << "  <export>\n";
    out << "    <build_type>ament_cmake</build_type>\n";
    out << "  </export>\n";
    out << "</package>\n";
    pkgFile.close();
  }

  // CMakeLists.txt
  QFile cmakeFile(outputDir + "/CMakeLists.txt");
  if (cmakeFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
    QTextStream out(&cmakeFile);
    out << "cmake_minimum_required(VERSION 3.8)\n";
    out << "project(" << pkgName << ")\n\n";
    out << "find_package(ament_cmake REQUIRED)\n\n";
    out << "install(DIRECTORY urdf config launch rviz meshes worlds\n";
    out << "  DESTINATION share/${PROJECT_NAME})\n\n";
    out << "ament_package()\n";
    cmakeFile.close();
  }

  return true;
}

void RobotReviewGeneratePage::onGenerationProgress(int percent, const QString& message) {
  progressBar_->setValue(percent);
  statusLabel_->setText(message);
}

void RobotReviewGeneratePage::onGenerationFinished(bool success) {
  generationComplete_ = true;
  generationSuccess_ = success;

  progressBar_->setVisible(false);

  if (success) {
    statusLabel_->setText(tr("Robot package generated successfully at: %1").arg(generatedPath_));
    statusLabel_->setStyleSheet("color: green; font-weight: bold;");
    generateButton_->setText(tr("Generated!"));
  } else {
    statusLabel_->setText(tr("Generation failed"));
    statusLabel_->setStyleSheet("color: red;");
    generateButton_->setEnabled(true);
    generateButton_->setText(tr("Retry"));
  }

  emit generationFinished(success);
  emit completeChanged();
}

bool RobotReviewGeneratePage::validatePage() {
  if (!generationComplete_) {
    QMessageBox::information(this, tr("Generate First"),
      tr("Please click 'Generate Robot Package' before finishing."));
    return false;
  }
  return generationSuccess_;
}

bool RobotReviewGeneratePage::isComplete() const {
  return generationComplete_ && generationSuccess_;
}

QString RobotReviewGeneratePage::getIncompleteReason() const {
  if (!generationComplete_) {
    return tr("Click 'Generate Robot Package' to create the files");
  }
  if (!generationSuccess_) {
    return tr("Generation failed - fix errors and retry");
  }
  return QString();
}

}  // namespace ros_weaver

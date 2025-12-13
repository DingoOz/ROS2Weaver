#ifndef ROS_WEAVER_WIZARDS_ROBOT_WIZARD_HPP
#define ROS_WEAVER_WIZARDS_ROBOT_WIZARD_HPP

#include <QWizard>
#include <QWizardPage>
#include <QLineEdit>
#include <QComboBox>
#include <QTextEdit>
#include <QCheckBox>
#include <QListWidget>
#include <QTreeWidget>
#include <QLabel>
#include <QRadioButton>
#include <QButtonGroup>
#include <QGroupBox>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QProgressBar>
#include <QPushButton>
#include <QFrame>
#include <QStackedWidget>
#include <QScrollArea>
#include <QTableWidget>

namespace ros_weaver {

class Project;

// =============================================================================
// Robot Type Definitions
// =============================================================================

enum class RobotType {
  DifferentialDrive,   // Two-wheeled mobile base (like TurtleBot, DiffBot)
  RRBot,               // 2-DOF Revolute-Revolute manipulator
  SixDofArm,           // 6-DOF Industrial robot arm
  CarLikeAckermann,    // Four-wheeled car with Ackermann steering
  Tricycle,            // Three-wheeled vehicle
  Gripper,             // Parallel gripper end-effector
  CustomRobot          // User-defined robot
};

enum class ControlInterface {
  Position,
  Velocity,
  Effort,
  PositionVelocity,
  VelocityEffort,
  All
};

// =============================================================================
// Robot Configuration Structures
// =============================================================================

struct JointConfig {
  QString name;
  QString type;  // "revolute", "continuous", "prismatic", "fixed"
  double lowerLimit = -3.14159;
  double upperLimit = 3.14159;
  double velocityLimit = 1.0;
  double effortLimit = 100.0;
  bool hasPositionInterface = true;
  bool hasVelocityInterface = true;
  bool hasEffortInterface = false;
};

struct WheelConfig {
  QString name;
  double radius = 0.05;      // meters
  double separation = 0.3;   // meters (for differential drive)
  bool hasPositionInterface = true;
  bool hasVelocityInterface = true;
};

struct SensorConfig {
  QString name;
  QString type;  // "camera", "lidar", "imu", "depth_camera", "gps", "contact"
  QString frameName;
  QString topicName;
  double updateRate = 30.0;
  // Camera-specific
  int imageWidth = 640;
  int imageHeight = 480;
  double fov = 1.047;  // radians (~60 degrees)
  // LiDAR-specific
  int numRays = 360;
  double minRange = 0.1;
  double maxRange = 10.0;
  double minAngle = -3.14159;
  double maxAngle = 3.14159;
};

struct ControllerConfig {
  QString name;
  QString type;
  QString pluginType;
  QStringList jointNames;
  QMap<QString, QVariant> parameters;
  bool enabled = true;
};

struct TeleopConfig {
  bool enableKeyboard = true;
  bool enableJoystick = false;
  bool enableSpaceMouse = false;
  QString cmdVelTopic = "/cmd_vel";
  double linearSpeed = 0.5;
  double angularSpeed = 1.0;
  double linearAccel = 0.5;
  double angularAccel = 1.0;
};

struct VisualizationConfig {
  bool generateRvizConfig = true;
  bool generateGazeboWorld = true;
  bool showRobotModel = true;
  bool showTF = true;
  bool showLaserScan = true;
  bool showCamera = false;
  bool showPointCloud = false;
  QString defaultFixedFrame = "base_link";
};

// Main robot configuration structure
struct RobotWizardConfig {
  // Basic info (Step 1)
  RobotType robotType = RobotType::DifferentialDrive;
  QString robotName = "my_robot";
  QString robotDescription;
  QString namespace_;

  // Robot structure (Step 2)
  QList<JointConfig> joints;
  QList<WheelConfig> wheels;
  QString baseFrameName = "base_link";
  double robotMass = 10.0;  // kg

  // Hardware interfaces (Step 3)
  ControlInterface commandInterface = ControlInterface::Velocity;
  ControlInterface stateInterface = ControlInterface::PositionVelocity;
  QString hardwarePlugin = "mock_components/GenericSystem";
  bool useSimHardware = true;

  // Controllers (Step 4)
  QList<ControllerConfig> controllers;
  double controlFrequency = 100.0;  // Hz

  // Sensors (Step 5)
  QList<SensorConfig> sensors;

  // Teleop (Step 6)
  TeleopConfig teleop;

  // Visualization (Step 7)
  VisualizationConfig visualization;

  // Output (Step 8)
  QString outputPath;
  QString packageName;
  bool generateUrdf = true;
  bool generateLaunchFile = true;
  bool generateControllerConfig = true;
};

// =============================================================================
// Forward Declarations
// =============================================================================

class RobotTypeSelectionPage;
class RobotConfigurationPage;
class HardwareInterfacesPage;
class ControllersSelectionPage;
class SensorsConfigurationPage;
class TeleopSetupPage;
class VisualizationSetupPage;
class RobotReviewGeneratePage;

// =============================================================================
// Progress Widget (reused from PackageWizard pattern)
// =============================================================================

class RobotWizardProgressWidget : public QFrame {
  Q_OBJECT

public:
  explicit RobotWizardProgressWidget(QWidget* parent = nullptr);

  void setSteps(const QStringList& stepNames);
  void setCurrentStep(int step);
  int currentStep() const { return currentStep_; }
  int totalSteps() const { return stepNames_.size(); }

private:
  void updateDisplay();

  QStringList stepNames_;
  int currentStep_ = 0;

  QLabel* stepLabel_;
  QLabel* titleLabel_;
  QProgressBar* progressBar_;
  QList<QLabel*> stepIndicators_;
  QWidget* indicatorContainer_;
};

// =============================================================================
// Main Robot Wizard Class
// =============================================================================

class RobotWizard : public QWizard {
  Q_OBJECT

public:
  explicit RobotWizard(QWidget* parent = nullptr);
  ~RobotWizard() override;

  RobotWizardConfig config() const { return config_; }
  QString generatedPackagePath() const { return generatedPackagePath_; }

  enum PageId {
    Page_RobotTypeSelection,
    Page_RobotConfiguration,
    Page_HardwareInterfaces,
    Page_ControllersSelection,
    Page_SensorsConfiguration,
    Page_TeleopSetup,
    Page_VisualizationSetup,
    Page_ReviewGenerate
  };

signals:
  void generationComplete(bool success, const QString& path);

protected:
  void accept() override;

private slots:
  void onCurrentPageChanged(int id);
  void onRobotTypeChanged(RobotType type);

private:
  void setupProgressWidget();
  void applyRobotTypeDefaults(RobotType type);

  RobotWizardConfig config_;
  QString generatedPackagePath_;

  RobotWizardProgressWidget* progressWidget_;
  RobotTypeSelectionPage* robotTypeSelectionPage_;
  RobotConfigurationPage* robotConfigurationPage_;
  HardwareInterfacesPage* hardwareInterfacesPage_;
  ControllersSelectionPage* controllersSelectionPage_;
  SensorsConfigurationPage* sensorsConfigurationPage_;
  TeleopSetupPage* teleopSetupPage_;
  VisualizationSetupPage* visualizationSetupPage_;
  RobotReviewGeneratePage* reviewGeneratePage_;
};

// =============================================================================
// Step 1: Robot Type Selection Page
// =============================================================================

class RobotTypeSelectionPage : public QWizardPage {
  Q_OBJECT

public:
  explicit RobotTypeSelectionPage(QWidget* parent = nullptr);

  void initializePage() override;
  bool validatePage() override;
  bool isComplete() const override;

  RobotType selectedRobotType() const;
  QString robotName() const;
  QString robotDescription() const;
  QString robotNamespace() const;

signals:
  void robotTypeChanged(RobotType type);

private slots:
  void onRobotTypeSelected(int index);
  void onRobotNameChanged(const QString& name);

private:
  void setupUi();
  void populateRobotTypes();
  void updateRobotDescription(RobotType type);
  QString getIncompleteReason() const;

  QListWidget* robotTypeList_;
  QLabel* robotImageLabel_;
  QTextEdit* robotDescriptionView_;
  QLabel* featuresLabel_;

  // Robot info form
  QLineEdit* robotNameEdit_;
  QLineEdit* namespaceEdit_;
  QTextEdit* descriptionEdit_;
  QLabel* nameValidationLabel_;
};

// =============================================================================
// Step 2: Robot Configuration Page
// =============================================================================

class RobotConfigurationPage : public QWizardPage {
  Q_OBJECT

public:
  explicit RobotConfigurationPage(QWidget* parent = nullptr);

  void initializePage() override;
  bool validatePage() override;

  QList<JointConfig> joints() const;
  QList<WheelConfig> wheels() const;
  QString baseFrameName() const;
  double robotMass() const;

public slots:
  void setRobotType(RobotType type);

private slots:
  void onAddJoint();
  void onRemoveJoint();
  void onJointItemChanged(QTreeWidgetItem* item, int column);
  void onWheelParameterChanged();

private:
  void setupUi();
  void populateDefaultJoints(RobotType type);
  void populateDefaultWheels(RobotType type);
  void updateUiForRobotType(RobotType type);

  RobotType currentRobotType_;

  // Joint configuration
  QGroupBox* jointGroup_;
  QTreeWidget* jointTree_;
  QPushButton* addJointButton_;
  QPushButton* removeJointButton_;

  // Wheel configuration (for mobile robots)
  QGroupBox* wheelGroup_;
  QDoubleSpinBox* wheelRadiusSpin_;
  QDoubleSpinBox* wheelSeparationSpin_;
  QDoubleSpinBox* wheelWidthSpin_;
  QSpinBox* wheelCountSpin_;

  // General robot parameters
  QLineEdit* baseFrameEdit_;
  QDoubleSpinBox* robotMassSpin_;

  QStackedWidget* configStack_;
};

// =============================================================================
// Step 3: Hardware Interfaces Page
// =============================================================================

class HardwareInterfacesPage : public QWizardPage {
  Q_OBJECT

public:
  explicit HardwareInterfacesPage(QWidget* parent = nullptr);

  void initializePage() override;
  bool validatePage() override;

  ControlInterface commandInterface() const;
  ControlInterface stateInterface() const;
  QString hardwarePlugin() const;
  bool useSimHardware() const;

public slots:
  void setRobotType(RobotType type);

private slots:
  void onHardwarePluginChanged(int index);

private:
  void setupUi();
  void updateInterfaceDescription();

  RobotType currentRobotType_;

  // Command interfaces
  QGroupBox* commandGroup_;
  QCheckBox* cmdPositionCheck_;
  QCheckBox* cmdVelocityCheck_;
  QCheckBox* cmdEffortCheck_;

  // State interfaces
  QGroupBox* stateGroup_;
  QCheckBox* statePositionCheck_;
  QCheckBox* stateVelocityCheck_;
  QCheckBox* stateEffortCheck_;

  // Hardware plugin selection
  QComboBox* hardwarePluginCombo_;
  QTextEdit* pluginDescriptionText_;
  QCheckBox* simHardwareCheck_;

  // Interface preview
  QTreeWidget* interfacePreview_;
};

// =============================================================================
// Step 4: Controllers Selection Page
// =============================================================================

class ControllersSelectionPage : public QWizardPage {
  Q_OBJECT

public:
  explicit ControllersSelectionPage(QWidget* parent = nullptr);

  void initializePage() override;
  bool validatePage() override;
  bool isComplete() const override;

  QList<ControllerConfig> controllers() const;
  double controlFrequency() const;

public slots:
  void setRobotType(RobotType type);

private slots:
  void onControllerSelectionChanged(QTreeWidgetItem* item, int column);
  void onControllerSelected(QTreeWidgetItem* current, QTreeWidgetItem* previous);

private:
  void setupUi();
  void populateControllersForType(RobotType type);
  void updateControllerDetails(const QString& controllerType);
  QString getIncompleteReason() const;

  RobotType currentRobotType_;

  QTreeWidget* controllerTree_;
  QTextEdit* controllerDescriptionText_;
  QTableWidget* controllerParamsTable_;
  QDoubleSpinBox* controlFrequencySpin_;

  // Controller descriptions
  QMap<QString, QString> controllerDescriptions_;
};

// =============================================================================
// Step 5: Sensors Configuration Page
// =============================================================================

class SensorsConfigurationPage : public QWizardPage {
  Q_OBJECT

public:
  explicit SensorsConfigurationPage(QWidget* parent = nullptr);

  void initializePage() override;
  bool validatePage() override;

  QList<SensorConfig> sensors() const;

public slots:
  void setRobotType(RobotType type);

private slots:
  void onAddSensor();
  void onRemoveSensor();
  void onSensorSelectionChanged();
  void onSensorTypeChanged(int index);

private:
  void setupUi();
  void populateDefaultSensors(RobotType type);
  void updateSensorConfigPanel(const QString& sensorType);

  RobotType currentRobotType_;

  QListWidget* sensorList_;
  QPushButton* addSensorButton_;
  QPushButton* removeSensorButton_;

  // Sensor configuration panel
  QStackedWidget* sensorConfigStack_;
  QComboBox* sensorTypeCombo_;
  QLineEdit* sensorNameEdit_;
  QLineEdit* sensorFrameEdit_;
  QLineEdit* sensorTopicEdit_;
  QDoubleSpinBox* sensorRateSpin_;

  // Camera config
  QWidget* cameraConfigWidget_;
  QSpinBox* cameraWidthSpin_;
  QSpinBox* cameraHeightSpin_;
  QDoubleSpinBox* cameraFovSpin_;

  // LiDAR config
  QWidget* lidarConfigWidget_;
  QSpinBox* lidarRaysSpin_;
  QDoubleSpinBox* lidarMinRangeSpin_;
  QDoubleSpinBox* lidarMaxRangeSpin_;
  QDoubleSpinBox* lidarMinAngleSpin_;
  QDoubleSpinBox* lidarMaxAngleSpin_;

  // IMU config
  QWidget* imuConfigWidget_;
};

// =============================================================================
// Step 6: Teleop Setup Page
// =============================================================================

class TeleopSetupPage : public QWizardPage {
  Q_OBJECT

public:
  explicit TeleopSetupPage(QWidget* parent = nullptr);

  void initializePage() override;
  bool validatePage() override;

  TeleopConfig teleopConfig() const;

public slots:
  void setRobotType(RobotType type);

private slots:
  void onTeleopMethodChanged();

private:
  void setupUi();
  void updateTeleopOptions(RobotType type);

  RobotType currentRobotType_;

  // Teleop methods
  QCheckBox* keyboardTeleopCheck_;
  QCheckBox* joystickTeleopCheck_;
  QCheckBox* spaceMouseCheck_;

  // Velocity limits
  QGroupBox* velocityGroup_;
  QDoubleSpinBox* linearSpeedSpin_;
  QDoubleSpinBox* angularSpeedSpin_;
  QDoubleSpinBox* linearAccelSpin_;
  QDoubleSpinBox* angularAccelSpin_;

  // Topic configuration
  QLineEdit* cmdVelTopicEdit_;
  QComboBox* twistTypeCombo_;

  // Joystick configuration
  QGroupBox* joystickGroup_;
  QComboBox* joystickTypeCombo_;
  QSpinBox* linearAxisSpin_;
  QSpinBox* angularAxisSpin_;
  QSpinBox* deadmanButtonSpin_;
};

// =============================================================================
// Step 7: Visualization Setup Page
// =============================================================================

class VisualizationSetupPage : public QWizardPage {
  Q_OBJECT

public:
  explicit VisualizationSetupPage(QWidget* parent = nullptr);

  void initializePage() override;
  bool validatePage() override;

  VisualizationConfig visualizationConfig() const;

public slots:
  void setRobotType(RobotType type);

private slots:
  void onRvizOptionChanged();
  void onGazeboOptionChanged();

private:
  void setupUi();

  RobotType currentRobotType_;

  // RViz configuration
  QGroupBox* rvizGroup_;
  QCheckBox* generateRvizCheck_;
  QCheckBox* showRobotModelCheck_;
  QCheckBox* showTfCheck_;
  QCheckBox* showLaserScanCheck_;
  QCheckBox* showCameraCheck_;
  QCheckBox* showPointCloudCheck_;
  QLineEdit* fixedFrameEdit_;

  // Gazebo configuration
  QGroupBox* gazeboGroup_;
  QCheckBox* generateGazeboCheck_;
  QComboBox* worldTemplateCombo_;
  QCheckBox* addGroundPlaneCheck_;
  QCheckBox* addSunCheck_;
  QCheckBox* addWallsCheck_;

  // Preview
  QTextEdit* configPreviewText_;
};

// =============================================================================
// Step 8: Review & Generate Page
// =============================================================================

class RobotReviewGeneratePage : public QWizardPage {
  Q_OBJECT

public:
  explicit RobotReviewGeneratePage(QWidget* parent = nullptr);

  void initializePage() override;
  bool validatePage() override;
  bool isComplete() const override;

signals:
  void generationStarted();
  void generationProgress(int percent, const QString& message);
  void generationFinished(bool success);

private slots:
  void onBrowseOutput();
  void onGenerate();
  void onGenerationProgress(int percent, const QString& message);
  void onGenerationFinished(bool success);

private:
  void setupUi();
  void updateSummary();
  bool performGeneration();
  QString getIncompleteReason() const;

  // File generation methods
  bool generateUrdfFile(const QString& outputDir);
  bool generateControllerConfig(const QString& outputDir);
  bool generateLaunchFile(const QString& outputDir);
  bool generateRvizConfig(const QString& outputDir);
  bool generateGazeboWorld(const QString& outputDir);
  bool generatePackageFiles(const QString& outputDir);

  // Output configuration
  QLineEdit* outputPathEdit_;
  QPushButton* browseButton_;
  QLineEdit* packageNameEdit_;
  QCheckBox* createSubdirCheck_;

  // File generation options
  QCheckBox* generateUrdfCheck_;
  QCheckBox* generateLaunchCheck_;
  QCheckBox* generateControllerCheck_;
  QCheckBox* generateRvizCheck_;
  QCheckBox* generateGazeboCheck_;

  // Summary
  QTextEdit* summaryText_;
  QTreeWidget* filesPreview_;

  // Progress
  QProgressBar* progressBar_;
  QPushButton* generateButton_;
  QLabel* statusLabel_;

  bool generationComplete_;
  bool generationSuccess_;
  QString generatedPath_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_WIZARDS_ROBOT_WIZARD_HPP

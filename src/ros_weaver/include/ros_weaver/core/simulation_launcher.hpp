#ifndef ROS_WEAVER_CORE_SIMULATION_LAUNCHER_HPP
#define ROS_WEAVER_CORE_SIMULATION_LAUNCHER_HPP

#include <QObject>
#include <QString>
#include <QProcess>
#include <QMap>
#include <QStringList>
#include <QTimer>
#include <QPointF>

namespace ros_weaver {

// Supported simulation backends
enum class SimulatorType {
  Gazebo,       // Gazebo Classic (gz sim)
  Ignition,     // Ignition Gazebo / Gazebo Sim
  AutoDetect    // Auto-detect available simulator
};

// Simulation launch configuration
struct SimulationConfig {
  SimulatorType simulatorType = SimulatorType::AutoDetect;
  QString worldFile;             // Path to .sdf or .world file
  QString robotModel;            // Path to robot URDF/SDF or model name
  QString robotNamespace;        // ROS namespace for the robot
  QPointF spawnPosition;         // Initial spawn position (x, y)
  double spawnZ = 0.0;           // Initial spawn height
  double spawnYaw = 0.0;         // Initial yaw (radians)
  bool headless = false;         // Run without GUI
  bool paused = true;            // Start paused
  bool useBridgeNode = true;     // Use ros_gz_bridge for topic conversion
  QStringList bridgeTopics;      // Topics to bridge
  QMap<QString, QString> envVars;  // Environment variables

  // Physics settings
  double physicsUpdateRate = 1000.0;  // Hz
  double realTimeFactor = 1.0;        // Target real-time factor

  QString configName;            // Name for this configuration
};

// Simulation status
struct SimulationStatus {
  bool isRunning = false;
  bool isPaused = false;
  double simTime = 0.0;
  double realTimeFactor = 0.0;
  QString lastError;
};

// Manages Gazebo/Ignition simulation launching
class SimulationLauncher : public QObject {
  Q_OBJECT

public:
  static SimulationLauncher& instance();

  // Prevent copying
  SimulationLauncher(const SimulationLauncher&) = delete;
  SimulationLauncher& operator=(const SimulationLauncher&) = delete;

  // Detection
  bool isGazeboAvailable() const;
  bool isIgnitionAvailable() const;
  SimulatorType detectBestSimulator() const;
  QString simulatorVersion(SimulatorType type) const;

  // Launch control
  bool launch(const SimulationConfig& config);
  void stop();
  void pause();
  void resume();
  void reset();

  // Status
  SimulationStatus status() const;
  bool isRunning() const;
  QString currentWorldFile() const;

  // Spawn entities
  bool spawnModel(const QString& name, const QString& sdfPath, const QPointF& pos, double z = 0, double yaw = 0);
  bool deleteModel(const QString& name);
  QStringList spawnedModels() const;

  // World management
  QStringList availableWorlds() const;
  QStringList availableRobotModels() const;

  // Quick launch presets
  bool launchEmptyWorld(SimulatorType type = SimulatorType::AutoDetect);
  bool launchTurtleBot3World(const QString& worldName = "turtlebot3_world");
  bool launchCustomWorld(const QString& worldPath);

  // Configuration templates
  static SimulationConfig createDefaultConfig();
  static SimulationConfig createTurtleBot3Config(const QString& variant = "waffle");
  static SimulationConfig createDiffDriveConfig(const QString& robotName = "diff_drive_robot");

signals:
  void simulationStarted();
  void simulationStopped();
  void simulationPaused();
  void simulationResumed();
  void simulationReset();
  void modelSpawned(const QString& name);
  void modelDeleted(const QString& name);
  void errorOccurred(const QString& error);
  void outputReceived(const QString& output);
  void statusChanged(const SimulationStatus& status);

private slots:
  void onProcessStarted();
  void onProcessFinished(int exitCode, QProcess::ExitStatus status);
  void onProcessError(QProcess::ProcessError error);
  void onReadyReadStdout();
  void onReadyReadStderr();
  void updateStatus();

private:
  SimulationLauncher();
  ~SimulationLauncher() override;

  void detectSimulators();
  QStringList buildLaunchCommand(const SimulationConfig& config) const;
  void setupEnvironment(QProcessEnvironment& env, const SimulationConfig& config) const;
  void startBridgeNode(const SimulationConfig& config);
  void stopBridgeNode();
  QString findWorldFile(const QString& worldName) const;
  QString findRobotModel(const QString& modelName) const;

  // Simulator availability
  bool gazeboAvailable_;
  bool ignitionAvailable_;
  QString gazeboVersion_;
  QString ignitionVersion_;

  // Current simulation
  QProcess* simProcess_;
  QProcess* bridgeProcess_;
  SimulationConfig currentConfig_;
  SimulationStatus currentStatus_;
  QStringList spawnedModels_;

  // Status update timer
  QTimer* statusTimer_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_CORE_SIMULATION_LAUNCHER_HPP

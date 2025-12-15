#ifndef ROS_WEAVER_CORE_SLAM_PIPELINE_MANAGER_HPP
#define ROS_WEAVER_CORE_SLAM_PIPELINE_MANAGER_HPP

#include <QObject>
#include <QString>
#include <QStringList>
#include <QMap>
#include <QVariant>
#include <QProcess>
#include <QTimer>

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <thread>
#include <atomic>
#include <mutex>

namespace ros_weaver {

class BagManager;
class PlaybackController;

/**
 * @brief SLAM configuration parameters
 */
struct SlamConfig {
  QString packageName;     // e.g., "slam_toolbox"
  QString launchFile;      // e.g., "online_async_launch.py"
  QString mode;            // e.g., "mapping", "localization"
  QString mapFrame = "map";
  QString odomFrame = "odom";
  QString baseFrame = "base_footprint";
  QString scanTopic = "/scan";
  QMap<QString, QVariant> parameters;
};

/**
 * @brief SLAM parameter preset
 */
struct SlamPreset {
  QString name;
  QString description;
  QString packageName;
  SlamConfig config;
};

/**
 * @brief SLAM node status
 */
enum class SlamNodeStatus {
  NotRunning,
  Starting,
  Running,
  Error,
  Stopping
};

/**
 * @brief Manages SLAM Toolbox lifecycle and parameter tuning
 *
 * Handles launching, monitoring, and stopping SLAM nodes. Provides
 * dynamic parameter reconfiguration and preset management.
 */
class SlamPipelineManager : public QObject {
  Q_OBJECT

public:
  explicit SlamPipelineManager(QObject* parent = nullptr);
  ~SlamPipelineManager() override;

  // SLAM node lifecycle
  bool launchSlam(const SlamConfig& config);
  void stopSlam();
  bool isRunning() const;
  SlamNodeStatus status() const;

  // Launch specific presets
  bool launchSlamToolboxAsync(const QMap<QString, QVariant>& params = {});
  bool launchSlamToolboxSync(const QMap<QString, QVariant>& params = {});

  // Parameter management
  void setParameter(const QString& name, const QVariant& value);
  QVariant getParameter(const QString& name) const;
  QMap<QString, QVariant> getAllParameters() const;
  QStringList parameterNames() const;

  // Preset management
  void loadPreset(const QString& presetName);
  bool savePreset(const QString& presetName, const QString& description = "");
  void deletePreset(const QString& presetName);
  QStringList availablePresets() const;
  SlamPreset currentPreset() const;
  void setPresetsDirectory(const QString& path);

  // Auto-rerun support
  void enableAutoRerun(bool enable);
  bool isAutoRerunEnabled() const;
  void triggerRerun();

  // Bag/Playback integration
  void setBagManager(BagManager* manager);
  void setPlaybackController(PlaybackController* controller);

  // Current configuration
  SlamConfig currentConfig() const;

signals:
  void slamStarted();
  void slamStopped();
  void slamError(const QString& error);
  void statusChanged(SlamNodeStatus status);
  void parameterChanged(const QString& name, const QVariant& value);
  void parametersUpdated();
  void presetLoaded(const QString& name);
  void presetSaved(const QString& name);
  void outputReceived(const QString& output);
  void rerunTriggered();

private slots:
  void onProcessStarted();
  void onProcessFinished(int exitCode, QProcess::ExitStatus exitStatus);
  void onProcessError(QProcess::ProcessError error);
  void onProcessReadyReadStdout();
  void onProcessReadyReadStderr();
  void onParameterCheckTimer();

private:
  void initializeRosNode();
  void shutdownRosNode();
  void createParameterClient(const QString& nodeName);
  void loadPresetsFromDirectory();
  void savePresetToFile(const SlamPreset& preset);
  bool checkSlamNodeRunning();
  QString buildLaunchCommand(const SlamConfig& config) const;

  // ROS 2 node for parameter client
  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<std::thread> spinThread_;
  std::atomic<bool> spinning_{false};

  // Parameter client for dynamic reconfigure
  std::shared_ptr<rclcpp::AsyncParametersClient> paramClient_;

  // Process management
  QProcess* slamProcess_ = nullptr;
  SlamNodeStatus status_ = SlamNodeStatus::NotRunning;
  QString slamNodeName_;

  // Configuration
  SlamConfig currentConfig_;
  QMap<QString, QVariant> currentParams_;
  bool autoRerunEnabled_ = false;

  // Presets
  QString presetsDirectory_;
  QMap<QString, SlamPreset> presets_;
  QString currentPresetName_;

  // Integration
  BagManager* bagManager_ = nullptr;
  PlaybackController* playbackController_ = nullptr;

  // Timer for parameter checking
  QTimer* paramCheckTimer_ = nullptr;

  mutable std::mutex mutex_;
};

}  // namespace ros_weaver

Q_DECLARE_METATYPE(ros_weaver::SlamConfig)
Q_DECLARE_METATYPE(ros_weaver::SlamPreset)
Q_DECLARE_METATYPE(ros_weaver::SlamNodeStatus)

#endif  // ROS_WEAVER_CORE_SLAM_PIPELINE_MANAGER_HPP

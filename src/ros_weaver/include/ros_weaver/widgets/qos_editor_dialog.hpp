#ifndef ROS_WEAVER_WIDGETS_QOS_EDITOR_DIALOG_HPP
#define ROS_WEAVER_WIDGETS_QOS_EDITOR_DIALOG_HPP

#include <QDialog>
#include <QComboBox>
#include <QSpinBox>
#include <QLabel>
#include <QGroupBox>
#include <QPlainTextEdit>
#include <QTabWidget>

#include "ros_weaver/core/qos_profile.hpp"

class QVBoxLayout;

namespace ros_weaver {

/**
 * @brief Visual editor for QoS profiles
 *
 * Features:
 * - Preset selection (Sensor Data, Services, Parameters, Default)
 * - Custom configuration for all QoS policies
 * - Compatibility checker with visual warnings
 * - Code snippet generation for C++ and Python
 */
class QosEditorDialog : public QDialog {
  Q_OBJECT

public:
  explicit QosEditorDialog(QWidget* parent = nullptr);
  explicit QosEditorDialog(const QosProfile& profile, QWidget* parent = nullptr);
  ~QosEditorDialog() override;

  /**
   * @brief Get the configured QoS profile
   */
  QosProfile profile() const;

  /**
   * @brief Set the QoS profile to edit
   */
  void setProfile(const QosProfile& profile);

  /**
   * @brief Set publisher QoS for compatibility checking
   */
  void setPublisherProfile(const QosProfile& profile);

  /**
   * @brief Set subscriber QoS for compatibility checking
   */
  void setSubscriberProfile(const QosProfile& profile);

  /**
   * @brief Set connection name for display
   */
  void setConnectionName(const QString& name);

public slots:
  void updateFromPreset(int index);
  void onPolicyChanged();
  void updateCompatibilityCheck();
  void updateCodeSnippets();

private:
  void setupUI();
  void setupPresetSection(QVBoxLayout* layout);
  void setupPoliciesSection(QVBoxLayout* layout);
  void setupCompatibilitySection(QVBoxLayout* layout);
  void setupCodeSection(QVBoxLayout* layout);
  void loadProfileToUI(const QosProfile& profile);
  QosProfile createProfileFromUI() const;

  // Current profile
  QosProfile profile_;
  QString connectionName_;

  // For compatibility checking
  QosProfile* publisherProfile_ = nullptr;
  QosProfile* subscriberProfile_ = nullptr;
  bool isPublisherMode_ = false;

  // Preset section
  QComboBox* presetCombo_;
  QLabel* presetDescription_;

  // Policy section
  QComboBox* reliabilityCombo_;
  QComboBox* durabilityCombo_;
  QComboBox* historyCombo_;
  QSpinBox* historyDepthSpin_;
  QComboBox* livelinessCombo_;
  QSpinBox* deadlineSpin_;
  QSpinBox* lifespanSpin_;
  QSpinBox* leaseDurationSpin_;

  // Compatibility section
  QGroupBox* compatibilityGroup_;
  QLabel* compatibilityStatus_;
  QLabel* compatibilityDetails_;

  // Code section
  QTabWidget* codeTabWidget_;
  QPlainTextEdit* cppCodeEdit_;
  QPlainTextEdit* pythonCodeEdit_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_WIDGETS_QOS_EDITOR_DIALOG_HPP

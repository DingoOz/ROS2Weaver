#ifndef ROS_WEAVER_WIDGETS_REMOTE_CONNECTION_DIALOG_HPP
#define ROS_WEAVER_WIDGETS_REMOTE_CONNECTION_DIALOG_HPP

#include <QDialog>
#include <QComboBox>
#include <QLineEdit>
#include <QSpinBox>
#include <QPushButton>
#include <QCheckBox>
#include <QTableWidget>
#include <QLabel>
#include <QGroupBox>

#include "ros_weaver/core/remote_connection_manager.hpp"

namespace ros_weaver {

/**
 * @brief Dialog for configuring and establishing remote robot connections
 *
 * Features:
 * - Profile selection and management
 * - SSH connection configuration
 * - ROS domain ID and distro settings
 * - Environment variable configuration
 * - Connection testing
 */
class RemoteConnectionDialog : public QDialog {
  Q_OBJECT

public:
  explicit RemoteConnectionDialog(RemoteConnectionManager* manager,
                                   QWidget* parent = nullptr);
  ~RemoteConnectionDialog() override = default;

  /**
   * @brief Get the configured profile
   */
  RobotProfile getProfile() const;

signals:
  void connectionRequested(const RobotProfile& profile);

private slots:
  void onProfileSelected(int index);
  void onNewProfileClicked();
  void onDeleteProfileClicked();
  void onSaveProfileClicked();
  void onBrowseKeyClicked();
  void onTestConnectionClicked();
  void onConnectClicked();
  void onTestResult(bool success, const QString& message);
  void onAddEnvVarClicked();
  void onRemoveEnvVarClicked();

private:
  void setupUi();
  void setupConnections();
  void loadProfiles();
  void populateFromProfile(const RobotProfile& profile);
  RobotProfile buildProfileFromUi() const;
  void updateUiState();

  RemoteConnectionManager* manager_;

  // Profile selection
  QComboBox* profileCombo_;
  QPushButton* newProfileButton_;
  QPushButton* deleteProfileButton_;
  QPushButton* saveProfileButton_;

  // Connection details
  QGroupBox* connectionGroup_;
  QLineEdit* profileNameEdit_;
  QLineEdit* hostnameEdit_;
  QSpinBox* portSpinBox_;
  QLineEdit* usernameEdit_;
  QLineEdit* sshKeyEdit_;
  QPushButton* browseKeyButton_;

  // ROS settings
  QGroupBox* rosGroup_;
  QSpinBox* domainIdSpinBox_;
  QComboBox* distroCombo_;
  QCheckBox* useTunnelCheckbox_;

  // Environment variables
  QGroupBox* envGroup_;
  QTableWidget* envTable_;
  QPushButton* addEnvButton_;
  QPushButton* removeEnvButton_;

  // Actions
  QPushButton* testButton_;
  QPushButton* connectButton_;
  QPushButton* cancelButton_;

  // Status
  QLabel* statusLabel_;

  // Profile list
  QList<RobotProfile> profiles_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_WIDGETS_REMOTE_CONNECTION_DIALOG_HPP

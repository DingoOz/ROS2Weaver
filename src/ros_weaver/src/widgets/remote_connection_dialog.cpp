#include "ros_weaver/widgets/remote_connection_dialog.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QHeaderView>
#include <QFileDialog>
#include <QMessageBox>
#include <QDir>

namespace ros_weaver {

RemoteConnectionDialog::RemoteConnectionDialog(RemoteConnectionManager* manager,
                                                 QWidget* parent)
  : QDialog(parent)
  , manager_(manager)
  , profileCombo_(nullptr)
  , newProfileButton_(nullptr)
  , deleteProfileButton_(nullptr)
  , saveProfileButton_(nullptr)
  , connectionGroup_(nullptr)
  , profileNameEdit_(nullptr)
  , hostnameEdit_(nullptr)
  , portSpinBox_(nullptr)
  , usernameEdit_(nullptr)
  , sshKeyEdit_(nullptr)
  , browseKeyButton_(nullptr)
  , rosGroup_(nullptr)
  , domainIdSpinBox_(nullptr)
  , distroCombo_(nullptr)
  , useTunnelCheckbox_(nullptr)
  , envGroup_(nullptr)
  , envTable_(nullptr)
  , addEnvButton_(nullptr)
  , removeEnvButton_(nullptr)
  , testButton_(nullptr)
  , connectButton_(nullptr)
  , cancelButton_(nullptr)
  , statusLabel_(nullptr)
{
  setWindowTitle(tr("Connect to Remote Robot"));
  setMinimumWidth(500);

  setupUi();
  setupConnections();
  loadProfiles();
  updateUiState();
}

void RemoteConnectionDialog::setupUi() {
  QVBoxLayout* mainLayout = new QVBoxLayout(this);

  // Profile selection
  QHBoxLayout* profileLayout = new QHBoxLayout();

  QLabel* profileLabel = new QLabel(tr("Profile:"));
  profileLayout->addWidget(profileLabel);

  profileCombo_ = new QComboBox();
  profileCombo_->setMinimumWidth(200);
  profileLayout->addWidget(profileCombo_);

  newProfileButton_ = new QPushButton(tr("New"));
  profileLayout->addWidget(newProfileButton_);

  deleteProfileButton_ = new QPushButton(tr("Delete"));
  profileLayout->addWidget(deleteProfileButton_);

  saveProfileButton_ = new QPushButton(tr("Save"));
  profileLayout->addWidget(saveProfileButton_);

  profileLayout->addStretch();
  mainLayout->addLayout(profileLayout);

  // Connection details
  connectionGroup_ = new QGroupBox(tr("Connection Details"));
  QFormLayout* connLayout = new QFormLayout(connectionGroup_);

  profileNameEdit_ = new QLineEdit();
  profileNameEdit_->setPlaceholderText(tr("My Robot"));
  connLayout->addRow(tr("Profile Name:"), profileNameEdit_);

  hostnameEdit_ = new QLineEdit();
  hostnameEdit_->setPlaceholderText(tr("192.168.1.100 or robot.local"));
  connLayout->addRow(tr("Hostname/IP:"), hostnameEdit_);

  portSpinBox_ = new QSpinBox();
  portSpinBox_->setRange(1, 65535);
  portSpinBox_->setValue(22);
  connLayout->addRow(tr("SSH Port:"), portSpinBox_);

  usernameEdit_ = new QLineEdit();
  usernameEdit_->setPlaceholderText(tr("ubuntu"));
  connLayout->addRow(tr("Username:"), usernameEdit_);

  QHBoxLayout* keyLayout = new QHBoxLayout();
  sshKeyEdit_ = new QLineEdit();
  sshKeyEdit_->setPlaceholderText(tr("~/.ssh/id_rsa"));
  keyLayout->addWidget(sshKeyEdit_);

  browseKeyButton_ = new QPushButton(tr("Browse..."));
  keyLayout->addWidget(browseKeyButton_);
  connLayout->addRow(tr("SSH Key:"), keyLayout);

  mainLayout->addWidget(connectionGroup_);

  // ROS settings
  rosGroup_ = new QGroupBox(tr("ROS2 Settings"));
  QFormLayout* rosLayout = new QFormLayout(rosGroup_);

  domainIdSpinBox_ = new QSpinBox();
  domainIdSpinBox_->setRange(0, 232);
  domainIdSpinBox_->setValue(0);
  domainIdSpinBox_->setToolTip(tr("ROS_DOMAIN_ID (0-232)"));
  rosLayout->addRow(tr("Domain ID:"), domainIdSpinBox_);

  distroCombo_ = new QComboBox();
  distroCombo_->addItems({"humble", "iron", "jazzy", "rolling"});
  rosLayout->addRow(tr("ROS Distro:"), distroCombo_);

  useTunnelCheckbox_ = new QCheckBox(tr("Use SSH tunnel for DDS"));
  useTunnelCheckbox_->setChecked(true);
  useTunnelCheckbox_->setToolTip(tr("Forward DDS discovery ports through SSH"));
  rosLayout->addRow(useTunnelCheckbox_);

  mainLayout->addWidget(rosGroup_);

  // Environment variables
  envGroup_ = new QGroupBox(tr("Environment Variables"));
  QVBoxLayout* envLayout = new QVBoxLayout(envGroup_);

  envTable_ = new QTableWidget();
  envTable_->setColumnCount(2);
  envTable_->setHorizontalHeaderLabels({tr("Variable"), tr("Value")});
  envTable_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
  envTable_->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
  envTable_->setMinimumHeight(80);
  envLayout->addWidget(envTable_);

  QHBoxLayout* envButtonLayout = new QHBoxLayout();
  addEnvButton_ = new QPushButton(tr("Add"));
  envButtonLayout->addWidget(addEnvButton_);

  removeEnvButton_ = new QPushButton(tr("Remove"));
  envButtonLayout->addWidget(removeEnvButton_);

  envButtonLayout->addStretch();
  envLayout->addLayout(envButtonLayout);

  mainLayout->addWidget(envGroup_);

  // Status
  statusLabel_ = new QLabel();
  statusLabel_->setStyleSheet("color: gray;");
  mainLayout->addWidget(statusLabel_);

  // Buttons
  QHBoxLayout* buttonLayout = new QHBoxLayout();

  testButton_ = new QPushButton(tr("Test Connection"));
  buttonLayout->addWidget(testButton_);

  buttonLayout->addStretch();

  connectButton_ = new QPushButton(tr("Connect"));
  connectButton_->setDefault(true);
  connectButton_->setStyleSheet("QPushButton { background-color: #4CAF50; color: white; }");
  buttonLayout->addWidget(connectButton_);

  cancelButton_ = new QPushButton(tr("Cancel"));
  buttonLayout->addWidget(cancelButton_);

  mainLayout->addLayout(buttonLayout);
}

void RemoteConnectionDialog::setupConnections() {
  connect(profileCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &RemoteConnectionDialog::onProfileSelected);
  connect(newProfileButton_, &QPushButton::clicked,
          this, &RemoteConnectionDialog::onNewProfileClicked);
  connect(deleteProfileButton_, &QPushButton::clicked,
          this, &RemoteConnectionDialog::onDeleteProfileClicked);
  connect(saveProfileButton_, &QPushButton::clicked,
          this, &RemoteConnectionDialog::onSaveProfileClicked);
  connect(browseKeyButton_, &QPushButton::clicked,
          this, &RemoteConnectionDialog::onBrowseKeyClicked);
  connect(testButton_, &QPushButton::clicked,
          this, &RemoteConnectionDialog::onTestConnectionClicked);
  connect(connectButton_, &QPushButton::clicked,
          this, &RemoteConnectionDialog::onConnectClicked);
  connect(cancelButton_, &QPushButton::clicked,
          this, &QDialog::reject);
  connect(addEnvButton_, &QPushButton::clicked,
          this, &RemoteConnectionDialog::onAddEnvVarClicked);
  connect(removeEnvButton_, &QPushButton::clicked,
          this, &RemoteConnectionDialog::onRemoveEnvVarClicked);

  connect(manager_, &RemoteConnectionManager::connectionTestResult,
          this, &RemoteConnectionDialog::onTestResult);

  // Enable connect button when required fields are filled
  auto updateConnectButton = [this]() {
    updateUiState();
  };

  connect(hostnameEdit_, &QLineEdit::textChanged, this, updateConnectButton);
  connect(usernameEdit_, &QLineEdit::textChanged, this, updateConnectButton);
}

void RemoteConnectionDialog::loadProfiles() {
  profiles_ = manager_->loadProfiles();

  profileCombo_->clear();
  profileCombo_->addItem(tr("(New Profile)"));

  for (const RobotProfile& profile : profiles_) {
    profileCombo_->addItem(profile.name, profile.name);
  }
}

void RemoteConnectionDialog::onProfileSelected(int index) {
  if (index <= 0) {
    // New profile - clear fields
    profileNameEdit_->clear();
    hostnameEdit_->clear();
    portSpinBox_->setValue(22);
    usernameEdit_->clear();
    sshKeyEdit_->clear();
    domainIdSpinBox_->setValue(0);
    distroCombo_->setCurrentIndex(0);
    useTunnelCheckbox_->setChecked(true);
    envTable_->setRowCount(0);
  } else {
    QString profileName = profileCombo_->currentData().toString();
    for (const RobotProfile& profile : profiles_) {
      if (profile.name == profileName) {
        populateFromProfile(profile);
        break;
      }
    }
  }

  updateUiState();
}

void RemoteConnectionDialog::onNewProfileClicked() {
  profileCombo_->setCurrentIndex(0);
  profileNameEdit_->clear();
  profileNameEdit_->setFocus();
}

void RemoteConnectionDialog::onDeleteProfileClicked() {
  QString profileName = profileCombo_->currentData().toString();
  if (profileName.isEmpty()) return;

  int ret = QMessageBox::question(this, tr("Delete Profile"),
      tr("Are you sure you want to delete profile '%1'?").arg(profileName),
      QMessageBox::Yes | QMessageBox::No);

  if (ret == QMessageBox::Yes) {
    manager_->deleteProfile(profileName);
    loadProfiles();
  }
}

void RemoteConnectionDialog::onSaveProfileClicked() {
  RobotProfile profile = buildProfileFromUi();

  if (profile.name.isEmpty()) {
    QMessageBox::warning(this, tr("Invalid Profile"),
                         tr("Please enter a profile name."));
    profileNameEdit_->setFocus();
    return;
  }

  if (profile.hostname.isEmpty()) {
    QMessageBox::warning(this, tr("Invalid Profile"),
                         tr("Please enter a hostname or IP address."));
    hostnameEdit_->setFocus();
    return;
  }

  manager_->saveProfile(profile);
  statusLabel_->setText(tr("Profile '%1' saved.").arg(profile.name));
  loadProfiles();

  // Select the saved profile
  int index = profileCombo_->findData(profile.name);
  if (index >= 0) {
    profileCombo_->setCurrentIndex(index);
  }
}

void RemoteConnectionDialog::onBrowseKeyClicked() {
  QString defaultPath = QDir::homePath() + "/.ssh";
  QString keyPath = QFileDialog::getOpenFileName(this, tr("Select SSH Key"),
                                                  defaultPath,
                                                  tr("All Files (*)"));
  if (!keyPath.isEmpty()) {
    sshKeyEdit_->setText(keyPath);
  }
}

void RemoteConnectionDialog::onTestConnectionClicked() {
  RobotProfile profile = buildProfileFromUi();

  if (profile.hostname.isEmpty() || profile.username.isEmpty()) {
    QMessageBox::warning(this, tr("Missing Information"),
                         tr("Please enter hostname and username."));
    return;
  }

  statusLabel_->setText(tr("Testing connection..."));
  testButton_->setEnabled(false);
  manager_->testConnection(profile);
}

void RemoteConnectionDialog::onConnectClicked() {
  RobotProfile profile = buildProfileFromUi();

  if (profile.hostname.isEmpty() || profile.username.isEmpty()) {
    QMessageBox::warning(this, tr("Missing Information"),
                         tr("Please enter hostname and username."));
    return;
  }

  emit connectionRequested(profile);
  accept();
}

void RemoteConnectionDialog::onTestResult(bool success, const QString& message) {
  testButton_->setEnabled(true);

  if (success) {
    statusLabel_->setStyleSheet("color: green;");
  } else {
    statusLabel_->setStyleSheet("color: red;");
  }
  statusLabel_->setText(message);
}

void RemoteConnectionDialog::onAddEnvVarClicked() {
  int row = envTable_->rowCount();
  envTable_->insertRow(row);
  envTable_->setItem(row, 0, new QTableWidgetItem(""));
  envTable_->setItem(row, 1, new QTableWidgetItem(""));
  envTable_->editItem(envTable_->item(row, 0));
}

void RemoteConnectionDialog::onRemoveEnvVarClicked() {
  QList<QTableWidgetItem*> selected = envTable_->selectedItems();
  if (!selected.isEmpty()) {
    int row = selected.first()->row();
    envTable_->removeRow(row);
  }
}

void RemoteConnectionDialog::populateFromProfile(const RobotProfile& profile) {
  profileNameEdit_->setText(profile.name);
  hostnameEdit_->setText(profile.hostname);
  portSpinBox_->setValue(profile.port);
  usernameEdit_->setText(profile.username);
  sshKeyEdit_->setText(profile.sshKeyPath);
  domainIdSpinBox_->setValue(profile.rosDomainId);
  useTunnelCheckbox_->setChecked(profile.useSshTunnel);

  int distroIndex = distroCombo_->findText(profile.rosDistro);
  if (distroIndex >= 0) {
    distroCombo_->setCurrentIndex(distroIndex);
  }

  // Environment variables
  envTable_->setRowCount(0);
  for (const QString& envVar : profile.environmentVariables) {
    QStringList parts = envVar.split('=');
    if (parts.size() >= 2) {
      int row = envTable_->rowCount();
      envTable_->insertRow(row);
      envTable_->setItem(row, 0, new QTableWidgetItem(parts[0]));
      envTable_->setItem(row, 1, new QTableWidgetItem(parts.mid(1).join('=')));
    }
  }
}

RobotProfile RemoteConnectionDialog::buildProfileFromUi() const {
  RobotProfile profile;
  profile.name = profileNameEdit_->text().trimmed();
  profile.hostname = hostnameEdit_->text().trimmed();
  profile.port = portSpinBox_->value();
  profile.username = usernameEdit_->text().trimmed();
  profile.sshKeyPath = sshKeyEdit_->text().trimmed();
  profile.rosDomainId = domainIdSpinBox_->value();
  profile.rosDistro = distroCombo_->currentText();
  profile.useSshTunnel = useTunnelCheckbox_->isChecked();

  // Environment variables
  for (int row = 0; row < envTable_->rowCount(); ++row) {
    QTableWidgetItem* nameItem = envTable_->item(row, 0);
    QTableWidgetItem* valueItem = envTable_->item(row, 1);
    if (nameItem && valueItem && !nameItem->text().isEmpty()) {
      profile.environmentVariables.append(
          QString("%1=%2").arg(nameItem->text(), valueItem->text()));
    }
  }

  return profile;
}

RobotProfile RemoteConnectionDialog::getProfile() const {
  return buildProfileFromUi();
}

void RemoteConnectionDialog::updateUiState() {
  bool hasSelection = profileCombo_->currentIndex() > 0;
  deleteProfileButton_->setEnabled(hasSelection);

  bool hasRequiredFields = !hostnameEdit_->text().isEmpty() &&
                           !usernameEdit_->text().isEmpty();
  connectButton_->setEnabled(hasRequiredFields);
  testButton_->setEnabled(hasRequiredFields);
}

}  // namespace ros_weaver

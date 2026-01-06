#include "ros_weaver/widgets/qos_editor_dialog.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QFormLayout>
#include <QPushButton>
#include <QDialogButtonBox>
#include <QFont>
#include <QClipboard>
#include <QApplication>

namespace ros_weaver {

QosEditorDialog::QosEditorDialog(QWidget* parent)
  : QDialog(parent)
  , profile_(QosProfile::defaultProfile())
{
  setWindowTitle(tr("QoS Profile Editor"));
  setMinimumWidth(550);
  setupUI();
  loadProfileToUI(profile_);
  updateCodeSnippets();
}

QosEditorDialog::QosEditorDialog(const QosProfile& profile, QWidget* parent)
  : QDialog(parent)
  , profile_(profile)
{
  setWindowTitle(tr("QoS Profile Editor"));
  setMinimumWidth(550);
  setupUI();
  loadProfileToUI(profile_);
  updateCodeSnippets();
}

QosEditorDialog::~QosEditorDialog() {
  delete publisherProfile_;
  delete subscriberProfile_;
}

void QosEditorDialog::setupUI() {
  auto* mainLayout = new QVBoxLayout(this);
  mainLayout->setSpacing(12);

  setupPresetSection(mainLayout);
  setupPoliciesSection(mainLayout);
  setupCompatibilitySection(mainLayout);
  setupCodeSection(mainLayout);

  // Dialog buttons
  auto* buttonBox = new QDialogButtonBox(
    QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
  connect(buttonBox, &QDialogButtonBox::accepted, this, &QDialog::accept);
  connect(buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
  mainLayout->addWidget(buttonBox);
}

void QosEditorDialog::setupPresetSection(QVBoxLayout* layout) {
  auto* presetGroup = new QGroupBox(tr("Preset"));
  auto* presetLayout = new QVBoxLayout(presetGroup);

  auto* comboLayout = new QHBoxLayout();
  comboLayout->addWidget(new QLabel(tr("Profile:")));

  presetCombo_ = new QComboBox();
  presetCombo_->addItem(tr("Default"), static_cast<int>(QosPreset::Default));
  presetCombo_->addItem(tr("Sensor Data"), static_cast<int>(QosPreset::SensorData));
  presetCombo_->addItem(tr("Services"), static_cast<int>(QosPreset::Services));
  presetCombo_->addItem(tr("Parameters"), static_cast<int>(QosPreset::Parameters));
  presetCombo_->addItem(tr("System Default"), static_cast<int>(QosPreset::SystemDefault));
  presetCombo_->addItem(tr("Custom"), static_cast<int>(QosPreset::Custom));
  connect(presetCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &QosEditorDialog::updateFromPreset);
  comboLayout->addWidget(presetCombo_);
  comboLayout->addStretch();
  presetLayout->addLayout(comboLayout);

  presetDescription_ = new QLabel();
  presetDescription_->setWordWrap(true);
  presetDescription_->setStyleSheet("color: #888; font-style: italic;");
  presetLayout->addWidget(presetDescription_);

  layout->addWidget(presetGroup);
}

void QosEditorDialog::setupPoliciesSection(QVBoxLayout* layout) {
  auto* policiesGroup = new QGroupBox(tr("QoS Policies"));
  auto* formLayout = new QFormLayout(policiesGroup);
  formLayout->setSpacing(8);

  // Reliability
  reliabilityCombo_ = new QComboBox();
  reliabilityCombo_->addItem(tr("Reliable"), static_cast<int>(QosReliability::Reliable));
  reliabilityCombo_->addItem(tr("Best Effort"), static_cast<int>(QosReliability::BestEffort));
  reliabilityCombo_->setToolTip(tr("Reliable: Messages guaranteed to be delivered\nBest Effort: Messages may be lost"));
  connect(reliabilityCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &QosEditorDialog::onPolicyChanged);
  formLayout->addRow(tr("Reliability:"), reliabilityCombo_);

  // Durability
  durabilityCombo_ = new QComboBox();
  durabilityCombo_->addItem(tr("Volatile"), static_cast<int>(QosDurability::Volatile));
  durabilityCombo_->addItem(tr("Transient Local"), static_cast<int>(QosDurability::TransientLocal));
  durabilityCombo_->setToolTip(tr("Volatile: Messages not stored\nTransient Local: Messages stored for late joiners"));
  connect(durabilityCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &QosEditorDialog::onPolicyChanged);
  formLayout->addRow(tr("Durability:"), durabilityCombo_);

  // History
  auto* historyLayout = new QHBoxLayout();
  historyCombo_ = new QComboBox();
  historyCombo_->addItem(tr("Keep Last"), static_cast<int>(QosHistory::KeepLast));
  historyCombo_->addItem(tr("Keep All"), static_cast<int>(QosHistory::KeepAll));
  historyCombo_->setToolTip(tr("Keep Last: Only store last N messages\nKeep All: Store all messages"));
  connect(historyCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &QosEditorDialog::onPolicyChanged);
  historyLayout->addWidget(historyCombo_);

  historyDepthSpin_ = new QSpinBox();
  historyDepthSpin_->setRange(1, 10000);
  historyDepthSpin_->setValue(10);
  historyDepthSpin_->setToolTip(tr("Number of messages to keep (for Keep Last)"));
  connect(historyDepthSpin_, QOverload<int>::of(&QSpinBox::valueChanged),
          this, &QosEditorDialog::onPolicyChanged);
  historyLayout->addWidget(new QLabel(tr("Depth:")));
  historyLayout->addWidget(historyDepthSpin_);
  historyLayout->addStretch();
  formLayout->addRow(tr("History:"), historyLayout);

  // Liveliness
  livelinessCombo_ = new QComboBox();
  livelinessCombo_->addItem(tr("Automatic"), static_cast<int>(QosLiveliness::Automatic));
  livelinessCombo_->addItem(tr("Manual by Node"), static_cast<int>(QosLiveliness::ManualByNode));
  livelinessCombo_->addItem(tr("Manual by Topic"), static_cast<int>(QosLiveliness::ManualByTopic));
  livelinessCombo_->setToolTip(tr("How liveliness is asserted"));
  connect(livelinessCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &QosEditorDialog::onPolicyChanged);
  formLayout->addRow(tr("Liveliness:"), livelinessCombo_);

  // Deadline
  deadlineSpin_ = new QSpinBox();
  deadlineSpin_->setRange(-1, 999999);
  deadlineSpin_->setValue(-1);
  deadlineSpin_->setSpecialValueText(tr("Infinite"));
  deadlineSpin_->setSuffix(" ms");
  deadlineSpin_->setToolTip(tr("Expected message arrival interval (-1 = infinite)"));
  connect(deadlineSpin_, QOverload<int>::of(&QSpinBox::valueChanged),
          this, &QosEditorDialog::onPolicyChanged);
  formLayout->addRow(tr("Deadline:"), deadlineSpin_);

  // Lifespan
  lifespanSpin_ = new QSpinBox();
  lifespanSpin_->setRange(-1, 999999);
  lifespanSpin_->setValue(-1);
  lifespanSpin_->setSpecialValueText(tr("Infinite"));
  lifespanSpin_->setSuffix(" ms");
  lifespanSpin_->setToolTip(tr("Message validity duration (-1 = infinite)"));
  connect(lifespanSpin_, QOverload<int>::of(&QSpinBox::valueChanged),
          this, &QosEditorDialog::onPolicyChanged);
  formLayout->addRow(tr("Lifespan:"), lifespanSpin_);

  // Lease Duration
  leaseDurationSpin_ = new QSpinBox();
  leaseDurationSpin_->setRange(-1, 999999);
  leaseDurationSpin_->setValue(-1);
  leaseDurationSpin_->setSpecialValueText(tr("Infinite"));
  leaseDurationSpin_->setSuffix(" ms");
  leaseDurationSpin_->setToolTip(tr("Liveliness lease duration (-1 = infinite)"));
  connect(leaseDurationSpin_, QOverload<int>::of(&QSpinBox::valueChanged),
          this, &QosEditorDialog::onPolicyChanged);
  formLayout->addRow(tr("Lease Duration:"), leaseDurationSpin_);

  layout->addWidget(policiesGroup);
}

void QosEditorDialog::setupCompatibilitySection(QVBoxLayout* layout) {
  compatibilityGroup_ = new QGroupBox(tr("Compatibility Check"));
  auto* compatLayout = new QVBoxLayout(compatibilityGroup_);

  compatibilityStatus_ = new QLabel(tr("No compatibility check configured"));
  compatibilityStatus_->setStyleSheet("font-weight: bold;");
  compatLayout->addWidget(compatibilityStatus_);

  compatibilityDetails_ = new QLabel();
  compatibilityDetails_->setWordWrap(true);
  compatibilityDetails_->setVisible(false);
  compatLayout->addWidget(compatibilityDetails_);

  layout->addWidget(compatibilityGroup_);
}

void QosEditorDialog::setupCodeSection(QVBoxLayout* layout) {
  auto* codeGroup = new QGroupBox(tr("Generated Code"));
  auto* codeLayout = new QVBoxLayout(codeGroup);

  codeTabWidget_ = new QTabWidget();

  // C++ code
  cppCodeEdit_ = new QPlainTextEdit();
  cppCodeEdit_->setReadOnly(true);
  cppCodeEdit_->setFont(QFont("Monospace", 9));
  cppCodeEdit_->setMaximumHeight(120);
  cppCodeEdit_->setStyleSheet(
    "QPlainTextEdit { background-color: #2b2b2b; color: #a9b7c6; }"
  );
  codeTabWidget_->addTab(cppCodeEdit_, "C++");

  // Python code
  pythonCodeEdit_ = new QPlainTextEdit();
  pythonCodeEdit_->setReadOnly(true);
  pythonCodeEdit_->setFont(QFont("Monospace", 9));
  pythonCodeEdit_->setMaximumHeight(120);
  pythonCodeEdit_->setStyleSheet(
    "QPlainTextEdit { background-color: #2b2b2b; color: #a9b7c6; }"
  );
  codeTabWidget_->addTab(pythonCodeEdit_, "Python");

  codeLayout->addWidget(codeTabWidget_);

  // Copy button
  auto* copyLayout = new QHBoxLayout();
  copyLayout->addStretch();
  auto* copyButton = new QPushButton(tr("Copy to Clipboard"));
  connect(copyButton, &QPushButton::clicked, this, [this]() {
    QString code;
    if (codeTabWidget_->currentIndex() == 0) {
      code = cppCodeEdit_->toPlainText();
    } else {
      code = pythonCodeEdit_->toPlainText();
    }
    QApplication::clipboard()->setText(code);
  });
  copyLayout->addWidget(copyButton);
  codeLayout->addLayout(copyLayout);

  layout->addWidget(codeGroup);
}

void QosEditorDialog::loadProfileToUI(const QosProfile& profile) {
  // Block signals to prevent cascading updates
  presetCombo_->blockSignals(true);
  reliabilityCombo_->blockSignals(true);
  durabilityCombo_->blockSignals(true);
  historyCombo_->blockSignals(true);
  historyDepthSpin_->blockSignals(true);
  livelinessCombo_->blockSignals(true);
  deadlineSpin_->blockSignals(true);
  lifespanSpin_->blockSignals(true);
  leaseDurationSpin_->blockSignals(true);

  // Set preset
  int presetIndex = presetCombo_->findData(static_cast<int>(profile.preset));
  if (presetIndex >= 0) {
    presetCombo_->setCurrentIndex(presetIndex);
  }

  // Set policies
  reliabilityCombo_->setCurrentIndex(
    reliabilityCombo_->findData(static_cast<int>(profile.reliability)));
  durabilityCombo_->setCurrentIndex(
    durabilityCombo_->findData(static_cast<int>(profile.durability)));
  historyCombo_->setCurrentIndex(
    historyCombo_->findData(static_cast<int>(profile.history)));
  historyDepthSpin_->setValue(profile.historyDepth);
  livelinessCombo_->setCurrentIndex(
    livelinessCombo_->findData(static_cast<int>(profile.liveliness)));

  // Convert nanoseconds to milliseconds for display
  deadlineSpin_->setValue(profile.deadline == -1 ? -1 : static_cast<int>(profile.deadline / 1000000));
  lifespanSpin_->setValue(profile.lifespan == -1 ? -1 : static_cast<int>(profile.lifespan / 1000000));
  leaseDurationSpin_->setValue(profile.leaseDuration == -1 ? -1 : static_cast<int>(profile.leaseDuration / 1000000));

  // Enable history depth only for KeepLast
  historyDepthSpin_->setEnabled(profile.history == QosHistory::KeepLast);

  // Update preset description
  updateFromPreset(presetCombo_->currentIndex());

  // Unblock signals
  presetCombo_->blockSignals(false);
  reliabilityCombo_->blockSignals(false);
  durabilityCombo_->blockSignals(false);
  historyCombo_->blockSignals(false);
  historyDepthSpin_->blockSignals(false);
  livelinessCombo_->blockSignals(false);
  deadlineSpin_->blockSignals(false);
  lifespanSpin_->blockSignals(false);
  leaseDurationSpin_->blockSignals(false);
}

QosProfile QosEditorDialog::createProfileFromUI() const {
  QosProfile profile;

  profile.preset = static_cast<QosPreset>(presetCombo_->currentData().toInt());
  profile.reliability = static_cast<QosReliability>(reliabilityCombo_->currentData().toInt());
  profile.durability = static_cast<QosDurability>(durabilityCombo_->currentData().toInt());
  profile.history = static_cast<QosHistory>(historyCombo_->currentData().toInt());
  profile.historyDepth = historyDepthSpin_->value();
  profile.liveliness = static_cast<QosLiveliness>(livelinessCombo_->currentData().toInt());

  // Convert milliseconds to nanoseconds
  profile.deadline = deadlineSpin_->value() == -1 ? -1 : deadlineSpin_->value() * 1000000LL;
  profile.lifespan = lifespanSpin_->value() == -1 ? -1 : lifespanSpin_->value() * 1000000LL;
  profile.leaseDuration = leaseDurationSpin_->value() == -1 ? -1 : leaseDurationSpin_->value() * 1000000LL;

  return profile;
}

QosProfile QosEditorDialog::profile() const {
  return createProfileFromUI();
}

void QosEditorDialog::setProfile(const QosProfile& profile) {
  profile_ = profile;
  loadProfileToUI(profile);
  updateCodeSnippets();
}

void QosEditorDialog::setPublisherProfile(const QosProfile& profile) {
  delete publisherProfile_;
  publisherProfile_ = new QosProfile(profile);
  isPublisherMode_ = false;  // We're editing subscriber
  updateCompatibilityCheck();
}

void QosEditorDialog::setSubscriberProfile(const QosProfile& profile) {
  delete subscriberProfile_;
  subscriberProfile_ = new QosProfile(profile);
  isPublisherMode_ = true;  // We're editing publisher
  updateCompatibilityCheck();
}

void QosEditorDialog::setConnectionName(const QString& name) {
  connectionName_ = name;
  if (!name.isEmpty()) {
    setWindowTitle(tr("QoS Profile Editor - %1").arg(name));
  }
}

void QosEditorDialog::updateFromPreset(int index) {
  QosPreset preset = static_cast<QosPreset>(presetCombo_->itemData(index).toInt());

  QString description;
  switch (preset) {
    case QosPreset::Default:
      description = tr("Standard ROS2 default QoS. Reliable, volatile, keep last 10.");
      break;
    case QosPreset::SensorData:
      description = tr("Optimized for high-frequency sensor data. Best effort to avoid blocking, small history depth.");
      break;
    case QosPreset::Services:
      description = tr("For service-like communications. Reliable delivery, volatile (no history needed).");
      break;
    case QosPreset::Parameters:
      description = tr("For parameter updates. Reliable, transient local so late joiners get current values.");
      break;
    case QosPreset::SystemDefault:
      description = tr("Use system default settings.");
      break;
    case QosPreset::Custom:
      description = tr("Custom configuration. Modify policies below.");
      break;
  }
  presetDescription_->setText(description);

  // Only load preset values for non-custom presets
  if (preset != QosPreset::Custom) {
    QosProfile presetProfile;
    switch (preset) {
      case QosPreset::Default: presetProfile = QosProfile::defaultProfile(); break;
      case QosPreset::SensorData: presetProfile = QosProfile::sensorDataProfile(); break;
      case QosPreset::Services: presetProfile = QosProfile::servicesProfile(); break;
      case QosPreset::Parameters: presetProfile = QosProfile::parametersProfile(); break;
      case QosPreset::SystemDefault: presetProfile = QosProfile::systemDefaultProfile(); break;
      default: break;
    }
    loadProfileToUI(presetProfile);
  }

  updateCodeSnippets();
  updateCompatibilityCheck();
}

void QosEditorDialog::onPolicyChanged() {
  // When user changes a policy manually, switch to Custom preset
  if (presetCombo_->currentData().toInt() != static_cast<int>(QosPreset::Custom)) {
    presetCombo_->blockSignals(true);
    presetCombo_->setCurrentIndex(presetCombo_->findData(static_cast<int>(QosPreset::Custom)));
    presetDescription_->setText(tr("Custom configuration. Modify policies below."));
    presetCombo_->blockSignals(false);
  }

  // Enable/disable history depth based on history policy
  historyDepthSpin_->setEnabled(
    static_cast<QosHistory>(historyCombo_->currentData().toInt()) == QosHistory::KeepLast);

  updateCodeSnippets();
  updateCompatibilityCheck();
}

void QosEditorDialog::updateCompatibilityCheck() {
  QosProfile currentProfile = createProfileFromUI();

  if (publisherProfile_) {
    // We're editing subscriber, check against publisher
    QStringList issues = QosProfile::compatibilityIssues(*publisherProfile_, currentProfile);

    if (issues.isEmpty()) {
      compatibilityStatus_->setText(tr("Compatible with publisher"));
      compatibilityStatus_->setStyleSheet("font-weight: bold; color: #4CAF50;");
      compatibilityDetails_->setVisible(false);
    } else {
      compatibilityStatus_->setText(tr("Compatibility warnings:"));
      compatibilityStatus_->setStyleSheet("font-weight: bold; color: #FF9800;");
      compatibilityDetails_->setText("- " + issues.join("\n- "));
      compatibilityDetails_->setStyleSheet("color: #FF9800;");
      compatibilityDetails_->setVisible(true);
    }
  } else if (subscriberProfile_) {
    // We're editing publisher, check against subscriber
    QStringList issues = QosProfile::compatibilityIssues(currentProfile, *subscriberProfile_);

    if (issues.isEmpty()) {
      compatibilityStatus_->setText(tr("Compatible with subscriber"));
      compatibilityStatus_->setStyleSheet("font-weight: bold; color: #4CAF50;");
      compatibilityDetails_->setVisible(false);
    } else {
      compatibilityStatus_->setText(tr("Compatibility warnings:"));
      compatibilityStatus_->setStyleSheet("font-weight: bold; color: #FF9800;");
      compatibilityDetails_->setText("- " + issues.join("\n- "));
      compatibilityDetails_->setStyleSheet("color: #FF9800;");
      compatibilityDetails_->setVisible(true);
    }
  } else {
    compatibilityStatus_->setText(tr("Configure publisher/subscriber QoS to check compatibility"));
    compatibilityStatus_->setStyleSheet("font-weight: bold; color: #888;");
    compatibilityDetails_->setVisible(false);
  }
}

void QosEditorDialog::updateCodeSnippets() {
  QosProfile currentProfile = createProfileFromUI();
  cppCodeEdit_->setPlainText(currentProfile.toCppCode());
  pythonCodeEdit_->setPlainText(currentProfile.toPythonCode());
}

}  // namespace ros_weaver

// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2025, ROS Weaver Contributors

#include "ros_weaver/wizards/mcp_server_wizard.hpp"
#include "ros_weaver/core/mcp_manager.hpp"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QLabel>
#include <QLineEdit>
#include <QTextEdit>
#include <QComboBox>
#include <QCheckBox>
#include <QSpinBox>
#include <QListWidget>
#include <QGroupBox>
#include <QJsonArray>
#include <QDir>

namespace ros_weaver {

// Page IDs
enum {
    Page_Intro,
    Page_Type,
    Page_BasicConfig,
    Page_Settings,
    Page_Summary
};

// ============================================================================
// MCPServerWizard
// ============================================================================

MCPServerWizard::MCPServerWizard(QWidget* parent)
    : QWizard(parent)
{
    setWindowTitle("Add MCP Server");
    setWizardStyle(QWizard::ModernStyle);
    setMinimumSize(600, 500);

    setupPages();

    // Set default button text
    setButtonText(QWizard::FinishButton, "Create Server");
}

MCPServerWizard::~MCPServerWizard() = default;

void MCPServerWizard::setupPages() {
    setPage(Page_Intro, new MCPServerIntroPage(this));
    setPage(Page_Type, new MCPServerTypePage(this));
    setPage(Page_BasicConfig, new MCPServerBasicConfigPage(this));
    setPage(Page_Settings, new MCPServerSettingsPage(this));
    setPage(Page_Summary, new MCPServerSummaryPage(this));
}

void MCPServerWizard::setSuggestedConfig(const MCPServerConfig& config, const QString& reason) {
    resultConfig_ = config;
    suggestionReason_ = reason;

    // Update intro page
    auto* introPage = qobject_cast<MCPServerIntroPage*>(page(Page_Intro));
    if (introPage) {
        introPage->setSuggestionReason(reason);
    }

    // Pre-select type
    auto* typePage = qobject_cast<MCPServerTypePage*>(page(Page_Type));
    if (typePage) {
        typePage->setSelectedType(config.serverType);
    }
}

void MCPServerWizard::accept() {
    // Build final config from wizard fields
    resultConfig_.id = QUuid::createUuid();
    resultConfig_.serverType = field("serverType").toString();
    resultConfig_.name = field("serverName").toString();
    resultConfig_.description = field("serverDescription").toString();
    resultConfig_.enabled = field("serverEnabled").toBool();
    resultConfig_.autoStart = field("serverAutoStart").toBool();

    // Get settings from settings page
    auto* settingsPage = qobject_cast<MCPServerSettingsPage*>(page(Page_Settings));
    if (settingsPage) {
        resultConfig_.settings = settingsPage->getSettings();
    }

    QWizard::accept();
}

// ============================================================================
// MCPServerIntroPage
// ============================================================================

MCPServerIntroPage::MCPServerIntroPage(QWidget* parent)
    : QWizardPage(parent)
{
    setTitle("Welcome to the MCP Server Wizard");
    setSubTitle("This wizard will help you configure a new MCP (Model Context Protocol) server.");

    auto* layout = new QVBoxLayout(this);

    introLabel_ = new QLabel(this);
    introLabel_->setWordWrap(true);
    introLabel_->setText(
        "MCP servers provide tools and resources that the Local AI can use to interact with "
        "your ROS 2 system and other data sources.\n\n"
        "Available server types include:\n"
        "  - ROS Logs: Monitor and search ROS 2 log messages\n"
        "  - ROS Topics: List, inspect, and echo topic data\n"
        "  - ROSbag: Check recording status and bag metadata\n"
        "  - ROS Control: Monitor ros2_control controllers\n"
        "  - System Stats: CPU, memory, disk, and temperature monitoring\n"
        "  - Files: File system access within allowed paths"
    );
    layout->addWidget(introLabel_);

    // Suggestion group (hidden by default)
    suggestionGroup_ = new QGroupBox("AI Suggestion", this);
    suggestionGroup_->setVisible(false);

    auto* suggestionLayout = new QVBoxLayout(suggestionGroup_);
    suggestionLabel_ = new QLabel(suggestionGroup_);
    suggestionLabel_->setWordWrap(true);
    suggestionLabel_->setStyleSheet("color: #4a90d9; font-style: italic;");
    suggestionLayout->addWidget(suggestionLabel_);

    layout->addWidget(suggestionGroup_);
    layout->addStretch();
}

void MCPServerIntroPage::setSuggestionReason(const QString& reason) {
    suggestionReason_ = reason;
}

void MCPServerIntroPage::initializePage() {
    if (!suggestionReason_.isEmpty()) {
        suggestionLabel_->setText(suggestionReason_);
        suggestionGroup_->setVisible(true);
    }
}

// ============================================================================
// MCPServerTypePage
// ============================================================================

MCPServerTypePage::MCPServerTypePage(QWidget* parent)
    : QWizardPage(parent)
{
    setTitle("Select Server Type");
    setSubTitle("Choose the type of MCP server you want to add.");

    auto* layout = new QVBoxLayout(this);

    typeList_ = new QListWidget(this);
    typeList_->setSelectionMode(QAbstractItemView::SingleSelection);

    // Populate with available types
    MCPManager& manager = MCPManager::instance();
    for (const QString& type : manager.availableServerTypes()) {
        auto* item = new QListWidgetItem(typeList_);
        item->setData(Qt::UserRole, type);

        // Get description and format display
        QString desc = manager.serverTypeDescription(type);
        MCPServerConfig defaultConfig = manager.defaultConfigForType(type);
        item->setText(QString("%1\n  %2").arg(defaultConfig.name).arg(desc));

        // Set icon based on type
        if (type.startsWith("ros")) {
            item->setIcon(QIcon::fromTheme("ros"));
        } else if (type == "system_stats") {
            item->setIcon(QIcon::fromTheme("utilities-system-monitor"));
        } else if (type == "files") {
            item->setIcon(QIcon::fromTheme("folder"));
        }
    }

    layout->addWidget(typeList_, 1);

    // Description area
    auto* descGroup = new QGroupBox("Description", this);
    auto* descLayout = new QVBoxLayout(descGroup);
    descriptionLabel_ = new QLabel(descGroup);
    descriptionLabel_->setWordWrap(true);
    descLayout->addWidget(descriptionLabel_);
    layout->addWidget(descGroup);

    connect(typeList_, &QListWidget::currentItemChanged,
            this, &MCPServerTypePage::onTypeSelectionChanged);

    registerField("serverType", this, "selectedType");
}

void MCPServerTypePage::initializePage() {
    if (typeList_->count() > 0 && typeList_->currentRow() < 0) {
        typeList_->setCurrentRow(0);
    }
}

bool MCPServerTypePage::validatePage() {
    return typeList_->currentItem() != nullptr;
}

int MCPServerTypePage::nextId() const {
    return Page_BasicConfig;
}

QString MCPServerTypePage::selectedType() const {
    if (auto* item = typeList_->currentItem()) {
        return item->data(Qt::UserRole).toString();
    }
    return QString();
}

void MCPServerTypePage::setSelectedType(const QString& type) {
    for (int i = 0; i < typeList_->count(); ++i) {
        if (typeList_->item(i)->data(Qt::UserRole).toString() == type) {
            typeList_->setCurrentRow(i);
            break;
        }
    }
}

void MCPServerTypePage::onTypeSelectionChanged() {
    QString type = selectedType();
    if (type.isEmpty()) {
        descriptionLabel_->clear();
        return;
    }

    MCPManager& manager = MCPManager::instance();
    MCPServerConfig config = manager.defaultConfigForType(type);

    QString desc = QString("<b>%1</b><br><br>%2<br><br>")
        .arg(config.name)
        .arg(config.description);

    // Add tool list for this type
    desc += "<b>Available Tools:</b><ul>";

    auto server = manager.createServer(config);
    if (server) {
        for (const MCPTool& tool : server->availableTools()) {
            desc += QString("<li><b>%1</b>: %2</li>").arg(tool.name).arg(tool.description);
        }
    }
    desc += "</ul>";

    descriptionLabel_->setText(desc);
    emit completeChanged();
}

// ============================================================================
// MCPServerBasicConfigPage
// ============================================================================

MCPServerBasicConfigPage::MCPServerBasicConfigPage(QWidget* parent)
    : QWizardPage(parent)
{
    setTitle("Basic Configuration");
    setSubTitle("Configure the basic settings for your MCP server.");

    auto* layout = new QFormLayout(this);

    nameEdit_ = new QLineEdit(this);
    nameEdit_->setPlaceholderText("Enter a name for this server");
    layout->addRow("Name:", nameEdit_);

    descriptionEdit_ = new QTextEdit(this);
    descriptionEdit_->setPlaceholderText("Optional description");
    descriptionEdit_->setMaximumHeight(80);
    layout->addRow("Description:", descriptionEdit_);

    enabledCheck_ = new QCheckBox("Server is enabled", this);
    enabledCheck_->setChecked(true);
    layout->addRow("", enabledCheck_);

    autoStartCheck_ = new QCheckBox("Start automatically when application launches", this);
    autoStartCheck_->setChecked(true);
    layout->addRow("", autoStartCheck_);

    registerField("serverName*", nameEdit_);
    registerField("serverDescription", descriptionEdit_, "plainText");
    registerField("serverEnabled", enabledCheck_);
    registerField("serverAutoStart", autoStartCheck_);
}

void MCPServerBasicConfigPage::initializePage() {
    // Pre-fill with default name based on type
    QString type = field("serverType").toString();
    MCPManager& manager = MCPManager::instance();
    MCPServerConfig defaultConfig = manager.defaultConfigForType(type);

    if (nameEdit_->text().isEmpty()) {
        nameEdit_->setText(defaultConfig.name);
    }
    if (descriptionEdit_->toPlainText().isEmpty()) {
        descriptionEdit_->setPlainText(defaultConfig.description);
    }
}

bool MCPServerBasicConfigPage::validatePage() {
    return !nameEdit_->text().trimmed().isEmpty();
}

// ============================================================================
// MCPServerSettingsPage
// ============================================================================

MCPServerSettingsPage::MCPServerSettingsPage(QWidget* parent)
    : QWizardPage(parent)
{
    setTitle("Server Settings");
    setSubTitle("Configure server-specific settings.");

    auto* layout = new QVBoxLayout(this);

    settingsContainer_ = new QWidget(this);
    settingsLayout_ = new QFormLayout(settingsContainer_);
    layout->addWidget(settingsContainer_);

    layout->addStretch();
}

void MCPServerSettingsPage::initializePage() {
    QString type = field("serverType").toString();

    if (type != currentServerType_) {
        clearSettings();
        currentServerType_ = type;

        if (type == "ros_logs") {
            setupROSLogsSettings();
        } else if (type == "ros_topics") {
            setupROSTopicsSettings();
        } else if (type == "rosbag") {
            setupRosbagSettings();
        } else if (type == "ros_control") {
            setupROSControlSettings();
        } else if (type == "system_stats") {
            setupSystemStatsSettings();
        } else if (type == "files") {
            setupFilesSettings();
        }
    }
}

bool MCPServerSettingsPage::validatePage() {
    return true;
}

void MCPServerSettingsPage::clearSettings() {
    // Remove all widgets from layout
    QLayoutItem* item;
    while ((item = settingsLayout_->takeAt(0)) != nullptr) {
        if (item->widget()) {
            item->widget()->deleteLater();
        }
        delete item;
    }

    // Reset pointers
    maxLinesSpinBox_ = nullptr;
    severityCombo_ = nullptr;
    maxMessageHistorySpinBox_ = nullptr;
    defaultBagPathEdit_ = nullptr;
    controllerManagerEdit_ = nullptr;
    updateIntervalSpinBox_ = nullptr;
    includeTempsCheck_ = nullptr;
    allowedPathsEdit_ = nullptr;
    maxFileSizeSpinBox_ = nullptr;
}

void MCPServerSettingsPage::setupROSLogsSettings() {
    maxLinesSpinBox_ = new QSpinBox(this);
    maxLinesSpinBox_->setRange(100, 100000);
    maxLinesSpinBox_->setValue(1000);
    maxLinesSpinBox_->setSuffix(" lines");
    settingsLayout_->addRow("Max log lines:", maxLinesSpinBox_);

    severityCombo_ = new QComboBox(this);
    severityCombo_->addItems({"debug", "info", "warn", "error", "fatal"});
    severityCombo_->setCurrentIndex(0);
    settingsLayout_->addRow("Minimum severity:", severityCombo_);
}

void MCPServerSettingsPage::setupROSTopicsSettings() {
    maxMessageHistorySpinBox_ = new QSpinBox(this);
    maxMessageHistorySpinBox_->setRange(10, 10000);
    maxMessageHistorySpinBox_->setValue(100);
    maxMessageHistorySpinBox_->setSuffix(" messages");
    settingsLayout_->addRow("Max message history:", maxMessageHistorySpinBox_);
}

void MCPServerSettingsPage::setupRosbagSettings() {
    defaultBagPathEdit_ = new QLineEdit(this);
    defaultBagPathEdit_->setPlaceholderText("Leave empty for current directory");
    settingsLayout_->addRow("Default bag path:", defaultBagPathEdit_);
}

void MCPServerSettingsPage::setupROSControlSettings() {
    controllerManagerEdit_ = new QLineEdit(this);
    controllerManagerEdit_->setText("/controller_manager");
    settingsLayout_->addRow("Controller manager:", controllerManagerEdit_);
}

void MCPServerSettingsPage::setupSystemStatsSettings() {
    updateIntervalSpinBox_ = new QSpinBox(this);
    updateIntervalSpinBox_->setRange(100, 60000);
    updateIntervalSpinBox_->setValue(1000);
    updateIntervalSpinBox_->setSuffix(" ms");
    settingsLayout_->addRow("Update interval:", updateIntervalSpinBox_);

    includeTempsCheck_ = new QCheckBox("Include temperature readings", this);
    includeTempsCheck_->setChecked(true);
    settingsLayout_->addRow("", includeTempsCheck_);
}

void MCPServerSettingsPage::setupFilesSettings() {
    allowedPathsEdit_ = new QTextEdit(this);
    allowedPathsEdit_->setPlaceholderText("One path per line");
    allowedPathsEdit_->setPlainText(QDir::homePath());
    allowedPathsEdit_->setMaximumHeight(100);
    settingsLayout_->addRow("Allowed paths:", allowedPathsEdit_);

    maxFileSizeSpinBox_ = new QSpinBox(this);
    maxFileSizeSpinBox_->setRange(1, 100);
    maxFileSizeSpinBox_->setValue(10);
    maxFileSizeSpinBox_->setSuffix(" MB");
    settingsLayout_->addRow("Max file size:", maxFileSizeSpinBox_);
}

QJsonObject MCPServerSettingsPage::getSettings() const {
    QJsonObject settings;

    if (currentServerType_ == "ros_logs") {
        if (maxLinesSpinBox_) settings["maxLines"] = maxLinesSpinBox_->value();
        if (severityCombo_) settings["severityFilter"] = severityCombo_->currentText();
    }
    else if (currentServerType_ == "ros_topics") {
        if (maxMessageHistorySpinBox_) settings["maxMessageHistory"] = maxMessageHistorySpinBox_->value();
    }
    else if (currentServerType_ == "rosbag") {
        if (defaultBagPathEdit_) settings["defaultPath"] = defaultBagPathEdit_->text();
    }
    else if (currentServerType_ == "ros_control") {
        if (controllerManagerEdit_) settings["controllerManager"] = controllerManagerEdit_->text();
    }
    else if (currentServerType_ == "system_stats") {
        if (updateIntervalSpinBox_) settings["updateIntervalMs"] = updateIntervalSpinBox_->value();
        if (includeTempsCheck_) settings["includeTemperatures"] = includeTempsCheck_->isChecked();
    }
    else if (currentServerType_ == "files") {
        if (allowedPathsEdit_) {
            QJsonArray paths;
            for (const QString& line : allowedPathsEdit_->toPlainText().split('\n', Qt::SkipEmptyParts)) {
                paths.append(line.trimmed());
            }
            settings["allowedPaths"] = paths;
        }
        if (maxFileSizeSpinBox_) {
            settings["maxFileSize"] = maxFileSizeSpinBox_->value() * 1024 * 1024;  // Convert to bytes
        }
    }

    return settings;
}

// ============================================================================
// MCPServerSummaryPage
// ============================================================================

MCPServerSummaryPage::MCPServerSummaryPage(QWidget* parent)
    : QWizardPage(parent)
{
    setTitle("Summary");
    setSubTitle("Review your MCP server configuration.");

    auto* layout = new QVBoxLayout(this);

    summaryLabel_ = new QLabel(this);
    summaryLabel_->setWordWrap(true);
    summaryLabel_->setTextFormat(Qt::RichText);
    layout->addWidget(summaryLabel_);

    layout->addStretch();
}

void MCPServerSummaryPage::initializePage() {
    QString type = field("serverType").toString();
    QString name = field("serverName").toString();
    QString desc = field("serverDescription").toString();
    bool enabled = field("serverEnabled").toBool();
    bool autoStart = field("serverAutoStart").toBool();

    MCPManager& manager = MCPManager::instance();
    QString typeDesc = manager.serverTypeDescription(type);

    QString summary = QString(
        "<h3>Server Configuration</h3>"
        "<table>"
        "<tr><td><b>Name:</b></td><td>%1</td></tr>"
        "<tr><td><b>Type:</b></td><td>%2</td></tr>"
        "<tr><td><b>Description:</b></td><td>%3</td></tr>"
        "<tr><td><b>Enabled:</b></td><td>%4</td></tr>"
        "<tr><td><b>Auto-start:</b></td><td>%5</td></tr>"
        "</table>"
        "<br><p>Click <b>Create Server</b> to add this MCP server.</p>"
    ).arg(name)
     .arg(typeDesc)
     .arg(desc.isEmpty() ? "(none)" : desc)
     .arg(enabled ? "Yes" : "No")
     .arg(autoStart ? "Yes" : "No");

    summaryLabel_->setText(summary);
}

}  // namespace ros_weaver

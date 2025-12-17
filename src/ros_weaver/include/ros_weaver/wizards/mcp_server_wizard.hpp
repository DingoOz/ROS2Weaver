// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2025, ROS Weaver Contributors

#pragma once

#include <QWizard>
#include <QWizardPage>
#include "ros_weaver/core/mcp_types.hpp"

class QLabel;
class QLineEdit;
class QTextEdit;
class QComboBox;
class QCheckBox;
class QSpinBox;
class QListWidget;
class QGroupBox;
class QFormLayout;

namespace ros_weaver {

/**
 * @brief Wizard for creating and configuring new MCP servers
 */
class MCPServerWizard : public QWizard {
    Q_OBJECT

public:
    explicit MCPServerWizard(QWidget* parent = nullptr);
    ~MCPServerWizard() override;

    MCPServerConfig resultConfig() const { return resultConfig_; }

    // Pre-populate with AI suggestion
    void setSuggestedConfig(const MCPServerConfig& config, const QString& reason);

public slots:
    void accept() override;

private:
    void setupPages();

    MCPServerConfig resultConfig_;
    QString suggestionReason_;
};

/**
 * @brief Introduction page with AI suggestion display
 */
class MCPServerIntroPage : public QWizardPage {
    Q_OBJECT

public:
    explicit MCPServerIntroPage(QWidget* parent = nullptr);

    void setSuggestionReason(const QString& reason);
    void initializePage() override;

private:
    QLabel* introLabel_;
    QGroupBox* suggestionGroup_;
    QLabel* suggestionLabel_;
    QString suggestionReason_;
};

/**
 * @brief Server type selection page
 */
class MCPServerTypePage : public QWizardPage {
    Q_OBJECT

public:
    explicit MCPServerTypePage(QWidget* parent = nullptr);

    void initializePage() override;
    bool validatePage() override;
    int nextId() const override;

    QString selectedType() const;
    void setSelectedType(const QString& type);

private slots:
    void onTypeSelectionChanged();

private:
    QListWidget* typeList_;
    QLabel* descriptionLabel_;
};

/**
 * @brief Basic configuration page (name, description, enabled, autostart)
 */
class MCPServerBasicConfigPage : public QWizardPage {
    Q_OBJECT

public:
    explicit MCPServerBasicConfigPage(QWidget* parent = nullptr);

    void initializePage() override;
    bool validatePage() override;

private:
    QLineEdit* nameEdit_;
    QTextEdit* descriptionEdit_;
    QCheckBox* enabledCheck_;
    QCheckBox* autoStartCheck_;
};

/**
 * @brief Server-specific settings page
 */
class MCPServerSettingsPage : public QWizardPage {
    Q_OBJECT

public:
    explicit MCPServerSettingsPage(QWidget* parent = nullptr);

    void initializePage() override;
    bool validatePage() override;

    QJsonObject getSettings() const;

private:
    void setupROSLogsSettings();
    void setupROSTopicsSettings();
    void setupRosbagSettings();
    void setupROSControlSettings();
    void setupSystemStatsSettings();
    void setupFilesSettings();
    void clearSettings();

    QFormLayout* settingsLayout_;
    QWidget* settingsContainer_;
    QString currentServerType_;

    // ROS Logs settings
    QSpinBox* maxLinesSpinBox_ = nullptr;
    QComboBox* severityCombo_ = nullptr;

    // ROS Topics settings
    QSpinBox* maxMessageHistorySpinBox_ = nullptr;

    // Rosbag settings
    QLineEdit* defaultBagPathEdit_ = nullptr;

    // ROS Control settings
    QLineEdit* controllerManagerEdit_ = nullptr;

    // System Stats settings
    QSpinBox* updateIntervalSpinBox_ = nullptr;
    QCheckBox* includeTempsCheck_ = nullptr;

    // Files settings
    QTextEdit* allowedPathsEdit_ = nullptr;
    QSpinBox* maxFileSizeSpinBox_ = nullptr;
};

/**
 * @brief Summary and confirmation page
 */
class MCPServerSummaryPage : public QWizardPage {
    Q_OBJECT

public:
    explicit MCPServerSummaryPage(QWidget* parent = nullptr);

    void initializePage() override;

private:
    QLabel* summaryLabel_;
};

}  // namespace ros_weaver

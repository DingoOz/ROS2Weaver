#ifndef ROS_WEAVER_WIDGETS_PRESET_SELECTOR_WIDGET_HPP
#define ROS_WEAVER_WIDGETS_PRESET_SELECTOR_WIDGET_HPP

#include <QWidget>
#include <QComboBox>
#include <QPushButton>
#include <QLabel>

namespace ros_weaver {

class ParameterPreset;

/**
 * @brief Toolbar widget for quick preset selection and management
 *
 * Features:
 * - Dropdown for quick preset switching
 * - Visual indicator of active preset
 * - Quick access to preset editor
 * - Export/Import buttons
 */
class PresetSelectorWidget : public QWidget {
  Q_OBJECT

public:
  explicit PresetSelectorWidget(QWidget* parent = nullptr);
  ~PresetSelectorWidget() override;

  void refreshPresets();

signals:
  void presetActivated(const QString& presetName);
  void editPresetsRequested();
  void exportPresetRequested();
  void importPresetRequested();

public slots:
  void onActivePresetChanged(ParameterPreset* preset);

private slots:
  void onComboIndexChanged(int index);
  void onEditClicked();
  void onExportClicked();
  void onImportClicked();

private:
  void setupUI();
  void updateIndicator();

  QLabel* label_;
  QComboBox* presetCombo_;
  QLabel* indicator_;
  QPushButton* editButton_;
  QPushButton* exportButton_;
  QPushButton* importButton_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_WIDGETS_PRESET_SELECTOR_WIDGET_HPP

#include "ros_weaver/widgets/theme_editor_dialog.hpp"
#include "ros_weaver/core/theme_manager.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QScrollArea>
#include <QColorDialog>
#include <QMessageBox>
#include <QDialogButtonBox>
#include <QPainter>

namespace ros_weaver {

// ColorEditWidget implementation
ColorEditWidget::ColorEditWidget(const QString& name, const QColor& color, QWidget* parent)
    : QWidget(parent)
    , name_(name)
    , color_(color)
{
  QHBoxLayout* layout = new QHBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(8);

  // Color preview
  colorPreview_ = new QFrame(this);
  colorPreview_->setFixedSize(24, 24);
  colorPreview_->setFrameShape(QFrame::Box);
  colorPreview_->setAutoFillBackground(true);
  layout->addWidget(colorPreview_);

  // Color label with hex value
  colorLabel_ = new QLabel(this);
  colorLabel_->setMinimumWidth(80);
  layout->addWidget(colorLabel_);

  // Name label
  QLabel* nameLabel = new QLabel(name, this);
  nameLabel->setMinimumWidth(120);
  layout->addWidget(nameLabel, 1);

  // Pick button
  pickButton_ = new QPushButton(tr("Pick..."), this);
  pickButton_->setFixedWidth(60);
  connect(pickButton_, &QPushButton::clicked, this, &ColorEditWidget::onPickColor);
  layout->addWidget(pickButton_);

  setColor(color);
}

void ColorEditWidget::setColor(const QColor& color) {
  color_ = color;

  // Update preview
  QPalette pal = colorPreview_->palette();
  pal.setColor(QPalette::Window, color);
  colorPreview_->setPalette(pal);

  // Update label
  colorLabel_->setText(color.name());
}

void ColorEditWidget::onPickColor() {
  QColor newColor = QColorDialog::getColor(color_, this, tr("Select Color"));
  if (newColor.isValid() && newColor != color_) {
    setColor(newColor);
    emit colorChanged(name_, newColor);
  }
}

// ThemeEditorDialog implementation
ThemeEditorDialog::ThemeEditorDialog(QWidget* parent)
    : QDialog(parent)
    , hasUnsavedChanges_(false)
{
  setWindowTitle(tr("Theme Editor"));
  setMinimumSize(700, 500);
  setupUI();
  refreshThemeList();
}

void ThemeEditorDialog::setupUI() {
  QHBoxLayout* mainLayout = new QHBoxLayout(this);
  mainLayout->setContentsMargins(12, 12, 12, 12);
  mainLayout->setSpacing(12);

  // Left panel - theme list
  QVBoxLayout* leftLayout = new QVBoxLayout();

  QLabel* themesLabel = new QLabel(tr("Themes"), this);
  themesLabel->setStyleSheet("font-weight: bold;");
  leftLayout->addWidget(themesLabel);

  themeList_ = new QListWidget(this);
  themeList_->setMaximumWidth(180);
  connect(themeList_, &QListWidget::itemClicked,
          this, &ThemeEditorDialog::onThemeSelected);
  leftLayout->addWidget(themeList_, 1);

  // Theme list buttons
  QHBoxLayout* themeButtonsLayout = new QHBoxLayout();
  newButton_ = new QPushButton(tr("New"), this);
  newButton_->setEnabled(false);  // Not supported with current ThemeManager
  themeButtonsLayout->addWidget(newButton_);

  duplicateButton_ = new QPushButton(tr("Duplicate"), this);
  duplicateButton_->setEnabled(false);  // Not supported
  themeButtonsLayout->addWidget(duplicateButton_);

  deleteButton_ = new QPushButton(tr("Delete"), this);
  deleteButton_->setEnabled(false);  // Not supported
  themeButtonsLayout->addWidget(deleteButton_);

  leftLayout->addLayout(themeButtonsLayout);
  mainLayout->addLayout(leftLayout);

  // Right panel - color editors
  QVBoxLayout* rightLayout = new QVBoxLayout();

  // Theme name (read-only for built-in themes)
  QHBoxLayout* nameLayout = new QHBoxLayout();
  nameLayout->addWidget(new QLabel(tr("Theme Name:"), this));
  themeNameEdit_ = new QLineEdit(this);
  themeNameEdit_->setReadOnly(true);
  nameLayout->addWidget(themeNameEdit_, 1);
  rightLayout->addLayout(nameLayout);

  // Colors scroll area
  QGroupBox* colorsGroup = new QGroupBox(tr("Theme Colors (Read Only)"), this);
  QVBoxLayout* colorsLayout = new QVBoxLayout(colorsGroup);

  QScrollArea* scrollArea = new QScrollArea(this);
  scrollArea->setWidgetResizable(true);
  scrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

  colorsPanel_ = new QWidget();
  QVBoxLayout* colorsPanelLayout = new QVBoxLayout(colorsPanel_);
  colorsPanelLayout->setSpacing(4);

  // Add color viewers for theme colors
  QStringList colorNames = {
      "Primary", "PrimaryHover", "Background", "Surface",
      "TextPrimary", "TextSecondary", "Success", "Warning", "Error",
      "Border", "CanvasBackground", "CanvasGridMinor", "CanvasGridMajor",
      "InputBackground", "InputBorder"
  };

  for (const QString& name : colorNames) {
    ColorEditWidget* editor = new ColorEditWidget(name, Qt::gray, colorsPanel_);
    editor->findChild<QPushButton*>()->setEnabled(false);  // Read-only
    connect(editor, &ColorEditWidget::colorChanged,
            this, &ThemeEditorDialog::onColorChanged);
    colorsPanelLayout->addWidget(editor);
    colorEditors_.append(editor);
  }

  colorsPanelLayout->addStretch();
  scrollArea->setWidget(colorsPanel_);
  colorsLayout->addWidget(scrollArea);
  rightLayout->addWidget(colorsGroup, 1);

  // Accent color (editable)
  QGroupBox* accentGroup = new QGroupBox(tr("Accent Color (Customizable)"), this);
  QHBoxLayout* accentLayout = new QHBoxLayout(accentGroup);

  QLabel* accentLabel = new QLabel(tr("Accent:"), this);
  accentLayout->addWidget(accentLabel);

  QFrame* accentPreview = new QFrame(this);
  accentPreview->setFixedSize(32, 32);
  accentPreview->setFrameShape(QFrame::Box);
  accentPreview->setObjectName("accentPreview");
  accentLayout->addWidget(accentPreview);

  QPushButton* accentButton = new QPushButton(tr("Change Accent Color"), this);
  connect(accentButton, &QPushButton::clicked, this, [this]() {
    ThemeManager& tm = ThemeManager::instance();
    QColor current = tm.accentColor();
    QColor newColor = QColorDialog::getColor(current, this, tr("Select Accent Color"));
    if (newColor.isValid()) {
      tm.setAccentColor(newColor);
      updatePreview();
    }
  });
  accentLayout->addWidget(accentButton);

  QPushButton* resetAccentButton = new QPushButton(tr("Reset"), this);
  connect(resetAccentButton, &QPushButton::clicked, this, [this]() {
    ThemeManager::instance().resetAccentColor();
    updatePreview();
  });
  accentLayout->addWidget(resetAccentButton);

  accentLayout->addStretch();
  rightLayout->addWidget(accentGroup);

  // Preview
  QGroupBox* previewGroup = new QGroupBox(tr("Preview"), this);
  QVBoxLayout* previewLayout = new QVBoxLayout(previewGroup);

  previewFrame_ = new QFrame(this);
  previewFrame_->setFixedHeight(80);
  previewFrame_->setFrameShape(QFrame::StyledPanel);
  previewLayout->addWidget(previewFrame_);

  previewLabel_ = new QLabel(tr("Theme preview will appear here"), this);
  previewLabel_->setAlignment(Qt::AlignCenter);
  previewLayout->addWidget(previewLabel_);

  rightLayout->addWidget(previewGroup);

  // Buttons
  QHBoxLayout* buttonsLayout = new QHBoxLayout();
  buttonsLayout->addStretch();

  applyButton_ = new QPushButton(tr("Apply Theme"), this);
  connect(applyButton_, &QPushButton::clicked, this, &ThemeEditorDialog::onApply);
  buttonsLayout->addWidget(applyButton_);

  QPushButton* closeButton = new QPushButton(tr("Close"), this);
  connect(closeButton, &QPushButton::clicked, this, &QDialog::accept);
  buttonsLayout->addWidget(closeButton);

  rightLayout->addLayout(buttonsLayout);
  mainLayout->addLayout(rightLayout, 1);

  // Hide unused buttons
  revertButton_ = nullptr;
  saveButton_ = nullptr;
}

void ThemeEditorDialog::refreshThemeList() {
  themeList_->clear();

  // Add built-in themes
  QStringList themes = {"Dark", "Light", "High Contrast"};
  ThemeManager& tm = ThemeManager::instance();
  QString currentThemeName = ThemeManager::themeName(tm.currentTheme());

  for (const QString& theme : themes) {
    QListWidgetItem* item = new QListWidgetItem(theme, themeList_);
    if (theme == currentThemeName) {
      item->setIcon(QIcon::fromTheme("emblem-default"));
    }
  }

  // Select current theme
  for (int i = 0; i < themeList_->count(); ++i) {
    if (themeList_->item(i)->text() == currentThemeName) {
      themeList_->setCurrentRow(i);
      loadThemeColors(currentThemeName);
      break;
    }
  }
}

void ThemeEditorDialog::onThemeSelected(QListWidgetItem* item) {
  if (!item) return;
  loadThemeColors(item->text());
}

void ThemeEditorDialog::loadThemeColors(const QString& themeName) {
  currentThemeName_ = themeName;
  themeNameEdit_->setText(themeName);

  ThemeManager& tm = ThemeManager::instance();

  // Map color names to actual colors from the current theme
  QMap<QString, QColor> themeColors;
  themeColors["Primary"] = tm.primaryColor();
  themeColors["PrimaryHover"] = tm.primaryHoverColor();
  themeColors["Background"] = tm.backgroundColor();
  themeColors["Surface"] = tm.surfaceColor();
  themeColors["TextPrimary"] = tm.textPrimaryColor();
  themeColors["TextSecondary"] = tm.textSecondaryColor();
  themeColors["Success"] = tm.successColor();
  themeColors["Warning"] = tm.warningColor();
  themeColors["Error"] = tm.errorColor();
  themeColors["Border"] = tm.borderColor();
  themeColors["CanvasBackground"] = tm.canvasBackgroundColor();
  themeColors["CanvasGridMinor"] = tm.canvasGridMinorColor();
  themeColors["CanvasGridMajor"] = tm.canvasGridMajorColor();
  themeColors["InputBackground"] = tm.inputBackgroundColor();
  themeColors["InputBorder"] = tm.inputBorderColor();

  for (ColorEditWidget* editor : colorEditors_) {
    QColor color = themeColors.value(editor->colorName(), Qt::gray);
    editor->setColor(color);
    currentColors_[editor->colorName()] = color;
  }

  updatePreview();
}

void ThemeEditorDialog::onNewTheme() {
  // Not supported with current ThemeManager
  QMessageBox::information(this, tr("Not Supported"),
      tr("Creating custom themes is not currently supported. "
         "You can customize the accent color instead."));
}

void ThemeEditorDialog::onDuplicateTheme() {
  // Not supported
}

void ThemeEditorDialog::onDeleteTheme() {
  // Not supported
}

void ThemeEditorDialog::onColorChanged(const QString& name, const QColor& color) {
  currentColors_[name] = color;
  hasUnsavedChanges_ = true;
  updatePreview();
}

void ThemeEditorDialog::onApply() {
  ThemeManager& tm = ThemeManager::instance();

  // Apply the selected theme
  if (currentThemeName_ == "Dark") {
    tm.setTheme(Theme::Dark);
  } else if (currentThemeName_ == "Light") {
    tm.setTheme(Theme::Light);
  } else if (currentThemeName_ == "High Contrast") {
    tm.setTheme(Theme::HighContrast);
  }

  emit themeApplied(currentThemeName_);
}

void ThemeEditorDialog::onSave() {
  // Not supported
}

void ThemeEditorDialog::onRevert() {
  loadThemeColors(currentThemeName_);
}

void ThemeEditorDialog::saveCurrentTheme() {
  // Not supported
}

void ThemeEditorDialog::updatePreview() {
  ThemeManager& tm = ThemeManager::instance();

  QColor bgColor = tm.backgroundColor();
  QColor textColor = tm.textPrimaryColor();
  QColor surfaceColor = tm.surfaceColor();
  QColor accentColor = tm.accentColor();

  // Update preview frame
  QPalette pal = previewFrame_->palette();
  pal.setColor(QPalette::Window, bgColor);
  previewFrame_->setPalette(pal);
  previewFrame_->setAutoFillBackground(true);

  // Update preview label
  QString styleSheet = QString(
      "QLabel { "
      "color: %1; "
      "background-color: %2; "
      "border: 2px solid %3; "
      "padding: 8px; "
      "border-radius: 4px; "
      "}"
  ).arg(textColor.name(), surfaceColor.name(), accentColor.name());

  previewLabel_->setStyleSheet(styleSheet);
  previewLabel_->setText(tr("Sample Node Block\nAccent: %1").arg(accentColor.name()));

  // Update accent preview
  QFrame* accentPreview = findChild<QFrame*>("accentPreview");
  if (accentPreview) {
    QPalette accentPal = accentPreview->palette();
    accentPal.setColor(QPalette::Window, accentColor);
    accentPreview->setPalette(accentPal);
    accentPreview->setAutoFillBackground(true);
  }
}

QString ThemeEditorDialog::themeName() const {
  return themeNameEdit_->text();
}

void ThemeEditorDialog::setThemeName(const QString& name) {
  themeNameEdit_->setText(name);
}

void ThemeEditorDialog::loadTheme(const QString& themeName) {
  for (int i = 0; i < themeList_->count(); ++i) {
    if (themeList_->item(i)->text() == themeName) {
      themeList_->setCurrentRow(i);
      loadThemeColors(themeName);
      break;
    }
  }
}

void ThemeEditorDialog::createNewTheme() {
  onNewTheme();
}

}  // namespace ros_weaver
